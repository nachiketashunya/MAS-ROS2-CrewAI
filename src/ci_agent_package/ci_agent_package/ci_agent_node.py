import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ci_agent_package.ci_agent import CIAgent
import time
from filelock import FileLock


import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.graph_manager import GraphManager

import concurrent.futures
from threading import Lock, Thread

class CIAgentNode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')

        self.manager = GraphManager()
        self.lock = Lock()

        self.logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")
        self.ci_agents = []
        self.callback_groups = []
        self.main_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.lock = threading.Lock()
        self.json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        for i in range(3):
            agent_callback_group = ReentrantCallbackGroup()
            publisher = self.create_publisher(
                String, 'ci_to_bi_navigation_request', 10, callback_group=agent_callback_group)
            subscriber = self.create_subscription(
                String, 'bi_to_ci_navigation_response', self.receive_navigation_response, 10, callback_group=self.main_callback_group)
            
            vi_indicator = self.create_publisher(String, 'vi_indicator', 10, callback_group=agent_callback_group)

            ci_agent = CIAgent(
                publisher=publisher,
                subscriber=subscriber,
                vi_indicator=vi_indicator,
                graph_manager=self.manager,
                agent_id=f"ci_agent_{i+1}",
                callback_group=agent_callback_group
            )
            self.ci_agents.append(ci_agent)
            self.callback_groups.append(agent_callback_group)

        self.confirm_vis_req = self.create_publisher(
            String, 'ci_to_vi_confirm_res', 10, callback_group=self.main_callback_group)
        self.vis_req_subscriber = self.create_subscription(
            String, 'vi_to_ci_request', self.handle_vis_req, 10, callback_group=self.main_callback_group)
        
        self.terminate_subscriber = self.create_subscription(String, 'termination', self.handle_termination, 10, callback_group=self.main_callback_group)

        self.bi_response_received = threading.Event()
        self.last_processed_response = {}
        print("CI node Initialization Complete")
        
    def start_graph_manager(self):
        # Create a thread to run the manager
        manager_thread = threading.Thread(target=self.run_manager)
        manager_thread.daemon = True  # Optional: set to True if you want the thread to close when the main program exits
        manager_thread.start()

    def run_manager(self):
        # Call the manager's run method
        self.manager.run()

    def handle_termination(self, msg):
        msg = msg.data

        self.logger.info(f'Shutdown signal received. Shutting Down CI Node')
        
        for ci_agent in self.ci_agents:
            self.logger.info(f'{ci_agent.agent_id} Total Visitors: {ci_agent.total_visitors} Total Violations: {ci_agent.total_violations}')
        
        rclpy.shutdown()  # Shuts down the ROS 2 system

    def handle_vis_req(self, msg):
        data = msg.data.split("==>")
        vis_id, building, room, host, meeting_time = data[0], data[1], data[2], data[3], data[4]
       
        self.logger.info(f"{vis_id} wants to visit {host} for {meeting_time}seconds")

        with self.lock:
            agent = self.avail_ci_agent()
        
        if agent:
            agent.set_unavailable()
            agent.total_visitors += 1
            agent.guide_duration = int(meeting_time) + 15


            self.logger.info(f"{agent.agent_id} is guiding {vis_id} to {host}")
            
            confirmation_msg = String()
            confirmation_msg.data = f"A==>{vis_id}==>{agent.agent_id}"
            self.confirm_vis_req.publish(confirmation_msg)

            self.logger.info(f"{agent.agent_id} sent confirmation message to {vis_id}")
            
            agent_thread = threading.Thread(target=self.guide_visitor_thread, args=(agent, vis_id, building, room, host, meeting_time))
            agent_thread.start()
        else:
            msg = f"UA==>{vis_id}==>{None}"
        
            confirmation_msg = String()
            confirmation_msg.data = msg
            self.confirm_vis_req.publish(confirmation_msg)

    def guide_visitor_thread(self, agent, vis_id, building, room, host, meeting_time):
        try:
            agent.guide_visitor(vis_id, building, room, host, meeting_time)
        finally:
            with self.lock:
                agent.set_available()

    def receive_navigation_response(self, msg):
        response = msg.data.split('->')
        bi_response = ""

        with self.lock:
            # Check if this response has already been processed
            response_key = f"{response[1]}_{response[2]}"
            if response_key in self.last_processed_response:
                self.logger.info(f"Duplicate response received for {response[1]} and {response[2]}. Ignoring.")
                return

            for ci_agent in self.ci_agents:
                if ci_agent.agent_id == response[1]:
                    if response[0] == "Unauthorized":
                        self.logger.info(f"{response[2]} is not authorized to meet the host! Returning to base")
                        bi_response = "Unauthorized"
                    elif response[0] == "Unavailable":
                        self.logger.info(f"Host is unavailable to meet {response[2]}! Returning to base")
                        bi_response = "Unavailable"
                    elif response[0] == "OOS":
                        self.logger.info(f"BI Agent is Out of Service! Returning to base")
                        bi_response = "OOS"
                    else:
                        path = response[4:].copy()
                        self.logger.info(f"{ci_agent.agent_id} received navigation response")
                        bi_response = "->".join(path)

                    ci_agent.update_bi_response(bi_response)
                    self.bi_response_received.set()

                    # Update position in JSON file
                    self.update_positions(ci_agent.agent_id, response[2], bi_response, response)

                    # Mark this response as processed
                    self.last_processed_response[response_key] = time.time()

    def update_positions(self, ci_agent_id, visitor_id, bi_response, response):
        campus_info = "/home/nachiketa/dup_auto_ass1/src/data/campus_info.json"
        with open(campus_info, "r") as f:
            data = json.load(f)

        if bi_response in ["Unauthorized", "Unavailable", "OOS"]:
            for path in reversed(data['buildings'][response[3]]):
                self.manager.update_agent_position(ci_agent_id, path)
                self.manager.update_agent_position(visitor_id, path)
                time.sleep(1)
        
            self.manager.update_agent_position(ci_agent_id, "CI Lobby")
            self.manager.update_agent_position(visitor_id,"VI Lobby")
           

    def avail_ci_agent(self):
        for ci_agent in self.ci_agents:
            if ci_agent.is_available():
                return ci_agent
        return None

def main(args=None):
    rclpy.init(args=args)
    node = CIAgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.start_graph_manager()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()