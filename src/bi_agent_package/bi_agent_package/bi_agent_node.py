import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bi_agent_package.bi_agent import BIAgent
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import random

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

class BIAgentNode(Node):
    def __init__(self):
        super().__init__('bi_agent_node')

        self.logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

        self.bi_agents = []
        self.callback_groups = []

        # Initialize 3 BI agents with callback groups
        for i in ['A', 'B', 'C', 'D']:
            agent_callback_group = ReentrantCallbackGroup()

            bi_agent = BIAgent(agent_id=f"bi_agent_{i}")
            self.bi_agents.append(bi_agent)
            self.callback_groups.append(agent_callback_group)

        print("BI Agent Node Initialized")

        # Publisher to respond to CI agent with navigation paths
        self.navigation_response_publisher = self.create_publisher(
            String, 
            'bi_to_ci_navigation_response', 
            10
        )

        # Subscription to receive navigation requests from CI agent
        self.navigation_request_subscription = self.create_subscription(
            String, 
            'ci_to_bi_navigation_request', 
            self.handle_navigation_request, 
            10
        )

        self.terminate_subscriber = self.create_subscription(String, 'termination', self.handle_termination, 10)

    def handle_termination(self, msg):
        msg = msg.data

        self.logger.info(f'Shutdown signal received. Shutting Down CI Node')
        
        for bi_agent in self.bi_agents:
            self.logger.info(f'{bi_agent.agent_id} Total Visitors: {bi_agent.total_cis} Total Violations: {bi_agent.total_violations}')
        
        rclpy.shutdown()  # Shuts down the ROS 2 system

    def handle_navigation_request(self, msg):
        """
        Handle navigation requests from CI agents.
        """
        data = msg.data.split("==>")

        building_id, room_id, ci_agent_id, visitor_id, meeting_time = data[0], data[1], data[2], data[3], data[4]

        self.logger.info(f"Received navigation request from {ci_agent_id}")

        # Find an available BI agent to handle the request
        bi_agent = self.get_bia(building_id)

        building_info = "/home/nachiketa/dup_auto_ass1/src/data/building_info.json"
    
        with open(building_info, 'r') as file:
            data = json.load(file)
    
        info = data["buildings"][building_id]["rooms"][room_id]

        if building_id == room_id:
            # Fetch the navigation path using BI agent tools
            path = info['path']

            path.insert(0, "All OK")
            path.insert(1, ci_agent_id)
            path.insert(2, visitor_id)
            path.insert(3, building_id)

            # Format and send the response back to the CI agent
            ret_path = "->".join(path)
            self.send_navigation_response(ret_path)   
            self.logger.info(f"Navigation response sent to {ci_agent_id}")

            bi_agent.set_oos()
            self.logger.info(f"{bi_agent.agent_id} is Out of Service for {meeting_time}seconds")

            bi_agent.oos_duration = max(int(meeting_time) + random.randint(-20, 20), 0)

            if bi_agent.oos_duration > int(meeting_time):
                bi_agent.total_violations += 1

        else:
            if bi_agent.is_oos:
                self.send_navigation_response(f"OOS->{ci_agent_id}->{visitor_id}->{building_id}")

            elif visitor_id not in info['authorized']:
                self.send_navigation_response(f"Unauthorized->{ci_agent_id}->{visitor_id}->{building_id}")
            
            elif info['available'] == "False":
                self.send_navigation_response(f"Unavailable->{ci_agent_id}->{visitor_id}->{building_id}")
            else:
                # Fetch the navigation path using BI agent tools
                path = info['path']

                path.insert(0, "All OK")
                path.insert(1, ci_agent_id)
                path.insert(2, visitor_id)
                path.insert(3, building_id)

                # Format and send the response back to the CI agent
                ret_path = "->".join(path)
                self.send_navigation_response(ret_path)   
                self.logger.info(f"Navigation reponse sent to {ci_agent_id}")

                bi_agent.total_cis += 1
                self.logger.info(f"Total CIs Guided by {bi_agent.agent_id}: {bi_agent.total_cis}")

    def send_navigation_response(self, response_data):
        """
        Send the navigation response back to the CI agent.
        """
        msg = String()
        msg.data = response_data
        self.navigation_response_publisher.publish(msg)

    def get_bia(self, building_id):
        """
        Find an available BI agent.
        """

        buil_info = building_id.split(" ")
        bia = f"bi_agent_{buil_info[1]}"

        for bi_agent in self.bi_agents:
            if bi_agent.agent_id == bia:
                return bi_agent

def main(args=None):
    rclpy.init(args=args)

    # Initialize the BI agent node
    node = BIAgentNode()

    # MultiThreadedExecutor for handling node callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

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