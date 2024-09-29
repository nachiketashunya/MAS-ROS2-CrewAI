import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ci_agent_package.ci_agent import CIAgent
import time

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import ret_logger

class CIAgentNode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')

        self.logger = ret_logger()

        self.ci_agents = []
        self.callback_groups = []  # Store callback groups for agents

        # Callback group for the main node itself
        self.main_callback_group = ReentrantCallbackGroup()

        # Create and manage multiple CIAgents
        for i in range(3):
            # Create callback group for each agent
            agent_callback_group = ReentrantCallbackGroup()

            # Publisher and Subscriber for each agent
            publisher = self.create_publisher(
                String, 'ci_to_bi_navigation_request', 10, callback_group=agent_callback_group)
            subscriber = self.create_subscription(
                String, 'bi_to_ci_navigation_response', self.receive_navigation_response, 10, callback_group=agent_callback_group)

            # Create the CIAgent object
            ci_agent = CIAgent(
                publisher=publisher,
                subscriber=subscriber,
                agent_id=f"ci_agent_{i+1}",
                callback_group=agent_callback_group
            )
            self.ci_agents.append(ci_agent)
            self.callback_groups.append(agent_callback_group)  # Keep track of callback groups

        # Main node publishers and subscribers for visitor requests
        self.confirm_vis_req = self.create_publisher(
            String, 'ci_to_vi_confirm_res', 10, callback_group=self.main_callback_group)
        self.vis_req_subscriber = self.create_subscription(
            String, 'vi_to_ci_request', self.handle_vis_req, 10, callback_group=self.main_callback_group)

        self.navigation_response_received = threading.Event()
        print("CI node Initialization Complete")

    def handle_vis_req(self, msg):
        """
        Handle visitor requests and assign an available CI agent.
        """
        data = msg.data.split("==>")
        vis_id, building, room, host, meeting_time = data[0], data[1], data[2], data[3], data[4]

        self.logger.info(f"{vis_id} wants to visit {host} in {room} for {meeting_time}seconds")

        # Find an available CI agent
        agent = self.avail_ci_agent()
        
        if agent:
            agent.set_unavailable()

            self.logger.info(f"{agent.agent_id} is guiding {vis_id} to {host}")
            
            confirmation_msg = String()
            confirmation_msg.data = f"{vis_id}==>{agent.agent_id}"
            self.confirm_vis_req.publish(confirmation_msg)

            self.logger.info(f"{agent.agent_id} sent confirmation message to {vis_id}")
            
            # Guide visitor using agent in its own thread
            agent_thread = threading.Thread(target=agent.guide_visitor, args=(vis_id, building, room, host))
            agent_thread.start()
        else:
            self.logger.info("No available CI Agent")

    def receive_navigation_response(self, msg):
        """
        Receive navigation response from BI agent and update respective CI agent.
        """

        path = msg.data.split('->')
        agent_id = path.pop()

        self.logger.info(f"{agent_id} received navigation reponse")
        # Find the agent with the correct ID and update its navigation path
        for ci_agent in self.ci_agents:
            if ci_agent.agent_id == agent_id:
                nav_path = "->".join(path)
                ci_agent.update_navigation_path(nav_path)
                self.navigation_response_received.set()
                break

    def avail_ci_agent(self):
        """
        Find an available CI agent.
        """
        for ci_agent in self.ci_agents:
            if ci_agent.is_available():
                return ci_agent
        return None

def main(args=None):
    rclpy.init(args=args)

    # Initialize the CI agent node (manager)
    node = CIAgentNode()

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