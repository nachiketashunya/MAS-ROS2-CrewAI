import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bi_agent_package.bi_agent import BIAgent
from bi_agent_package.tasks.handle_navigation_task import HandleNavigationTask
import threading
import sys
import logging
import time
import random
import json

class BIAgentNode(Node):
    def __init__(self):
        super().__init__('bi_agent_node')

        print("BI Agent Initialization")

        # Create a BI agent instance
        self.bi_agent = BIAgent()

        # Initialize the navigation task
        self.navigation_task = HandleNavigationTask()

        # JSON file for coordinates
        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/bi_positions.json"

        # Publisher to respond to CI agent with navigation paths
        self.navigation_response_publisher = self.create_publisher(String, 'bi_to_ci_navigation_response', 10)

        # Subscription to receive navigation requests from CI agent
        self.navigation_request_subscription = self.create_subscription(
            String,
            'ci_to_bi_navigation_request',
            self.handle_navigation_request,
            10
        )

    def handle_navigation_request(self, msg):
        """
        Handle navigation requests from the CI agent.
        """
        request_data = msg.data
        self.get_logger().info(f"Received navigation request: {request_data}")

        # Execute the navigation task
        response = self.navigation_task.execute(
            request_data=request_data,
            send_navigation_response=self.send_navigation_response
        )
        self.get_logger().info(response)

    def send_navigation_response(self, response_data):
        """
        Send the navigation response back to the CI agent.
        """
        msg = String()
        msg.data = response_data
        self.navigation_response_publisher.publish(msg)
        self.get_logger().info(f"Sent navigation response: {msg.data}")

    def write_pos_to_json(self, new_pos):
        data = {
            'position': new_pos
        }
        
        try:
            with open(self.json_file, "w") as f:
                json.dump(data, f)
            print(f"Updated JSON file with new position: {new_pos}")
        except Exception as e:
            print(f"Error writing to JSON file: {e}")

    # Function to update agent position in the building
    def update_agent_position(self):
        building_nodes = ['Building A Entrance', 'Room A1', 'Room A2', 'Room A3']
        
        while rclpy.ok():
            try:
                # Update agent position within the building
                random_pos = random.choice(building_nodes)
                self.write_pos_to_json(random_pos)

                logging.debug(f"BI Agent position updated to {random_pos}")
                time.sleep(5)  # Update every 5 seconds
            except Exception as e:
                logging.error(f"Error updating agent position: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize the BI agent node
    node = BIAgentNode()

    # Start the position update thread
    try:
        node.update_agent_position()
    except Exception as e:
        logging.error(f"Error occurred: {e}")

    # Spin to keep the node alive and handle communication
    rclpy.spin(node)

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()