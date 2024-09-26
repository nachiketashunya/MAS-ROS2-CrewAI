import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ci_agent_package.ci_agent import CIAgent  # Import the updated CI Agent class
import threading
import sys
import logging
import time
import random
import json

sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.graph_manager import GraphManager  # Path to graph_manager

class CIAgentNode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')

        print("Initialization")

        # Create a CI agent instance from the CI agent class (now uses custom tools)
        self.ci_agent = CIAgent(
            publisher=self.create_publisher(String, 'ci_to_bi_navigation_request', 10),
            subscriber=self.create_subscription(
                String,
                'bi_to_ci_navigation_response',
                self.receive_navigation_response,
                10
            )
        )

        # JSON file for coordinates
        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"

        # State to track the received navigation response
        self.navigation_path = None

    def receive_navigation_response(self, msg):
        """
        Handle the response from the BI agent.
        This response provides the internal path to the host.
        """
        self.navigation_path = msg.data
        self.get_logger().info(f'Received navigation response: {msg.data}')

    def write_pos_to_json(self, new_pos):
        data = {'position': new_pos}
        json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
        with open(json_file, "w") as f:
            json.dump(data, f)
        print(f"Updated JSON file with new position: {new_pos}")

    def update_agent_position(self):
        """
        Function to update agent position periodically.
        """
        graph_manager = GraphManager()
        while rclpy.ok():
            try:
                # Update agent position based on current ROS node state
                all_nodes = list(graph_manager.campus_graph.nodes())
                random_pos = random.choice(all_nodes)
                self.write_pos_to_json(random_pos)
                logging.debug(f"Agent position updated to {random_pos}")
                time.sleep(5)  # Update every 5 seconds
            except Exception as e:
                logging.error(f"Error updating agent position: {e}")

    def guide_visitor(self, visitor_id, building_id):
        """
        Start the process to guide the visitor to the building and then to the host.
        This function uses the CI agent and its custom tools.
        """
        try:
            # Phase 1: Guide visitor to the building entrance
            result = self.ci_agent.guide_visitor(visitor_id, building_id)
            self.get_logger().info(result)

        except Exception as e:
            self.get_logger().error(f"Error during guidance: {e}")
            raise e

def main(args=None):
    rclpy.init(args=args)

    # Initialize the CI agent node
    node = CIAgentNode()

    # Simulate a navigation request for a visitor
    visitor_id = "V001"
    building_id = "B001"

    logging.info(f"Starting the Escort Task with visitor {visitor_id} and building {building_id}")

    try:
        # Guide the visitor using the CI agent
        node.guide_visitor(visitor_id=visitor_id, building_id=building_id)

        # Optionally, update the agent's position on the campus graph
        node.update_agent_position()

    except Exception as e:
        logging.error(f"Error occurred: {e}")

    # Spin to keep the node alive and handle communication
    rclpy.spin(node)

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()