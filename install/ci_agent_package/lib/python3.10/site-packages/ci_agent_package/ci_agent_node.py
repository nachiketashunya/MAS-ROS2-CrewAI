import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import logging
import time
import random
import json
from rclpy.executors import MultiThreadedExecutor

sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from ci_agent_package.ci_agent import CIAgent
from common_interfaces.src.graph_manager import GraphManager

class CIAgentNode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')

        print("Initialization")

        self.publisher = self.create_publisher(String, 'ci_to_bi_navigation_request', 10)
        self.subscriber = self.create_subscription(
            String,
            'bi_to_ci_navigation_response',
            self.receive_navigation_response,
            10
        )
        
        self.ci_agent = CIAgent(
            publisher=self.publisher,
            subscriber=self.subscriber
        )

        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
        self.navigation_path = None
        self.navigation_response_received = threading.Event()

    def receive_navigation_response(self, msg):
        print("Response Received: ", msg)
        self.ci_agent.update_navigation_path(msg.data)
        self.navigation_response_received.set()

    def write_pos_to_json(self, new_pos):
        data = {'position': new_pos}
        with open(self.json_file, "w") as f:
            json.dump(data, f)
        print(f"Updated JSON file with new position: {new_pos}")

    # def write_pos_to_json(agent_id, new_pos, navigation_path=None):
    #     # Load the existing JSON data
    #     json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
    #     try:
    #         with open(json_file, "r") as f:
    #             data = json.load(f)
    #     except FileNotFoundError:
    #         data = {"agents": {}}

    #     # Update the agent's position and navigation path
    #     if agent_id not in data["agents"]:
    #         data["agents"][agent_id] = {"position": new_pos, "navigation_path": []}
        
    #     data["agents"][agent_id]["position"] = new_pos
        
    #     if navigation_path:
    #         data["agents"][agent_id]["navigation_path"] = navigation_path

    #     # Write the updated data back to the JSON file
    #     with open(json_file, "w") as f:
    #         json.dump(data, f, indent=4)
        
    #     print(f"Updated JSON file for {agent_id}: Position={new_pos}, Navigation Path={navigation_path}")

    def guide_visitor(self, visitor_id, building_id):
        tasks = self.ci_agent.define_tasks()
        
        
        result1 = tasks[0].execute(inputs={'visitor_id': visitor_id, 'building_id': building_id, 'navigation_path': None})
        result2 = tasks[1].execute(inputs={'visitor_id': visitor_id, 'building_id': building_id, 'navigation_path': result1})
            

def main(args=None):
    rclpy.init(args=args)

    node = CIAgentNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    visitor_id = "V001"
    building_id = "Building A"

    logging.info(f"Starting the Escort Task with visitor {visitor_id} and building {building_id}")

    # Run the guide_visitor method in a separate thread
    guide_thread = threading.Thread(target=node.guide_visitor, args=(visitor_id, building_id))
    guide_thread.start()

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