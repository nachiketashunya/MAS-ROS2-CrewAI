from crewai_tools import BaseTool
import rclpy
from std_msgs.msg import String
import time

class RequestBuildingNavigationTool(BaseTool):
    name: str = "Request building navigation tool"
    description: str = "Tool for requesting BI for building navigation"

    def send_request(self, visitor_id, building_id):        
        msg = String()
        msg.data = f"Requesting navigation for visitor {visitor_id} inside building {building_id}."
        self.publisher.publish(msg)
        print(f"Published navigation request for visitor {visitor_id} to BI agent for building {building_id}.")

    def receive_response(self, msg):
        # Callback for the subscriber to handle BI agent's response
        self.navigation_path = msg.data
        print(f"Received navigation response from BI agent: {msg.data}")

    def _run(self, publisher, subscriber, visitor_id, building_id):
        self.navigation_path = None
        self.publisher = publisher
        self.subscriber = subscriber

        # Send navigation request
        self.send_request(visitor_id, building_id)

        # Wait for a response from BI agent
        print("Waiting for navigation response...")
        while self.navigation_path is None:
            rclpy.spin_once(self.subscriber)
        
        return self.navigation_path