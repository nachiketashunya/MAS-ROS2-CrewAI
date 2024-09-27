from crewai_tools import BaseTool
import json
from std_msgs.msg import String
import time

class RequestBuildingNavigationTool(BaseTool):
    def __init__(self, publisher, subscriber):
        super().__init__(name="Request building navigation tool", description="Tool for requesting BI for building navigation")
        self._publisher = publisher
        self._navigation_path = None

    def send_request(self, visitor_id, building_id):   
        req_data = {
            'building_id': building_id
        }     

        req_data_str = json.dumps(req_data)
        msg = String()
        # msg.data = req_data_str
        msg.data = "Building A"

        self._publisher.publish(msg)
        print(f"Published navigation request for visitor {visitor_id} to BI agent for building {building_id}.")

    def set_navigation_path(self, navigation_path):
        print("Nav Path: ", navigation_path)
        self._navigation_path = navigation_path

    def _run(self, visitor_id, building_id):
        self._navigation_path = None

        # Send navigation request
        self.send_request(visitor_id, building_id)

        # Wait for a response from BI agent
        print("Waiting for navigation response...")
        while self._navigation_path is None:
            print("Waiting")
            time.sleep(30)
            self.send_request(visitor_id, building_id)
            print(f"Resent the navigation request to BI Agent")
        
        return self._navigation_path