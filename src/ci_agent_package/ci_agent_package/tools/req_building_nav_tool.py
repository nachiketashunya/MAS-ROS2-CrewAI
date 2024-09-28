from crewai_tools import BaseTool
import json
from std_msgs.msg import String
import time

class RequestBuildingNavigationTool(BaseTool):
    def __init__(self, publisher, subscriber):
        super().__init__(name="Request building navigation tool", description="Tool for requesting BI for building navigation")
        self._publisher = publisher
        self._navigation_path = None

    def send_request(self, agent_id, visitor_id, building_id, room):   
        msg = String()
        # msg.data = req_data_str
        msg.data = f"{building_id}==>{room}==>{agent_id}"

        self._publisher.publish(msg)
        print(f"{agent_id} published navigation request for {visitor_id} to {building_id}.")

    def set_navigation_path(self, navigation_path):
        print("Nav Path: ", navigation_path)
        self._navigation_path = navigation_path

    def _run(self, agent_id, visitor_id, building_id, room):
        self._navigation_path = None

        # Wait for a response from BI agent
        print("Waiting for navigation response...")
        while self._navigation_path is None:
            self.send_request(agent_id, visitor_id, building_id, room)
            print(f"Sent the navigation request to BI Agent")
            print("Waiting for the navigation response")
            time.sleep(60)
        
        return self._navigation_path