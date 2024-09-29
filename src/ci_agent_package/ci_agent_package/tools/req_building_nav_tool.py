from crewai_tools import BaseTool
import json
from std_msgs.msg import String
import time

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

class RequestBuildingNavigationTool(BaseTool):
    def __init__(self, publisher, subscriber):
        super().__init__(name="Request building navigation tool", description="Tool for requesting BI for building navigation")
        self._publisher = publisher
        self._bi_response = None
        self._logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

    def send_request(self, agent_id, visitor_id, building_id, room):   
        msg = String()
        # msg.data = req_data_str
        msg.data = f"{building_id}==>{room}==>{agent_id}==>{visitor_id}"

        self._publisher.publish(msg)
        self._logger.info(f"{agent_id} published navigation request for {visitor_id} to {building_id}.")

    def set_bi_response(self, bi_response):
        self._bi_response = bi_response

    def _run(self, agent_id, visitor_id, building_id, room):
        self._bi_response = None

        while self._bi_response is None:
            self.send_request(agent_id, visitor_id, building_id, room)
            self._logger.info(f"{agent_id} is waiting for the BI response")
            time.sleep(60)
        
        return self._bi_response