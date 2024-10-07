from crewai_tools import BaseTool
from std_msgs.msg import String
import threading
import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

class RequestBuildingNavigationTool(BaseTool):
    def __init__(self, publisher, subscriber):
        super().__init__(name="Request building navigation tool", description="Tool for requesting BI for building navigation")
        self._publisher = publisher
        self._bi_response = None
        self._logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")
        self._response_received = threading.Event()

    def send_request(self, agent_id, visitor_id, building_id, room, meeting_time):   
        msg = String()
        msg.data = f"{building_id}==>{room}==>{agent_id}==>{visitor_id}==>{meeting_time}"

        self._publisher.publish(msg)
        self._logger.info(f"{agent_id} published navigation request for {visitor_id} to {building_id}.")

    def set_bi_response(self, bi_response):
        self._bi_response = bi_response
        self._response_received.set()

    def _run(self, agent_id, visitor_id, building_id, room, meeting_time):
        self._bi_response = None
        self._response_received.clear()

        max_attempts = 3
        attempt = 0

        while attempt < max_attempts:
            self.send_request(agent_id, visitor_id, building_id, room, meeting_time)
            self._logger.info(f"{agent_id} is waiting for the BI response (Attempt {attempt + 1})")

            if self._response_received.wait(timeout=60):
                return self._bi_response
            
            attempt += 1

        self._logger.error(f"{agent_id} did not receive a response after {max_attempts} attempts")
        return "Timed Out"