from crewai_tools import BaseTool
import time
import json

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

class NavigateToBuildingTool(BaseTool):
    def __init__(self, publisher):
        super().__init__(name="Navigate to Building Tool", description="Tool to navigate")
        self._publisher = publisher
        self._logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

    def _run(self, agent_id, visitor_id, building_id, graph_manager):
        campus_info = "/home/nachiketa/dup_auto_ass1/src/data/campus_info.json"
        with open(campus_info, "r") as f:
            data = json.load(f)

        self._logger.info(f"{agent_id} is navigating to {building_id} with {visitor_id}")

        import threading
        lock = threading.Lock()

        with lock:
            for path in data['buildings'][building_id]:
                # Update the information
                graph_manager.update_agent_position(agent_id, path)
                graph_manager.update_agent_position(visitor_id, path)
                time.sleep(1)