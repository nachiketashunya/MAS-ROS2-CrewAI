from crewai_tools import BaseTool
from std_msgs.msg import String
import time
from filelock import FileLock
import json

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.logger_config import get_logger
from common_interfaces.src.graph_manager import GraphManager

class NavigateToBuildingTool(BaseTool):
    def __init__(self, publisher):
        super().__init__(name="Navigate to Building Tool", description="Tool to navigate")
        self._publisher = publisher
        self._logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

    def _run(self, agent_id, visitor_id, building_id, graph_manager):
        # FileLock for JSON operations
        self._json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        campus_info = "/home/nachiketa/dup_auto_ass1/src/data/campus_info.json"
        with open(campus_info, "r") as f:
            data = json.load(f)

        self._logger.info(f"{agent_id} is navigating to {building_id} with {visitor_id}")

       
        for path in data['buildings'][building_id]:
            # Update the information
            graph_manager.update_agent_position(agent_id, path)
            graph_manager.update_agent_position(visitor_id, path)
            time.sleep(1)