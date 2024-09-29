from crewai_tools import BaseTool
from std_msgs.msg import String
import time
from filelock import FileLock

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.logger_config import get_logger

class NavigateToBuildingTool(BaseTool):
    def __init__(self, publisher):
        super().__init__(name="Navigate to Building Tool", description="Tool to navigate")
        self._publisher = publisher
        self._logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

    def _run(self, agent_id, visitor_id, building_id):
        # FileLock for JSON operations
        self._json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        self._logger.info(f"{agent_id} is navigating to {building_id} with {visitor_id}")

        with self._json_lock:
            # Update the information
            write_pos_to_json(agent_id, building_id, None)
            write_pos_to_json(visitor_id, building_id, None)
            time.sleep(1)