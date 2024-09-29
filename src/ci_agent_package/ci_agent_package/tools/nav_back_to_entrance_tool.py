from crewai_tools import BaseTool
import time

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.logger_config import get_logger

from filelock import FileLock

class NavigateBackToEntranceTool(BaseTool):
    name: str = "Navigate Back to Entrance Tool"
    description: str = "This tool is for navigation back to entrance."

    def _run(self, agent_id, visitor_id, building_id, navigation_path):
        # FileLock for JSON operations
        self._json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

        path_list = navigation_path.split("->")
        for path in reversed(path_list):
            # Update the information
            with self._json_lock:
                write_pos_to_json(agent_id, path, None)
                write_pos_to_json(visitor_id, path, None)
                time.sleep(1)
        
        with self._json_lock:
            write_pos_to_json(agent_id, 'Campus Entrance', None)
            write_pos_to_json(visitor_id, 'Campus Entrance', None)

            time.sleep(2)

            write_pos_to_json(agent_id, 'CI Lobby', None)
            write_pos_to_json(visitor_id, 'VI Lobby', None)

        
        time.sleep(10)

        logger.info(f"{agent_id} is back at the Campus Entrance")