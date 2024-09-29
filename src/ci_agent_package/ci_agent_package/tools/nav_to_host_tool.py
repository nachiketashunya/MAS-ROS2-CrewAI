from crewai_tools import BaseTool
import time
from filelock import FileLock

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.logger_config import ret_logger

class NavigateToHostTool(BaseTool):
    name: str = "Navigate to Host Tool"
    description: str = "This tool is for navigation to host of visitor."

    def _run(self, agent_id, visitor_id, building_id, navigation_path):
        logger = ret_logger()

        # FileLock for JSON operations
        self._json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        # Simulate guiding the visitor inside the building using the provided navigation path
        logger.info(f"Guiding {visitor_id} inside {building_id} using the navigation path: {navigation_path}")
        
        path_list = navigation_path.split("->")
        for path in path_list:
            with self._json_lock:
                # Update the information
                write_pos_to_json(agent_id, path, None)
                write_pos_to_json(visitor_id, path, None)
                time.sleep(2)

        logger.info(f"Visitor {visitor_id} successfully guided to the host in building {building_id}")
