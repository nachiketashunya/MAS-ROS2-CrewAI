from crewai_tools import BaseTool
import time
import threading
import sys
import json
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

class NavigateToHostTool(BaseTool):
    name: str = "Navigate to Host Tool"
    description: str = "This tool is for navigation to host of visitor."

    def _run(self, agent_id, visitor_id, building_id, host, navigation_path, graph_manager):
        logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

        # Simulate guiding the visitor inside the building using the provided navigation path
        logger.info(f"Guiding {visitor_id} to {host} using the navigation path: {navigation_path}")
        
        campus_info = "/home/nachiketa/dup_auto_ass1/src/data/campus_info.json"
        with open(campus_info, "r") as f:
            data = json.load(f)

        path_list = navigation_path.split("->")

        
        lock = threading.Lock()
        with lock:
            for path in path_list:
                # Update the information
                graph_manager.update_agent_position(agent_id, path)
                graph_manager.update_agent_position(visitor_id, path)
                time.sleep(1)

        logger.info(f"{visitor_id} successfully guided to the {host}")
