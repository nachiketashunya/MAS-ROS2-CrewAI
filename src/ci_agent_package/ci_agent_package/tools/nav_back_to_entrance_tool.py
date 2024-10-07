from crewai_tools import BaseTool
import time

import sys
import json
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.logger_config import get_logger

import threading

class NavigateBackToEntranceTool(BaseTool):
    name: str = "Navigate Back to Entrance Tool"
    description: str = "This tool is for navigation back to entrance."

    def _run(self, agent_id, visitor_id, building_id, navigation_path, graph_manager):
        logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

        campus_info = "/home/nachiketa/dup_auto_ass1/src/data/campus_info.json"
        with open(campus_info, "r") as f:
            data = json.load(f)
           
        lock = threading.Lock()

        with lock:
            if navigation_path is not None:
                path_list = navigation_path.split("->")
                for path in reversed(path_list):
                    graph_manager.update_agent_position(agent_id, path)
                    graph_manager.update_agent_position(visitor_id, path)
                    time.sleep(1)
        
            for path in reversed(data['buildings'][building_id]):
                # Update the information
                graph_manager.update_agent_position(agent_id, path)
                graph_manager.update_agent_position(visitor_id, path)
                time.sleep(1)
           
            graph_manager.update_agent_position(agent_id, 'CI Lobby')
            graph_manager.update_agent_position(visitor_id, 'VI Lobby')        

        logger.info(f"{agent_id} is back at the Campus Entrance")