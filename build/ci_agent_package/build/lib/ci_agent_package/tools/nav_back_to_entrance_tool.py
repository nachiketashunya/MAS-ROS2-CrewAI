from crewai_tools import BaseTool
import time

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json

class NavigateBackToEntranceTool(BaseTool):
    name: str = "Navigate to Building Tool"
    description: str = "This tool is for navigation to building of visitor."

    def _run(self, visitor_id, building_id, navigation_path):
        
        print("Navigation Paht", navigation_path)
        path_list = navigation_path.split("->")

        for path in reversed(path_list):
            # Update the information
            write_pos_to_json('ci_agent_1', path, None)
            time.sleep(2)
        
        write_pos_to_json('ci_agent_1', 'Entrance', None)

        return f"CI agent is back at the Building Gate"