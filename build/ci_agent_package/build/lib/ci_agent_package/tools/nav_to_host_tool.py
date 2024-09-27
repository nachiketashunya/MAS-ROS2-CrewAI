from crewai_tools import BaseTool
import time

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json

class NavigateToHostTool(BaseTool):
    name: str = "Navigate to Host Tool"
    description: str = "This tool is for navigation to host of visitor."

    def _run(self, visitor_id, building_id, navigation_path):
        # Simulate guiding the visitor inside the building using the provided navigation path
        print(f"Guiding visitor {visitor_id} inside building {building_id} using the navigation path: {navigation_path}")

        path_list = navigation_path.split("->")

        for path in path_list:
            # Update the information
            write_pos_to_json('ci_agent_1', path, None)
            time.sleep(2)

        return f"Visitor {visitor_id} successfully guided to the host in building {building_id}."