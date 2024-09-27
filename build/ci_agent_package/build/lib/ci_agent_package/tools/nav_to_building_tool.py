from crewai_tools import BaseTool
from std_msgs.msg import String

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json

class NavigateToBuildingTool(BaseTool):
    def __init__(self, publisher):
        super().__init__(name="Navigate to Building Tool", description="Tool to navigate")
        self._publisher = publisher

    def _run(self, visitor_id, building_id):
        message = f"ci_agent_1 is navigating to {building_id} with visitor {visitor_id}"

        # Update the information
        write_pos_to_json('ci_agent_1', building_id, None)

        self._publisher.publish(String(data=message))
        return message