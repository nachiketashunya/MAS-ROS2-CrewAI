from crewai_tools import BaseTool
import time

class NavigateToHostTool(BaseTool):
    name: str = "Navigate to Host Tool"
    description: str = "This tool is for navigation to host of visitor."

    def _run(self, visitor_id, building_id, navigation_path):
        # Simulate guiding the visitor inside the building using the provided navigation path
        print(f"Guiding visitor {visitor_id} inside building {building_id} using the navigation path: {navigation_path}")
        time.sleep(2)  # Simulating internal navigation
        return f"Visitor {visitor_id} successfully guided to the host in building {building_id}."