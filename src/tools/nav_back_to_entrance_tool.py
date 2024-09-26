from crewai_tools import BaseTool
import time

class NavigateBackToEntranceTool(BaseTool):
    name: str = "Navigate to Building Tool"
    description: str = "This tool is for navigation to building of visitor."

    def _run(self, visitor_id, building_id):
        # Simulate guiding the visitor inside the building using the provided navigation path
        print(f"Guiding visitor {visitor_id} back to entrancef from building id: {building_id}")
        time.sleep(2)  # Simulating internal navigation
        return f"Visitor {visitor_id} successfully guided to the host in building {building_id}."