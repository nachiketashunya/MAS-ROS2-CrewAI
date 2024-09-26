from crewai_tools import BaseTool

class NavigateToBuildingTool(BaseTool):
    name: str = "Navigate to Building Tool"
    description: str = "This tool is for navigation to building of visitor."

    def _run(self, visitor_id, building_id):
        return f"Visitor: {visitor_id} Building: {building_id}"