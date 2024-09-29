from crewai_tools import BaseTool
import json

class FetchNavigationTool(BaseTool):
    name: str = "Fetch Navigation of Building"
    description: str = "This tool is fetching the navigation inside of a building."

    def _run(self, building_id, room):
        path = ""
        building_info = "/home/nachiketa/dup_auto_ass1/src/data/building_info.json"
        try:
            with open(building_info, 'r') as file:
                data = json.load(file)
            # Access the path of the specific room within the specified building
            
            path = data["buildings"][building_id]["rooms"][room]["path"]
            
        except KeyError:
            return f"Room not found in '{building_id}'."
    
        return path