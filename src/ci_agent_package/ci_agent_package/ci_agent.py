from crewai import Agent, Task, Crew
from crewai import Agent
import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")

from tools.nav_to_building_tool import NavigateToBuildingTool
from tools.req_building_nav_tool import RequestBuildingNavigationTool
from tools.nav_to_host_tool import NavigateToHostTool
from tools.nav_back_to_entrance_tool import NavigateBackToEntranceTool
from ci_agent_package.ci_agent_package.tasks import EscortToEntranceTask
from ci_agent_package.ci_agent_package.tasks import EscortToHostTask
import os

class CIAgent:
    def __init__(self, publisher, subscriber):
        # Define the CI agent
        # Define the tools
        self.publisher = publisher
        self.subscriber = subscriber

        tools = {
            'NavigateToBuildingTool': NavigateToBuildingTool(),
            'RequestBuildingNavigationTool': RequestBuildingNavigationTool(),
            'NavigateToHostTool': NavigateToHostTool(),
            'NavigateBackToEntranceTool': NavigateBackToEntranceTool()
        }

        self.agent = Agent(
            role='Campus Incharge',
            goal='Escort visitors to the designated building',
            backstory="You're responsible for guiding visitors from the campus entrance to the host.",
            memory=False,
            verbose=False,
            tools=tools
        )
        
    def guide_visitor(self, visitor_id, building_id):
        # Phase 1: Guide visitor to building
        result = self.agent.tools['NavigateToBuildingTool'].run(visitor_id, building_id)
        print(result)
        
        # # Phase 2: Request navigation inside building from BI agent
        # navigation_path = self.request_building_navigation_tool.run(self.publisher, self.subscriber, visitor_id, building_id)
        
        # # Phase 3: Guide visitor to host
        # result = self.navigate_to_host_tool.run(visitor_id, building_id, navigation_path)
        # print(result)

        # # Phase 4: Guide visitor back to entrance
        # result = self.navigate_back_to_entrance_tool.run(visitor_id, building_id)
        # print(result)
    
    def define_tasks(self, visitor_id, building_id):
        escort_to_host_task = EscortToHostTask()
        escort_to_entrance_task = EscortToEntranceTask()

        return [escort_to_host_task, escort_to_entrance_task]