from crewai import Agent
from ci_agent_package.tools.nav_to_building_tool import NavigateToBuildingTool
from ci_agent_package.tools.req_building_nav_tool import RequestBuildingNavigationTool
from ci_agent_package.tools.nav_to_host_tool import NavigateToHostTool
from ci_agent_package.tools.nav_back_to_entrance_tool import NavigateBackToEntranceTool
from ci_agent_package.tasks.escort_to_entrance import EscortToEntranceTask
from ci_agent_package.tasks.escort_to_host import EscortToHostTask


class CIAgent:
    def __init__(self, publisher, subscriber):
        # Initialize the tools
        self.publisher = publisher
        self.subscriber = subscriber

        # Create the tools used by the agent
        self.navigate_to_building_tool = NavigateToBuildingTool(publisher)
        self.request_building_navigation_tool = RequestBuildingNavigationTool(publisher, subscriber)
        self.navigate_to_host_tool = NavigateToHostTool()
        self.navigate_back_to_entrance_tool = NavigateBackToEntranceTool()

        # Assign the tools to the agent
        tools = [
            self.navigate_to_building_tool,
            self.request_building_navigation_tool,
            self.navigate_to_host_tool,
            self.navigate_back_to_entrance_tool
        ]

        # Initialize the agent in CrewAI
        self.agent = Agent(
            role='Campus Incharge',
            goal='Escort visitors to the designated building and host',
            backstory="You're responsible for guiding visitors from the campus entrance to the host and then back to the entrance.",
            memory=False,
            verbose=True,
            tools=tools  # Attach tools to the agent
        )

    def update_navigation_path(self, navigation_path):
        """
        Method to update the navigation path inside the RequestBuildingNavigationTool.
        This will be called from the ROS subscriber when the BI agent provides a response.
        """
        print(f"Updating navigation path: {navigation_path}")
        self.request_building_navigation_tool.set_navigation_path(navigation_path)

    def define_tasks(self):
        """
        Define the tasks for the agent, such as escorting the visitor to the host and back to the entrance.
        """
        escort_to_host_task = EscortToHostTask(agent=self.agent)
        escort_to_entrance_task = EscortToEntranceTask(agent=self.agent)

        return [escort_to_host_task, escort_to_entrance_task]