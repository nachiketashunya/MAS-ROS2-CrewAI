from crewai import Agent, Task
from bi_agent_package.tools.fetch_nav_tool import FetchNavigationTool

class BIAgent:
    def __init__(self, agent_id, callback_group):
        self.agent_id = agent_id
        self.available = True
        self.tools = [FetchNavigationTool()]

        # Initialize the agent in CrewAI
        self.agent = Agent(
            role='Building Incharge',
            goal='Assist CI agents with building navigation',
            backstory="You are responsible for providing navigation assistance within the building and ensuring visitors reach their hosts smoothly.",
            memory=False,
            verbose=True,
            tools=self.tools  # Attach tools to the agent
        )
    
    def set_available(self):
        self.available = True
    
    def set_unavailable(self):
        self.available = False
    
    def is_available(self):
        return self.available

    def define_tasks(self):
        # Define the task of handling navigation requests as a CrewAI Task
        handle_navigation_task = Task(
            description='Handle navigation requests from CI agents and provide navigation paths within the building.',
            expected_output='Navigation path or response provided to CI agent.',
        )
        return [handle_navigation_task]

    def handle_navigation(self, building_id, room_id):
        """
        Fetch navigation path within the building using BI agent tools.
        """
        task = self.define_tasks()[0]
        return task.execute(inputs={'building_id': building_id, 'room_id': room_id})