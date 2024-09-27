from crewai import Agent, Task
from bi_agent_package.tools.fetch_nav_tool import FetchNavigationTool

class BIAgent:
    def __init__(self):
        # Define the BI agent with CrewAI
        self.tools = [FetchNavigationTool()]
        self.agent = Agent(
            role='Building Incharge',
            goal='Assist CI agents with building navigation',
            backstory="You are responsible for providing navigation assistance within the building and ensuring visitors reach their hosts smoothly.",
            memory=False,
            verbose=True,
            tools=self.tools
        )

    def define_tasks(self):
        # Define the task of handling navigation requests as a CrewAI Task
        handle_navigation_task = Task(
            description='Handle navigation requests from CI agents and provide navigation paths within the building.',
            expected_output='Navigation path or response provided to CI agent.',
        )
       
        return [handle_navigation_task]