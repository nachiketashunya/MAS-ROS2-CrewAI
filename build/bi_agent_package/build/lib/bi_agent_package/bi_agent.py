from crewai import Agent, Task

class BIAgent:
    def __init__(self):
        # Define the BI agent with CrewAI
        self.agent = Agent(
            role='Building Incharge',
            goal='Assist CI agents with building navigation',
            verbose=True,
            memory=True,
            backstory=(
                "You are responsible for providing navigation assistance within the building "
                "and ensuring visitors reach their hosts smoothly."
            )
        )
        self.define_tasks()

    def define_tasks(self):
        # Define the task of handling navigation requests as a CrewAI Task
        handle_navigation_task = Task(
            description='Handle navigation requests from CI agents and provide navigation paths within the building.',
            expected_output='Navigation path or response provided to CI agent.',
            tools=[],  # Add relevant tools for ROS communication if needed
        )
        # Add the task to the agent
        self.agent.add_task(handle_navigation_task)

    def run_agent(self, inputs):
        # Kick off the agent with inputs (e.g., visitor_id, building_id)
        result = self.agent.kickoff(inputs=inputs)
        print(result)
        return result