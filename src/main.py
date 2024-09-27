from crewai import Crew
from ci_agent_package.ci_agent_package.ci_agent import CIAgent

def main():
    # Initialize ROS2 publisher and subscriber
    publisher = None  # Replace with actual ROS2 publisher
    subscriber = None  # Replace with actual ROS2 subscriber

    # Initialize CI agent
    ci_agent = CIAgent(publisher, subscriber)

    # Visitor and building IDs
    visitor_id = "V001"
    building_id = "B001"

    # Define the tasks
    tasks = ci_agent.define_tasks(visitor_id, building_id)

    # Create a crew with the tasks
    crew = Crew(
        agents=[ci_agent.agent],
        tasks=tasks,
        verbose=True
    )

    # Execute tasks with inputs
    result = crew.kickoff(inputs={'visitor_id': visitor_id, 'building_id': building_id})
    print(result)

if __name__ == '__main__':
    main()