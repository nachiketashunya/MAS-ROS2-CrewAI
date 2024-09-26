from crewai import Task 

class EscortToHostTask(Task):
    def __init__(self):
        super().__init__(
            description='Escort visitor from campus entrance to the host inside the building.',
            expected_output='Visitor successfully escorted to the host.'
        )

    def execute(self, agent, inputs):
        visitor_id = inputs['visitor_id']
        building_id = inputs['building_id']
        
        # Phase 1: Guide to building entrance
        result = agent.tools['NavigateToBuildingTool'].run(visitor_id, building_id)
        print(result)

        # Phase 2: Request navigation from BI agent
        navigation_path = agent.tools['RequestBuildingNavigationTool'].run(agent.publisher, agent.subscriber, visitor_id, building_id)
        print(navigation_path)

        # Phase 3: Guide to host
        result = agent.tools['NavigateToHostTool'].run(visitor_id, building_id, navigation_path)
        print(result)

        return "Escort to host completed"