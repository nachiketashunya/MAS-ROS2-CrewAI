from crewai import Task 

class EscortToHostTask(Task):
    def __init__(self, agent):
        super().__init__(
            description='Escort visitor from campus entrance to the host inside the building.',
            expected_output='Visitor successfully escorted to the host.',
            agent=agent
        )

    def execute(self, inputs):
        visitor_id = inputs['visitor_id']
        building_id = inputs['building_id']
        
        # Phase 1: Guide to building entrance
        result = self.agent.tools[0].run(visitor_id, building_id)
        print(result)

        # Phase 2: Request navigation from BI agent
        navigation_path = self.agent.tools[1].run(visitor_id, building_id)
        print(navigation_path)

        # Phase 3: Guide to host
        result = self.agent.tools[2].run(visitor_id, building_id, navigation_path)
        print(result)

        return navigation_path