from crewai import Task

class EscortToEntranceTask(Task):
    def __init__(self, agent):
        super().__init__(
            description='Escort visitor from the host back to the campus entrance.',
            expected_output='Visitor successfully escorted back to the campus entrance.',
            agent=agent
        )

    def execute(self, inputs):
        visitor_id = inputs['visitor_id']
        building_id = inputs['building_id']
        room = inputs['room']
        navigation_path = inputs['navigation_path']
        agent_id = inputs['agent_id']

        # Phase 1: Guide visitor back to the building entrance
        result = self.agent.tools[3].run(agent_id, visitor_id, building_id, navigation_path)
        print(result)

        return "Escort back to campus entrance completed"