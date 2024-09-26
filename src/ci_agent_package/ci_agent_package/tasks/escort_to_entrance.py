from crewai import Task

class EscortToEntranceTask(Task):
    def __init__(self):
        super().__init__(
            description='Escort visitor from the host back to the campus entrance.',
            expected_output='Visitor successfully escorted back to the campus entrance.'
        )

    def execute(self, agent, inputs):
        visitor_id = inputs['visitor_id']
        building_id = inputs['building_id']

        # Phase 1: Guide visitor back to the building entrance
        result = agent.tools['NavigateBackToEntranceTool'].run(visitor_id)
        print(result)

        return "Escort back to campus entrance completed"