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
        agent_id = inputs['agent_id']
        room = inputs['room']
        meeting_time = inputs['meeting_time']
        host = inputs['host']
        
        # Phase 1: Guide to building entrance
        self.agent.tools[0].run(agent_id, visitor_id, building_id)

        # Phase 2: Request navigation from BI agent
        bi_response = self.agent.tools[1].run(agent_id, visitor_id, building_id, room, meeting_time)
        
        if bi_response in ['Unauthorized', 'Unavailable', 'OOS', 'Timed Out']:
            return False

        # Phase 3: Guide to host
        self.agent.tools[2].run(agent_id, visitor_id, building_id, host, bi_response)

        return bi_response

