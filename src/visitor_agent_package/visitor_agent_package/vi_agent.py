from crewai import Agent

class VIAgent:
    def __init__(self, agent_id, building, room, host, meeting_time, callback_group):
        self.agent_id = agent_id
        self.host = host
        self.building = building
        self.room = room
        self.meeting_time = meeting_time
        self.is_ci_assgnd = False

        # Initialize the agent in CrewAI
        self.agent = Agent(
            role='Visitor',
            goal='Visit a host',
            backstory='You are a visitor waiting to be escorted to meet a host.',
            memory=False,
            verbose=False
        )
