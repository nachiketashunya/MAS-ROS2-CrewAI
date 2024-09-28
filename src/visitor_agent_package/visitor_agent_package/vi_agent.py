from crewai import Agent
from std_msgs.msg import String
import json

class VIAgent:
    def __init__(self, agent_id, building, room, host):
        self.agent_id = agent_id
        self.host = host
        self.building = building
        self.room = room 
        
        self.is_ci_assgnd = False

        self.agent = Agent(
            role='Visitor',
            goal='Visit a host',
            backstory='You are a visitor waiting to be escorted to meet a host.',
            memory=False,
            verbose=False
        )

