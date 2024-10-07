class BIAgent:
    def __init__(self, agent_id):
        self.agent_id = agent_id
        self.is_oos = False

        self.total_cis = 0
        self.total_violations = 0

        self.oos_duration = 0
    
    def set_oos(self):
        self.is_oos = True
    
    def set_bis(self):
        self.is_oos = False
    
    def is_oos(self):
        return self.is_oos