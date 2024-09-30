from crewai import Agent
from ci_agent_package.tools.nav_to_building_tool import NavigateToBuildingTool
from ci_agent_package.tools.req_building_nav_tool import RequestBuildingNavigationTool
from ci_agent_package.tools.nav_to_host_tool import NavigateToHostTool
from ci_agent_package.tools.nav_back_to_entrance_tool import NavigateBackToEntranceTool
from ci_agent_package.tasks.escort_to_entrance import EscortToEntranceTask
from ci_agent_package.tasks.escort_to_host import EscortToHostTask

import sys
sys.path.append("/home/nachiketa/dup_auto_ass1/src")
from common_interfaces.src.update_json import write_pos_to_json
from common_interfaces.src.logger_config import get_logger

from std_msgs.msg import String
from filelock import FileLock

import time


class CIAgent:
    def __init__(self, publisher, subscriber, agent_id, vi_indicator, callback_group):
        # Initialize the tools
        self.publisher = publisher
        self.subscriber = subscriber
        self.agent_id = agent_id
        self.vi_indicator = vi_indicator
        self.available = True

        self.total_visitors = 0
        self.total_violations = 0
        self.guide_duration = 0

        self.logger = get_logger(log_file_path="/home/nachiketa/dup_auto_ass1/src/data/events.log")

        # FileLock for JSON operations
        self._json_lock = FileLock("/home/nachiketa/dup_auto_ass1/src/data/positions.json")

        # Tools for navigation
        self.navigate_to_building_tool = NavigateToBuildingTool(publisher)
        self.request_building_navigation_tool = RequestBuildingNavigationTool(publisher, subscriber)
        self.navigate_to_host_tool = NavigateToHostTool()
        self.navigate_back_to_entrance_tool = NavigateBackToEntranceTool()

        # Attach tools to the agent
        tools = [
            self.navigate_to_building_tool,
            self.request_building_navigation_tool,
            self.navigate_to_host_tool,
            self.navigate_back_to_entrance_tool
        ]

        # Initialize the agent in CrewAI
        self.agent = Agent(
            role='Campus Incharge',
            goal='Escort visitors to the designated building and host',
            backstory="You're responsible for guiding visitors from the campus entrance to the host and then back to the entrance.",
            memory=False,
            verbose=True,
            tools=tools
        )
    
    def set_available(self):
        self.available = True
    
    def set_unavailable(self):
        self.available = False
    
    def is_available(self):
        return self.available

    def update_bi_response(self, bi_response):
        """
        Update the navigation path inside the RequestBuildingNavigationTool.
        """
        self.request_building_navigation_tool.set_bi_response(bi_response)

    def define_tasks(self):
        """
        Define tasks such as escorting the visitor to the host and back to the entrance.
        """
        escort_to_host_task = EscortToHostTask(agent=self.agent)
        escort_to_entrance_task = EscortToEntranceTask(agent=self.agent)

        return [escort_to_host_task, escort_to_entrance_task]

    def guide_visitor(self, visitor_id, building, room, host, meeting_time):
        """
        Guide the visitor to the building, host, and back to the entrance.
        """
        tasks = self.define_tasks()

        inputs = {
            'agent_id': self.agent_id, 
            'visitor_id': visitor_id, 
            'building_id': building, 
            'room': room, 
            'host': host,
            'meeting_time': meeting_time, 
            'navigation_path': None
        }
        
        start = time.time()

        result1 = tasks[0].execute(inputs=inputs)
        
        if result1 == False:
            self.logger.info(f"Returning to base")

            with self._json_lock:
                write_pos_to_json(self.agent_id, 'Campus Entrance', None)
                write_pos_to_json(visitor_id, 'Campus Entrance', None)

                time.sleep(2)

                write_pos_to_json(self.agent_id, 'CI Lobby', None)
                write_pos_to_json(visitor_id, 'VI Lobby', None)
        
        else:
            inputs['navigation_path'] = result1
            result2 = tasks[1].execute(inputs=inputs)
            
        self.set_available()
        self.logger.info(f"{self.agent_id} is available. Status {self.agent_id}: {self.is_available()}")

        end = time.time()

        total_duration = end - start

        if total_duration > self.guide_duration:
            self.total_violations += 1
            self.logger.info(f"{self.agent_id} took more than expected time. Total Violations: {self.total_violations}")

        self.logger.info(f"{self.agent_id} Total Visitos Guided: {self.total_visitors} Total Violations: {self.total_violations}")

        vi_msg = String()
        vi_msg.data = visitor_id
        self.vi_indicator.publish(vi_msg)

        self.logger.info(f"{visitor_id} is back at VI Lobby")
