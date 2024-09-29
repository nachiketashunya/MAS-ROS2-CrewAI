import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visitor_agent_package.vi_agent import VIAgent
import json
import random
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class VIAgentNode(Node):
    def __init__(self):
        super().__init__('vi_agent_node')

        self.vi_agents = []
        self.callback_groups = []  # Store callback groups for agents
        self.assigned_event = threading.Event()

        # Define some hosts for the visitors
        hosts = ['A1', 'A2', 'A3', 'A4', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3', 'C4', 'D1', 'D2']

        # Initialize 5 VI agents with random hosts and callback groups
        for i in range(7):
            agent_callback_group = ReentrantCallbackGroup()

            host = random.choice(hosts)
            room = f"Room {host}"
            building = f"Building {host[0]} Entrance"
            meeting_time = random.randint(60, 180)

            vi_agent = VIAgent(
                agent_id=f"vi_agent_{i+1}", 
                building=building, 
                room=room, 
                host=host,
                meeting_time=meeting_time,
                callback_group=agent_callback_group
            )
            self.vi_agents.append(vi_agent)
            self.callback_groups.append(agent_callback_group)  # Track callback groups

        # Publisher to send a request to CI agent for escorting the visitor
        self.request_publisher = self.create_publisher(String, 'vi_to_ci_request', 10)

        # Subscriber to receive confirmation from CI agent that an escort has been assigned
        self.confirmation_subscriber = self.create_subscription(
            String, 
            'ci_to_vi_confirm_res', 
            self.confirmation_res, 
            10
        )

    def request_guidance(self, visitor):
        """
        Request CI agent to guide the visitor to the host.
        """
        data = f"{visitor.agent_id}==>{visitor.building}==>{visitor.room}==>{visitor.host}==>{visitor.meeting_time}"

        msg = String()
        msg.data = data
        self.request_publisher.publish(msg)
        print(f"{visitor.agent_id} requested to meet {visitor.host} for {visitor.meeting_time}seconds.")

    def confirmation_res(self, msg):
        """
        Handle confirmation response from CI agent.
        """
        data = msg.data.split("==>")
        vis_id, ci_id = data[0], data[1]
        
        for vi_agent in self.vi_agents:
            if vi_agent.agent_id == vis_id:
                print(f"{ci_id} is assigned to {vis_id}")
                
                vi_agent.is_ci_assgnd = True
                time.sleep(30)
                self.assigned_event.set()  # Signal that the confirmation has been received

    def process_vi_agents(self):
        """
        Handle VI agents one by one, waiting for CI agent confirmation before proceeding to the next.
        """
        for vi_agent in self.vi_agents:
            while not vi_agent.is_ci_assgnd:
                self.request_guidance(vi_agent)
                rclpy.spin_once(self, timeout_sec=2)  # Spin to check for messages

                # Wait until confirmation is received before processing the next agent
                self.assigned_event.wait()
                self.assigned_event.clear()  # Reset event for next agent

            time.sleep(1)  # Add a delay to simulate processing time


def main(args=None):
    rclpy.init(args=args)

    node = VIAgentNode()

    # Process VI agents
    vi_agent_thread = threading.Thread(target=node.process_vi_agents)
    vi_agent_thread.start()

    # MultiThreadedExecutor to handle the node spinning and callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()