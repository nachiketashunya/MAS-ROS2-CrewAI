import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visitor_agent_package.vi_agent import VIAgent
import json
import random
import time
from rclpy.executors import MultiThreadedExecutor
import threading

class VIAgentNode(Node):
    def __init__(self):
        super().__init__('vi_agent_node')

        self.vi_agents = []
        self.assigned_event = False

        # Define some hosts for the visitors
        hosts = ['A1', 'A2', 'A3', 'B1', 'B2', 'C1', 'C2', 'C3', 'C4']

        # Initialize 5 VI agents with random hosts
        for i in range(5):
            host = random.choice(hosts)
            room = f"Room {host}"
            building = f"Building {host[0]}"

            vi_agent = VIAgent(agent_id=f"vi_agent_{i+1}", building=building, room=room, host=host)
            self.vi_agents.append(vi_agent)

        # Publisher to send a request to CI agent for escorting the visitor
        self.request_publisher = self.create_publisher(String, 'vi_to_ci_request', 10)

        # Subscriber to receive confirmation from CI agent that an escort has been assigned
        self.confirmation_subscriber = self.create_subscription(String, 'ci_to_vi_confirm_res', self.confirmation_res, 10)

    def request_guidance(self, visitor):
        """
        Request CI agent to guide the visitor to the host.
        """
        data = f"{visitor.agent_id}==>{visitor.building}==>{visitor.room}==>{visitor.host}"

        msg = String()
        msg.data = data
        self.request_publisher.publish(msg)
        print(f"VI Agent {visitor.agent_id} requested guidance to {visitor.host}.")

    def confirmation_res(self, msg):
        """
        Handle confirmation response from CI agent.
        """
        print("Confirmation Response Received")
        print(msg.data)
        vis_id = msg.data
        for vi_agent in self.vi_agents:
            if vi_agent.agent_id == vis_id:
                print(f"CI Assigned to {vi_agent.agent_id}")
                vi_agent.is_ci_assgnd = True
                self.assigned_event = True  # Signal that the confirmation has been received

    def process_vi_agents(self):
        """
        Handle VI agents one by one, waiting for CI agent confirmation before proceeding to the next.
        """
        for vi_agent in self.vi_agents:
            while not vi_agent.is_ci_assgnd:
                self.request_guidance(vi_agent)
                rclpy.spin_once(self, timeout_sec=2)  # Spin to check for messages
                if self.assigned_event:
                    self.assigned_event = False  # Reset event for next agent
                    break
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