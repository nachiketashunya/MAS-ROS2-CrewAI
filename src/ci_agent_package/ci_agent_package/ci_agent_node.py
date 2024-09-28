import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
from rclpy.executors import MultiThreadedExecutor
from ci_agent_package.ci_agent import CIAgent
import time


class CIAgentThread(threading.Thread):
    """
    Thread class to run each CIAgent in its own thread.
    """
    def _init_(self, ci_agent):
        threading.Thread._init_(self)
        self.ci_agent = ci_agent
        self.daemon = True  # Ensure that threads exit when the main program exits

    def run(self):
        while True:
            if not self.ci_agent.is_available():
                print(f"{self.ci_agent.agent_id} is guiding visitor {self.ci_agent.visitor_id}.")
                # Simulate agent guiding the visitor
                self.ci_agent.perform_guidance()
                self.ci_agent.set_available()  # Mark agent as available again after completing task
            time.sleep(1)


class CIAgentNode(Node):
    def _init_(self):
        super()._init_('ci_agent_node')

        self.ci_agents = []
        self.lock = threading.Lock()  # To manage thread-safe access to agent availability

        # Initialize 3 CI agents and start them in separate threads
        for i in range(3):
            ci_agent = CIAgent(
                publisher=self.create_publisher(String, 'ci_to_bi_navigation_request', 10),
                subscriber=None,
                agent_id=f"ci_agent_{i+1}"
            )
            self.ci_agents.append(ci_agent)

            # Start each CIAgent in its own thread
            agent_thread = CIAgentThread(ci_agent)
            agent_thread.start()

        print("CI node Initialization")

        # Publishers and Subscribers
        self.confirm_vis_req = self.create_publisher(String, 'ci_to_vi_confirm_res', 10)
        self.vis_req_subscriber = self.create_subscription(
            String,
            'vi_to_ci_request',
            self.handle_vis_req,
            10
        )

        # Listener for responses from BI agent
        self.subscriber = self.create_subscription(
            String,
            'bi_to_ci_navigation_response',
            self.receive_navigation_response,
            10
        )

        self.navigation_response_received = threading.Event()

    def handle_vis_req(self, msg):
        """
        Handle visitor requests.
        """
        data = msg.data.split("==>")
        vis_id, building, room, host = data[0], data[1], data[2], data[3]

        print(f"Visitor {vis_id} wants to visit HOST: {host} in {room}, {building}")

        # Find an available CI agent
        agent = self.avail_ci_agent()
        
        if agent:
            with self.lock:  # Thread-safe access to agent
                agent.available = False
                print(f"{agent.agent_id} is guiding {vis_id} to {host}")

                # Notify visitor that a CI agent has been assigned
                confirmation_msg = String()
                confirmation_msg.data = vis_id
                self.confirm_vis_req.publish(confirmation_msg)
                print(f"Confirmation for {vis_id} published")

                # Guide visitor (this will be handled in the agent's thread)
                agent.assign_task(visitor_id=vis_id, building=building, room=room, host=host)
        else:
            print("No available CI agent.")

    def receive_navigation_response(self, msg):
        """
        Receive navigation response from BI agent.
        """
        print("Navigation Response Received")
        path = msg.data.split('->')
        agent_id = path.pop()  # Last item is the agent ID

        # Update the respective CI agent with the navigation path
        for ci_agent in self.ci_agents:
            if ci_agent.agent_id == agent_id:
                nav_path = "->".join(path)
                ci_agent.update_navigation_path(nav_path)
                self.navigation_response_received.set()

    def avail_ci_agent(self):
        """
        Find an available CI agent.
        """
        with self.lock:
            for ci_agent in self.ci_agents:
                if ci_agent.is_available():
                    return ci_agent
        return None


def main(args=None):
    rclpy.init(args=args)

    # Initialize the CI agent node (manager)
    node = CIAgentNode()

    # MultiThreadedExecutor for handling node callbacks
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


if _name_ == '_main_':
    main()