import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bi_agent_package.bi_agent import BIAgent
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class BIAgentNode(Node):
    def __init__(self):
        super().__init__('bi_agent_node')

        self.bi_agents = []
        self.callback_groups = []

        # Initialize 3 BI agents with callback groups
        for i in range(3):
            agent_callback_group = ReentrantCallbackGroup()

            bi_agent = BIAgent(agent_id=f"bi_agent_{i+1}", callback_group=agent_callback_group)
            self.bi_agents.append(bi_agent)
            self.callback_groups.append(agent_callback_group)

        print("BI Agent Node Initialized")

        # Publisher to respond to CI agent with navigation paths
        self.navigation_response_publisher = self.create_publisher(
            String, 
            'bi_to_ci_navigation_response', 
            10
        )

        # Subscription to receive navigation requests from CI agent
        self.navigation_request_subscription = self.create_subscription(
            String, 
            'ci_to_bi_navigation_request', 
            self.handle_navigation_request, 
            10
        )

    def handle_navigation_request(self, msg):
        """
        Handle navigation requests from CI agents.
        """
        data = msg.data.split("==>")

        print("Received navigation request from CI agent", data)

        building_id, room_id, ci_agent_id = data[0], data[1], data[2]

        # Find an available BI agent to handle the request
        bi_agent = self.avail_bi_agent()

        if bi_agent:
            # Fetch the navigation path using BI agent tools
            path = bi_agent.tools[0].run(building_id, room_id)
            path.append(ci_agent_id)  # Add CI agent ID to the end of the path

            # Format and send the response back to the CI agent
            ret_path = "->".join(path)
            self.send_navigation_response(ret_path)

        else:
            print("No available BI agent.")

    def send_navigation_response(self, response_data):
        """
        Send the navigation response back to the CI agent.
        """
        msg = String()
        msg.data = response_data
        self.navigation_response_publisher.publish(msg)
        print("Navigation response sent to CI agent:", response_data)

    def avail_bi_agent(self):
        """
        Find an available BI agent.
        """
        for bi_agent in self.bi_agents:
            if bi_agent.is_available():
                return bi_agent
        return None


def main(args=None):
    rclpy.init(args=args)

    # Initialize the BI agent node
    node = BIAgentNode()

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


if __name__ == '__main__':
    main()