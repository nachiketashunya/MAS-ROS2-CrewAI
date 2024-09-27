import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bi_agent_package.bi_agent import BIAgent
import json

class BIAgentNode(Node):
    def __init__(self):
        super().__init__('bi_agent_node')

        print("BI Agent Initialization")

        # Create a BI agent instance
        self.bi_agent = BIAgent()

        # Publisher to respond to CI agent with navigation paths
        self.navigation_response_publisher = self.create_publisher(String, 'bi_to_ci_navigation_response', 10)

        # Subscription to receive navigation requests from CI agent
        self.navigation_request_subscription = self.create_subscription(
            String,
            'ci_to_bi_navigation_request',
            self.handle_navigation_request,
            10
        )

    def handle_navigation_request(self, msg):
        """
        Handle navigation requests from the CI agent.
        """
        data = msg.data
        # data = json.loads(msg.data)

        path = self.bi_agent.tools[0].run(data)
        # path = self.bi_agent.tools[0].run(data['building_id'])
        ret_path = "->".join(path)
        self.send_navigation_response(ret_path)
        # self.get_logger().info(response)

    def send_navigation_response(self, response_data):
        """
        Send the navigation response back to the CI agent.
        """
        msg = String()
        msg.data = response_data
        self.navigation_response_publisher.publish(msg)
        print("Navigation response published")

        self.get_logger().info(f"Sent navigation response: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize the BI agent node
    node = BIAgentNode()

    # Spin to keep the node alive and handle communication
    rclpy.spin(node)

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()