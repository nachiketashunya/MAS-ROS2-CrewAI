import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visitor_agent_package.vi_agent import VIAgent
import random
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class VIAgentNode(Node):
    def __init__(self):
        super().__init__('vi_agent_node')

        self.vi_agents = []
        self.callback_groups = []
        self.assigned_event = threading.Event()
        self.lock = threading.Lock()

        hosts = ['A1', 'A2', 'A3', 'A4', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3', 'C4', 'D1', 'D2']

        for i in range(7):
            agent_callback_group = ReentrantCallbackGroup()
            vi_agent = self.create_vi_agent(i, hosts, agent_callback_group)
            self.vi_agents.append(vi_agent)
            self.callback_groups.append(agent_callback_group)

        self.request_publisher = self.create_publisher(String, 'vi_to_ci_request', 10)
        
        # Use MutuallyExclusiveCallbackGroup for subscriptions
        subscription_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.confirmation_subscriber = self.create_subscription(
            String, 
            'ci_to_vi_confirm_res', 
            self.confirmation_res, 
            10,
            callback_group=subscription_callback_group
        )
        self.indicator_subscription = self.create_subscription(
            String,
            'vi_indicator',
            self.handle_indication,
            10,
            callback_group=subscription_callback_group
        )

    def create_vi_agent(self, i, hosts, callback_group):
        host = random.choice(hosts)
        room = f"Room {host}"
        building = f"Building {host[0]} Entrance"
        meeting_time = random.randint(60, 180)

        return VIAgent(
            agent_id=f"vi_agent_{i+1}", 
            building=building, 
            room=room, 
            host=host,
            meeting_time=meeting_time,
            callback_group=callback_group
        )

    def handle_indication(self, msg):
        vis_id = msg.data
        with self.lock:
            for vi_agent in self.vi_agents:
                if vi_agent.agent_id == vis_id:
                    # vi_agent.is_ci_assgnd = False
                    # Reset agent for a new meeting
                    hosts = ['A1', 'A2', 'A3', 'A4', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3', 'C4', 'D1', 'D2']
                    host = random.choice(hosts)
                
                    output = random.choices([0, 1], weights=[70, 30], k=1)[0]
                    
                    vi_agent.room = f"Room {host}"
                    vi_agent.host = host

                    if output == 1:
                        vi_agent.room = f"Building {host[0]} Entrance"
                        vi_agent.host = f"bi_agent_{host[0]}"

                    vi_agent.building = f"Building {host[0]} Entrance"
                    vi_agent.meeting_time = random.randint(30, 90)
                    self.get_logger().info(f"{vis_id} has finished meeting and is ready for a new request")
                    
                    if vi_agent.req_count < 3:
                        self.request_guidance(vi_agent)
                        vi_agent.req_count += 1


    def request_guidance(self, visitor):
        data = f"{visitor.agent_id}==>{visitor.building}==>{visitor.room}==>{visitor.host}==>{visitor.meeting_time}"
        msg = String()
        msg.data = data
        self.request_publisher.publish(msg)
        self.get_logger().info(f"{visitor.agent_id} requested to meet {visitor.host} for {visitor.meeting_time}seconds.")

    def confirmation_res(self, msg):
        data = msg.data.split("==>")
        status, vis_id, ci_id = data[0], data[1], data[2]
        
        if status == "A":
            with self.lock:
                for vi_agent in self.vi_agents:
                    if vi_agent.agent_id == vis_id:
                        self.get_logger().info(f"{ci_id} is assigned to {vis_id}")
                        vi_agent.is_ci_assgnd = True
                        self.assigned_event.set()
                        break
        else:
            self.get_logger().info(f"Assigned event is reset")
            self.assigned_event.set()

        # Remove the sleep here
        # time.sleep(20)

    def process_vi_agents(self):
        while rclpy.ok():
            with self.lock:
                available_agents = [agent for agent in self.vi_agents if not agent.is_ci_assgnd]
            
            if available_agents:
                vi_agent = random.choice(available_agents)

                if vi_agent.req_count < 3:
                    self.request_guidance(vi_agent)
                    vi_agent.req_count += 1
                
                    # Wait until confirmation is received before processing the next agent
                    self.assigned_event.wait()
                    self.assigned_event.clear()
            
            time.sleep(1)  # Add a small delay to prevent tight looping

def main(args=None):
    rclpy.init(args=args)
    node = VIAgentNode()
    vi_agent_thread = threading.Thread(target=node.process_vi_agents)
    vi_agent_thread.start()
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