import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePythonAgent(Node):
    def __init__(self):
        super().__init__('simple_python_agent')
        self.publisher_ = self.create_publisher(String, 'high_level_goals', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        goals = ["wave", "stand", "bow"]
        msg.data = goals[self.i % len(goals)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing high-level goal: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_agent = SimplePythonAgent()
    rclpy.spin(simple_agent)
    simple_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
