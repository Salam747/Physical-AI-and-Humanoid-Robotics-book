import rclpy
from rclpy.node import Node
from rclpy.time import Time
from robot_status_msgs.msg import RobotStatus
from std_msgs.msg import Header

class CustomStatusPublisher(Node):

    def __init__(self):
        super().__init__('custom_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.status_code = 0

    def timer_callback(self):
        msg = RobotStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status_code = self.status_code % 3  # Cycle through 0, 1, 2
        msg.battery_level = 100.0 - (self.status_code * 5.0) # Example battery level decrease
        if msg.status_code == 0:
            msg.message = "Robot operating normally."
        elif msg.status_code == 1:
            msg.message = "Low battery warning."
        else:
            msg.message = "System error detected."
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.message}" (Battery: {msg.battery_level:.1f}%)')
        self.status_code += 1

def main(args=None):
    rclpy.init(args=args)
    custom_status_publisher = CustomStatusPublisher()
    rclpy.spin(custom_status_publisher)
    custom_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
