import rclpy
from rclpy.node import Node
from robot_status_msgs.msg import RobotStatus

class CustomStatusSubscriber(Node):

    def __init__(self):
        super().__init__('custom_status_subscriber')
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received RobotStatus: Timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                               f'Status Code={msg.status_code}, Battery={msg.battery_level:.1f}%, Message="{msg.message}"')

def main(args=None):
    rclpy.init(args=args)
    custom_status_subscriber = CustomStatusSubscriber()
    rclpy.spin(custom_status_subscriber)
    custom_status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
