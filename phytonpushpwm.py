import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TargetRPMPublisher(Node):
    def __init__(self):
        super().__init__('target_rpm_publisher')
        self.publisher_ = self.create_publisher(Float32, 'targetrpm', 10)
        self.timer = self.create_timer(0.5, self.publish_target_rpm)  # Publish every 0.5 seconds
        self.target_rpm = 150.0  # Default target RPM
        self.get_logger().info('TargetRPM Publisher Node Started')

    def publish_target_rpm(self):
        msg = Float32()
        msg.data = self.target_rpm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target RPM: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetRPMPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
