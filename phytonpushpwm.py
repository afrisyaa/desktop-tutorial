import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import termios
import sys
import tty

class TargetRPMPublisher(Node):
    def __init__(self):
        super().__init__('target_rpm_publisher')
        self.publisher_ = self.create_publisher(Float32, 'targetrpm', 10)
        self.target_rpm = 0.0  # Initial target RPM
        self.get_logger().info('TargetRPM Publisher Node Started. Use "w" to increase, "s" to decrease, "x" to stop.')

        # Start publishing in a timer loop
        self.timer = self.create_timer(0.5, self.publish_target_rpm)

    def publish_target_rpm(self):
        msg = Float32()
        msg.data = self.target_rpm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target RPM: {msg.data}')

    def handle_key_press(self, key):
        if key == 'w':  # Increase RPM
            self.target_rpm += 10.0
            self.get_logger().info(f'Increased target RPM to {self.target_rpm}')
        elif key == 's':  # Decrease RPM
            self.target_rpm = max(0.0, self.target_rpm - 10.0)  # Prevent negative RPM
            self.get_logger().info(f'Decreased target RPM to {self.target_rpm}')
        elif key == 'x':  # Stop motor
            self.target_rpm = 0.0
            self.get_logger().info('Stopped motor (target RPM set to 0)')
        else:
            self.get_logger().info('Invalid key. Use "w", "s", or "x".')

def get_key():
    """Get a single character from standard input without echo."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TargetRPMPublisher()

    try:
        while rclpy.ok():
            # Non-blocking key press
            key = get_key()
            node.handle_key_press(key)
            rclpy.spin_once(node, timeout_sec=0.1)  # Allow ROS to process events
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
