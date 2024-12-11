import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class TeleopAckermann(Node):
    def __init__(self):
        super().__init__('teleop_ackermann')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5  # Default linear speed
        self.angular_speed = 15.0  # Default angular speed
        self.current_steering_angle = 0.0  # Steering angle (angular.z)
        self.current_speed = 0.0  # Speed of the robot
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """Capture a single key press from the terminal."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Non-blocking with timeout
        if rlist:
            key = sys.stdin.read(1)
            return key
        return None

    def control_loop(self):
        """Main control loop for teleoperation."""
        print("\nControl Keys:\n[w] Forward, [a] Left, [d] Right, [s] Stop, [x] Reverse, [Ctrl+C] Exit\n")
        print("Publishing commands...")

        while rclpy.ok():
            key = self.get_key()
            msg = Twist()

            if key == 'w':  # Move forward
                self.current_speed = self.linear_speed
                msg.linear.x = self.current_speed
                msg.angular.z = self.current_steering_angle
                print("Moving forward")
            elif key == 'a':  # Turn left
                self.current_steering_angle = -self.angular_speed
                msg.linear.x = self.current_speed
                msg.angular.z = self.current_steering_angle
                print("Turning left")
            elif key == 'd':  # Turn right
                self.current_steering_angle = self.angular_speed
                msg.linear.x = self.current_speed
                msg.angular.z = self.current_steering_angle
                print("Turning right")
            elif key == 's':  # Stop
                self.current_speed = 0.0
                self.current_steering_angle = 0.0
                msg.linear.x = self.current_speed
                msg.angular.z = self.current_steering_angle
                print("Stopping")
            elif key == 'x':  # Move backward
                self.current_speed = -self.linear_speed
                msg.linear.x = self.current_speed
                msg.angular.z = self.current_steering_angle
                print("Reversing")
            elif key is None:  # No key press
                continue
            else:
                print(f"Unknown key: {key}")
                continue

            # Publish the message
            self.publisher.publish(msg)

    def stop(self):
        """Stop the robot by publishing a zero-motion message."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def restore_terminal_settings(self):
        """Restore the original terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)


def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopAckermann()

    try:
        teleop.control_loop()
    except KeyboardInterrupt:
        print("\nExiting teleop...")
    finally:
        teleop.stop()
        teleop.restore_terminal_settings()  # Restore terminal settings
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
