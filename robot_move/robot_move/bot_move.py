import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import serial
import json
import math

class AckermannControl(Node):
    def __init__(self):
        super().__init__('ackermann_control')

        # Serial port parameters
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.01)

        # ROS 2 publishers and subscribers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer to read feedback from Arduino
        self.create_timer(0.02, self.read_feedback)

        self.get_logger().info("Ackermann Control Node Initialized!")

    def cmd_vel_callback(self, msg):
        # Log received cmd_vel message
        self.get_logger().info(f"Received cmd_vel: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")

        # Map `angular.z` to steering angle (-45 to 45 degrees)
        # steering_angle = msg.angular.z * 45  # Map angular.z to ±45 degrees
        # steering_angle = max(-45, min(45, steering_angle))  # Clamp between -45 and 45
        steering_angle = msg.angular.z

        # Map `linear.x` to wheel velocities (-1 to 1)
        left_wheel_velocity = max(-1, min(1, msg.linear.x))  # Clamp between -1 and 1
        right_wheel_velocity = left_wheel_velocity  # Symmetric velocity for both wheels

        # Formulate the command as JSON
        command = {
            "left_wheel_velocity": left_wheel_velocity,
            "right_wheel_velocity": right_wheel_velocity,
            "steering_angle": steering_angle  # Send directly as ±45 degrees
        }

        # Send the command to Arduino
        command_str = json.dumps(command) + '\n'  # Add newline for Arduino compatibility
        try:
            self.serial.write(command_str.encode('utf-8'))
            self.get_logger().info(f"Sent command: {command_str}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def read_feedback(self):
        try:
            # Read a line from the serial port
            raw_line = self.serial.readline().decode('utf-8').strip()

            if raw_line:
                self.get_logger().info(f"Raw feedback from Arduino: {raw_line}")

                # Parse JSON feedback
                feedback = json.loads(raw_line)
                self.publish_odometry(feedback)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to decode JSON from Arduino: {raw_line}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in feedback processing: {e}")

    def publish_odometry(self, feedback):
        try:
            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # Set position and orientation
            odom.pose.pose.position.x = float(feedback["bot_x"])
            odom.pose.pose.position.y = float(feedback["bot_y"])
            odom.pose.pose.orientation.z = math.sin(feedback["theta"] / 2.0)
            odom.pose.pose.orientation.w = math.cos(feedback["theta"] / 2.0)

            # Set velocity
            odom.twist.twist.linear.x = float(feedback["velocity"])

            # Publish odometry
            self.odom_publisher.publish(odom)
            self.get_logger().info(f"Published odometry: {odom}")

        except KeyError as e:
            self.get_logger().warn(f"Missing key in feedback: {e}")
        except ValueError as e:
            self.get_logger().warn(f"Invalid value in feedback: {e}")

def main(args=None):
    rclpy.init(args=args)
    ackermann_control = AckermannControl()

    try:
        rclpy.spin(ackermann_control)
    except KeyboardInterrupt:
        ackermann_control.get_logger().info('Shutting down Ackermann Control Node...')
    finally:
        ackermann_control.serial.close()
        ackermann_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
