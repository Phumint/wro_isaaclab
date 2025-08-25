# Import necessary ROS2 and Python libraries
import rclpy
from rclpy.node import Node
# The change here is to import Float64 instead of Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import onnxruntime as ort
import numpy as np
import math
# Import ament_index_python to find the path to the ONNX file
from ament_index_python.packages import get_package_share_directory
import os

class PolicyControllerNode(Node):
    """
    ROS2 node that controls a differential drive and Ackermann steering robot
    using a pre-trained ONNX policy.
    """

    def __init__(self):
        super().__init__('policy_controller_node')
        self.get_logger().info('Policy Controller Node Initialized.')

        # --- Parameters (Update these to match your robot and environment) ---
        # Get the path to the package's shared directory
        package_share_directory = get_package_share_directory('my_robot_controller')
        # Correctly construct the full path to the ONNX policy file
        self.policy_path = os.path.join(package_share_directory, 'scripts', 'my_robot_policy.onnx')

        # --- Robot Physical Parameters (UPDATED from wro_env_square.py) ---
        self.WHEEL_RADIUS = 0.034    # Wheel radius in meters
        self.TRACK_WIDTH = 0.09      # Distance between the two rear wheels in meters (y-axis)
        self.WHEEL_BASE = 0.1425     # Distance between the front and rear axles in meters (x-axis)
        
        # --- ROS Topics (Update these to match your Gazebo setup) ---
        # Topic to subscribe to for robot pose and velocity.
        self.odom_topic = '/odom'
        
        # Topics to publish for Ackermann steering
        # We're changing the topics to match a standard ackermann_steering_controller setup.
        self.rear_wheel_topic = '/steer_bot/rear_wheel_controller/commands'
        self.steer_topic = '/steer_bot/steer_controller/commands'

        # --- ROS2 Setup ---
        # Publishers
        # We're now publishing two separate Float64 messages.
        self.rear_wheel_publisher = self.create_publisher(Float64, self.rear_wheel_topic, 10)
        self.steer_publisher = self.create_publisher(Float64, self.steer_topic, 10)

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)

        # Timer to run the control loop at a fixed rate (e.g., 20 Hz)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # --- Policy and State Variables ---
        self.ort_session = None
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_yaw = 0.0
        self.robot_lin_vel_x = 0.0
        self.robot_lin_vel_y = 0.0
        self.robot_ang_vel_z = 0.0
        self.last_throttle_action = 0.0
        self.last_steering_action = 0.0

        # Target waypoints (These will need to be dynamically updated in a real application)
        # For this example, we'll use the same square trajectory as in the Isaac Lab environment.
        self.waypoints = np.array([
            [1.0, 1.0], [0.0, 1.0], [-1.0, 1.0], [-1.0, 0.0],
            [-1.0, -1.0], [0.0, -1.0], [1.0, -1.0], [1.0, 0.0]
        ])
        self.current_waypoint_index = 0
        self.is_goal_reached = False
        
        # Load the ONNX model
        self.load_policy()

    def load_policy(self):
        """Loads the ONNX model from the specified file path."""
        try:
            # Use CPUExecutionProvider for simplicity, but you can change this for GPU acceleration
            self.ort_session = ort.InferenceSession(self.policy_path, providers=['CPUExecutionProvider'])
            self.get_logger().info(f'Successfully loaded ONNX policy: {self.policy_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX policy: {e}')
            self.ort_session = None

    def odom_callback(self, msg: Odometry):
        """Callback for the odometry subscriber to update the robot's pose and velocity."""
        # Update linear velocities in the robot's body frame
        self.robot_lin_vel_x = msg.twist.twist.linear.x
        self.robot_lin_vel_y = msg.twist.twist.linear.y
        
        # Update angular velocity
        self.robot_ang_vel_z = msg.twist.twist.angular.z
        
        # Update position
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y

        # Update yaw from quaternion
        (roll, pitch, yaw) = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.robot_yaw = yaw

    def get_observations(self):
        """
        Gathers and preprocesses the observations from the environment
        to match the input format of the ONNX policy.
        """
        # Calculate errors to the current waypoint
        target_pos_x = self.waypoints[self.current_waypoint_index, 0]
        target_pos_y = self.waypoints[self.current_waypoint_index, 1]

        # Calculate position error vector
        pos_error_x = target_pos_x - self.robot_pose_x
        pos_error_y = target_pos_y - self.robot_pose_y
        
        # Calculate heading to target
        heading_to_target = math.atan2(pos_error_y, pos_error_x)
        
        # Calculate heading error relative to robot's current yaw
        heading_error = heading_to_target - self.robot_yaw
        
        # Check for goal completion (if close to the current waypoint)
        distance_to_target = math.sqrt(pos_error_x**2 + pos_error_y**2)
        if distance_to_target < 0.2:  # Threshold for reaching waypoint
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            if self.current_waypoint_index == 0:
                self.is_goal_reached = True
                self.get_logger().info("All waypoints reached. Trajectory complete.")
            
        # Reconstruct the observation vector based on wro_env_square.py
        obs = np.array([
            distance_to_target,
            math.cos(heading_error),
            math.sin(heading_error),
            self.robot_lin_vel_x,
            self.robot_lin_vel_y,
            self.robot_ang_vel_z,
            self.last_throttle_action,
            self.last_steering_action
        ], dtype=np.float32)

        return obs.reshape(1, -1) # Reshape for the ONNX model input

    def control_loop(self):
        """
        Main control loop that runs on a timer.
        It gets observations, runs the policy, and publishes commands.
        """
        if self.ort_session is None or self.is_goal_reached:
            # If the model is not loaded or the goal is reached, stop publishing commands.
            # We're using Float64 messages now, so we need to publish them.
            self.rear_wheel_publisher.publish(Float64(data=0.0))
            self.steer_publisher.publish(Float64(data=0.0))
            return

        # 1. Get observations
        obs = self.get_observations()

        # 2. Run the ONNX model
        input_name = self.ort_session.get_inputs()[0].name
        output_name = self.ort_session.get_outputs()[0].name
        actions = self.ort_session.run([output_name], {input_name: obs})[0]

        # The policy output is assumed to be [throttle_command, steering_command]
        throttle_command = actions[0, 0]
        steering_command = actions[0, 1]

        # Store the last actions for the next observation
        self.last_throttle_action = throttle_command
        self.last_steering_action = steering_command

        # 3. Publish the commands
        # The policy's output directly corresponds to the commands we need to publish.
        # We need to scale them appropriately.
        
        # Create Float64 messages to publish
        rear_wheel_vel_msg = Float64()
        # Scale the throttle command, as done in the Isaac Lab training script.
        rear_wheel_vel_msg.data = float(throttle_command * 2.0)
        
        steer_angle_msg = Float64()
        # Scale the steering command to a valid angle (e.g., +/- pi/4 radians)
        steer_angle_msg.data = float(steering_command * (math.pi / 4))

        # Publish the messages to the correct topics
        self.rear_wheel_publisher.publish(rear_wheel_vel_msg)
        self.steer_publisher.publish(steer_angle_msg)

        self.get_logger().info(f"Published Command: Rear Wheel Velocity={rear_wheel_vel_msg.data:.2f}, Steering Angle={steer_angle_msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PolicyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
