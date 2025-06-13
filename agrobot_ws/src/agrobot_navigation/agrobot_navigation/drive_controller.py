import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveStraight, Turn, Center
from simple_pid import PID

# Thresholds for completing actions
STABILITY_THRESHOLD = 5  # Number of consecutive readings required to confirm stability
CENTERING_TOLERANCE_MM = 10 # Tolerance in mm for centering
DRIVE_STRAIGHT_TOLERANCE_MM = 5 # Tolerance in mm for reaching the front distance
TURN_ERROR_TOLERANCE_MM = 40 # Total sensor error tolerance for a successful turn
TURN_TIMEOUT_S = 15.0  # Timeout for the turn action in seconds

class DriveController(Node):
    """
    Controls the robot's driving based on Time-of-Flight (ToF) sensor data.
    This node provides action servers for centering, driving straight, and turning.
    It uses PID controllers for smooth and accurate motion.
    """
    def __init__(self):
        super().__init__('drive_controller')
        
        # Use a ReentrantCallbackGroup to allow callbacks to run in parallel
        cb_group = ReentrantCallbackGroup()

        # ROS parameters for PID and speed settings
        self.declare_parameter('p_forward', 0.5)
        self.declare_parameter('i_forward', 0.0)
        self.declare_parameter('d_forward', 0.1)
        self.declare_parameter('p_lateral', 0.2)
        self.declare_parameter('i_lateral', 0.0)
        self.declare_parameter('d_lateral', 0.05)
        self.declare_parameter('turn_speed', 50)

        # Get params
        p_forward = self.get_parameter('p_forward').get_parameter_value().double_value
        i_forward = self.get_parameter('i_forward').get_parameter_value().double_value
        d_forward = self.get_parameter('d_forward').get_parameter_value().double_value
        p_lateral = self.get_parameter('p_lateral').get_parameter_value().double_value
        i_lateral = self.get_parameter('i_lateral').get_parameter_value().double_value
        d_lateral = self.get_parameter('d_lateral').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().integer_value

        # PID controllers
        self.pid_forward = PID(p_forward, i_forward, d_forward, setpoint=0)
        self.pid_lateral = PID(p_lateral, i_lateral, d_lateral, setpoint=0)
        self.pid_forward.output_limits = (-100, 100)  # Motor speed percentage
        self.pid_lateral.output_limits = (-50, 50)    # Motor speed percentage
        
        # ToF data and action goal state
        self.tof_data = None
        self.active_goal = False
        
        # Create publishers
        self.drive_pub = self.create_publisher(DriveCommand, 'cmd/drive', 10)

        # Create subscribers
        self.tof_sub = self.create_subscription(
            ToFData, 'tof/data', self.tof_callback, 10, callback_group=cb_group
        )

        # Create action servers
        self.center_action_server = ActionServer(
            self, Center, 'control/center',
            execute_callback=self.execute_center_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.drive_straight_action_server = ActionServer(
            self, DriveStraight, 'control/drive_straight',
            execute_callback=self.drive_straight_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.turn_action_server = ActionServer(
            self, Turn, 'control/turn',
            execute_callback=self.execute_turn_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.get_logger().info("Drive Controller node is ready.")

    ## ------------------ ##
    ## -- Core Methods -- ##
    ## ------------------ ##

    def tof_callback(self, msg):
        """Stores the latest ToF sensor data."""
        self.tof_data = msg

    def stop_robot(self):
        """Publishes a zero-speed command to stop the robot."""
        drive_cmd = DriveCommand()
        drive_cmd.left_speed = 0
        drive_cmd.right_speed = 0
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info("Robot stopped.")

    ## ---------------------------- ##
    ## -- Action Server Callbacks-- ##
    ## ---------------------------- ##

    def goal_callback(self, goal_request):
        """Accept or reject a new goal request."""
        if self.active_goal:
            self.get_logger().warn('Another goal is active, rejecting new goal.')
            return GoalResponse.REJECT
        self.active_goal = True
        self.get_logger().info(f'Accepting goal for {goal_request.__class__.__name__}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept a request to cancel a goal."""
        self.get_logger().info('Received request to cancel goal.')
        self.active_goal = False
        self.stop_robot()
        return CancelResponse.ACCEPT

# ========== Turn Callback ========= #

    def execute_turn_callback(self, goal_handle):
        """Executes a 90-degree turn using ToF sensors for feedback."""
        self.get_logger().info('Executing goal: Turn')
        angle = goal_handle.request.angle

        if self.tof_data is None:
            self.get_logger().error("No ToF data available to start turn.")
            goal_handle.abort()
            self.active_goal = False
            return Turn.Result(success=False)

        # Store initial distances to calculate target distances
        initial_front = self.tof_data.front
        initial_back = self.tof_data.back
        initial_left = self.tof_data.left
        initial_right = self.tof_data.right

        # Determine target distances based on a 90-degree turn
        if angle > 0:  # Right turn
            target_front, target_right, target_back, target_left = initial_left, initial_front, initial_right, initial_back
        else:  # Left turn
            target_front, target_right, target_back, target_left = initial_right, initial_back, initial_front, initial_left
        
        # Set motor speeds for turning
        drive_cmd = DriveCommand()
        turn_speed_val = self.turn_speed if angle > 0 else -self.turn_speed
        drive_cmd.left_speed = turn_speed_val
        drive_cmd.right_speed = -turn_speed_val
        self.drive_pub.publish(drive_cmd)
        
        start_time = self.get_clock().now()
        stability_count = 0
        rate = self.create_rate(20) # 20 Hz loop

        while rclpy.ok() and self.active_goal:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > TURN_TIMEOUT_S:
                self.get_logger().error("Turn action timed out!")
                goal_handle.abort()
                break

            if self.tof_data:
                # Calculate the total absolute error from target distances
                total_error = sum([
                    abs(self.tof_data.front - target_front),
                    abs(self.tof_data.back - target_back),
                    abs(self.tof_data.left - target_left),
                    abs(self.tof_data.right - target_right)
                ])

                if total_error < TURN_ERROR_TOLERANCE_MM:
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        self.get_logger().info("Turn successful!")
                        goal_handle.succeed()
                        break
                else:
                    stability_count = 0
            
            rate.sleep()

        self.stop_robot()
        self.active_goal = False
        return Turn.Result(success=True if goal_handle.status == goal_handle.Status.SUCCEEDED else False)

# ========== Center Callback ========= #

    def execute_center_callback(self, goal_handle):
        """Executes the centering action using a lateral PID controller."""
        self.get_logger().info('Executing goal: Center')
        
        if self.tof_data is None:
            self.get_logger().error('No ToF data available to start centering.')
            goal_handle.abort()
            self.active_goal = False
            return Center.Result(success=False)

        drive_cmd = DriveCommand()
        rate = self.create_rate(20) # 20 Hz loop
        stability_count = 0
        self.pid_lateral.reset()

        while rclpy.ok() and self.active_goal:
            if self.tof_data is None:
                rate.sleep()
                continue
            
            lateral_error = self.tof_data.left - self.tof_data.right
            
            if abs(lateral_error) < CENTERING_TOLERANCE_MM:
                stability_count += 1
                if stability_count >= STABILITY_THRESHOLD:
                    self.get_logger().info("Centering successful.")
                    goal_handle.succeed()
                    break
            else:
                stability_count = 0

            turn_speed = self.pid_lateral(lateral_error)
            drive_cmd.left_speed = int(-turn_speed)
            drive_cmd.right_speed = int(turn_speed)
            self.drive_pub.publish(drive_cmd)
            
            rate.sleep()
        
        self.stop_robot()
        self.active_goal = False
        return Center.Result(success=True if goal_handle.status == goal_handle.Status.SUCCEEDED else False)

# ========== Drive Straight Callback ========= #

    def drive_straight_execute_callback(self, goal_handle):
        """Executes the drive straight action using forward and lateral PID controllers."""
        self.get_logger().info('Executing goal: Drive Straight')
        
        if self.tof_data is None:
            self.get_logger().error('No ToF data available to start driving straight.')
            goal_handle.abort()
            self.active_goal = False
            return DriveStraight.Result(success=False)

        desired_front_distance = goal_handle.request.front_distance
        drive_cmd = DriveCommand()
        rate = self.create_rate(20) # 20 Hz loop
        stability_count = 0

        self.pid_forward.reset()
        self.pid_lateral.reset()
        
        while rclpy.ok() and self.active_goal:
            if self.tof_data is None:
                rate.sleep()
                continue

            forward_error = self.tof_data.front - desired_front_distance
            lateral_error = self.tof_data.left - self.tof_data.right
            
            if abs(forward_error) < DRIVE_STRAIGHT_TOLERANCE_MM:
                stability_count += 1
                if stability_count >= STABILITY_THRESHOLD:
                    self.get_logger().info("Drive straight successful.")
                    goal_handle.succeed()
                    break
            else:
                stability_count = 0

            forward_speed = -self.pid_forward(forward_error)
            turn_correction = self.pid_lateral(lateral_error)

            drive_cmd.left_speed = int(forward_speed - turn_correction)
            drive_cmd.right_speed = int(forward_speed + turn_correction)
            self.drive_pub.publish(drive_cmd)
            
            rate.sleep()

        self.stop_robot()
        self.active_goal = False
        return DriveStraight.Result(success=True if goal_handle.status == goal_handle.Status.SUCCEEDED else False)

def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveController()
    
    # Use a MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(drive_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()