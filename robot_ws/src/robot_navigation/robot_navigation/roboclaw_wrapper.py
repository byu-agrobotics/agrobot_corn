import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from roboclaw_3 import Roboclaw

# Roboclaw and Robot Constants
ROBOCLAW_ADDR = 0x80
ROBOCLAW_BAUD = 115200
ROBOCLAW_NAME = "/dev/roboclaw"
TIMEOUT_THRESHOLD = 2  # seconds

ENABLE_MOTORS = True

# --- YOU MUST CONFIGURE THESE VALUES FOR YOUR ROBOT ---
WHEEL_BASE = 0.3  # meters - Distance between the center of the left and right wheels
MAX_ROBOT_SPEED_MPS = 0.5  # m/s - Maximum speed your robot can achieve at Roboclaw command 127
ROBOCLAW_MAX_CMD_VAL = 127 # Maximum speed value for Roboclaw commands (typically 0-127)
# --- END OF ROBOT SPECIFIC CONFIGURATION ---

class RoboclawWrapper(Node):
    '''
    :author: Nelson Durrant (w Google Gemini 2.5 Pro)
    :date: May 2024

    Node that interfaces with the roboclaw motor controller to drive the robot.

    Subscribers:
         - cmd_vel (geometry_msgs/msg/Twist)
    '''

    def __init__(self):
        super().__init__('roboclaw_wrapper')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',  # Standard topic name for velocity commands
            self.cmd_vel_callback,
            10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Initialize robot parameters
        self.wheel_base = self.declare_parameter('wheel_base', WHEEL_BASE).get_parameter_value().double_value
        self.max_robot_speed_mps = self.declare_parameter('max_robot_speed_mps', MAX_ROBOT_SPEED_MPS).get_parameter_value().double_value
        global ENABLE_MOTORS # Use global to modify module-level variable
        
        self.get_logger().info(f"DriveInterface initialized with wheel_base: {self.wheel_base} m, max_robot_speed_mps: {self.max_robot_speed_mps} m/s")


        if ENABLE_MOTORS:
            try:
                self.roboclaw = Roboclaw(ROBOCLAW_NAME, ROBOCLAW_BAUD)
                if self.roboclaw.Open():
                    self.get_logger().info('Roboclaw port opened successfully.')
                    self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)  # left side
                    self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)  # right side
                else:
                    self.get_logger().error('Failed to open Roboclaw port. MOTORS DISABLED.')
                    self.roboclaw = None # Ensure roboclaw is None if not opened
                    ENABLE_MOTORS = False
            except Exception as e:
                self.get_logger().error(f'Exception opening Roboclaw: {str(e)}. MOTORS DISABLED.')
                self.roboclaw = None
                ENABLE_MOTORS = False
        else:
            self.roboclaw = None # Ensure roboclaw attribute exists even if motors are disabled
            self.get_logger().warn('Motors are disabled by configuration (ENABLE_MOTORS=False).')

        self.last_time_cmd_received = time.time() # Initialize with current time

    def cmd_vel_callback(self, msg):
        '''
        Callback function for the cmd_vel subscriber.

        :param msg: The Twist message.
        :type msg: geometry_msgs.msg.Twist
        '''

        global ENABLE_MOTORS # Access the global ENABLE_MOTORS

        if not ENABLE_MOTORS or self.roboclaw is None:
            self.get_logger().warn('Motors are disabled or Roboclaw not initialized, ignoring cmd_vel.')
            return

        linear_x = msg.linear.x  # Forward/backward velocity
        angular_z = msg.angular.z  # Rotational velocity around Z axis

        # Differential drive kinematics:
        # v_left = linear_x - (angular_z * wheel_base / 2)
        # v_right = linear_x + (angular_z * wheel_base / 2)
        target_left_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        target_right_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        # Scale m/s to Roboclaw command value (0-ROBOCLAW_MAX_CMD_VAL)
        # Ensure max_robot_speed_mps is not zero to avoid division by zero
        if self.max_robot_speed_mps == 0:
            self.get_logger().error("max_robot_speed_mps is zero, cannot scale motor commands.")
            return

        left_cmd_scaled_float = (target_left_mps / self.max_robot_speed_mps) * ROBOCLAW_MAX_CMD_VAL
        right_cmd_scaled_float = (target_right_mps / self.max_robot_speed_mps) * ROBOCLAW_MAX_CMD_VAL
        
        self.get_logger().info(f"CmdVel: lin_x={linear_x:.2f}, ang_z={angular_z:.2f} -> L_mps={target_left_mps:.2f}, R_mps={target_right_mps:.2f} -> L_raw={left_cmd_scaled_float:.2f}, R_raw={right_cmd_scaled_float:.2f}")

        # Determine direction and prepare command value for Roboclaw for Left Motor (M1)
        if left_cmd_scaled_float >= 0:
            left_command_val = min(int(left_cmd_scaled_float), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, left_command_val)
        else:
            left_command_val = min(int(abs(left_cmd_scaled_float)), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.BackwardM1(ROBOCLAW_ADDR, left_command_val)

        # Determine direction and prepare command value for Roboclaw for Right Motor (M2)
        if right_cmd_scaled_float >= 0:
            right_command_val = min(int(right_cmd_scaled_float), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, right_command_val)
        else:
            right_command_val = min(int(abs(right_cmd_scaled_float)), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.BackwardM2(ROBOCLAW_ADDR, right_command_val)
        
        self.get_logger().info(f"Roboclaw Cmds: L={left_command_val}, R={right_command_val} (Directions: L_fwd={left_cmd_scaled_float>=0}, R_fwd={right_cmd_scaled_float>=0})")

        self.last_time_cmd_received = time.time()

    def timer_callback(self):
        '''
        Timer callback function to check for expired commands.
        If no command is received within TIMEOUT_THRESHOLD, motors are stopped.
        '''
        global ENABLE_MOTORS # Access the global ENABLE_MOTORS

        if not ENABLE_MOTORS or self.roboclaw is None:
            return

        if time.time() - self.last_time_cmd_received > TIMEOUT_THRESHOLD:
            self.get_logger().warn(f'Drive command timeout ({TIMEOUT_THRESHOLD}s), stopping motors.')
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)
            # Reset last_time_cmd_received to prevent continuous warnings until a new command arrives
            self.last_time_cmd_received = time.time()


def main(args=None):
    rclpy.init(args=args)

    roboclaw_wrapper_node = RoboclawWrapper()
    rclpy.spin(roboclaw_wrapper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    if roboclaw_wrapper_node.roboclaw is not None and ENABLE_MOTORS: # Ensure motors are stopped on shutdown
        roboclaw_wrapper_node.get_logger().info('Shutting down, stopping motors.')
        roboclaw_wrapper_node.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)
        roboclaw_wrapper_node.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)

    roboclaw_wrapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()