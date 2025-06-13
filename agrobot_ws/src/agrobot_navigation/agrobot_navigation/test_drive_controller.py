import rclpy
import time
import random
from rclpy.action import ActionClient
from rclpy.node import Node

from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveStraight

class DriveControllerTester(Node):
    """
    A test node that simulates a robot for drive_controller.py.
    - It simulates the robot's position based on drive commands.
    - It publishes ToF data with noise based on its simulated position.
    - It calls the /control/drive_straight action server to start the test.
    """

    def __init__(self):
        super().__init__('drive_controller_tester')

        # --- Simulation State ---
        self.sim_front = 200.0
        self.sim_back = 190.0
        self.sim_left = 150.0
        self.sim_right = 150.0
        self.last_forward_speed = 0.0
        self.SPEED_TO_DISTANCE_SCALE = 0.005

        # --- ROS2 Communications ---
        self.tof_publisher = self.create_publisher(ToFData, 'tof/data', 10)
        self.drive_sub = self.create_subscription(
            DriveCommand,
            'cmd/drive',
            self.drive_cmd_callback,
            10)
        self.simulation_timer = self.create_timer(0.05, self.update_simulation)
        self._action_client = ActionClient(self, DriveStraight, 'control/drive_straight')

    def drive_cmd_callback(self, msg: DriveCommand):
        """Listen to the controller's speed commands."""
        self.last_forward_speed = (msg.left_speed + msg.right_speed) / 2.0

    def update_simulation(self):
        """This is the core of our simulator."""
        distance_change = self.last_forward_speed * self.SPEED_TO_DISTANCE_SCALE
        self.sim_front -= distance_change

        msg = ToFData()
        msg.front = self.sim_front + random.uniform(-1.0, 1.0)
        msg.back = self.sim_back + random.uniform(-1.0, 1.0)
        msg.left = self.sim_left + random.uniform(-0.5, 0.5)
        msg.right = self.sim_right + random.uniform(-0.5, 0.5)

        self.tof_publisher.publish(msg)
        
        # ADD THIS LOGGING LINE BACK IN WITH MORE DETAIL
        self.get_logger().info(f'Simulated Pos: [front={self.sim_front:.2f}] | Last Speed Cmd: [{self.last_forward_speed:.2f}]')


    def send_goal(self):
        """Waits for connections, primes the controller, then sends a goal."""
        self.get_logger().info('Waiting for ToF data subscriber...')
        while self.tof_publisher.get_subscription_count() == 0:
            time.sleep(0.1)
        self.get_logger().info('Subscriber found.')
        
        self.get_logger().info('Priming controller with first ToF message...')
        self.update_simulation()
        time.sleep(0.1)

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = DriveStraight.Goal()
        goal_msg.front_distance = 100.0

        self.get_logger().info(f'Sending goal request: drive to {goal_msg.front_distance}mm from front wall.')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result from drive_controller: Success = {result.success}')
        self.get_logger().info('Test finished, shutting down test node.')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tester_node = DriveControllerTester()
    try:
        tester_node.send_goal()
        rclpy.spin(tester_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and tester_node.executor:
            tester_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()