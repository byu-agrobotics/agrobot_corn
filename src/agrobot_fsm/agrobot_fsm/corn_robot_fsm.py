#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from agrobot_interfaces.srv import MoveServo
from tutorial_interfaces.msg import Num


class RobotState(Enum):
    NAVIGATING = auto()
    SEEDING = auto()
    REMOVING = auto()


class CornRobotFSM(Node):

    def __init__(self):
        super().__init__('corn_robot_fsm')

        self.state = RobotState.NAVIGATING

        # Most recent result published by the detection camera.
        self.latest_detection = ''

        # Prevent another action from starting while a service sequence
        # is already running.
        self.busy = False

        # Prevent repeated actuation if the camera publishes the same
        # centered result multiple times for one object.
        self.base_centered_latched = False
        self.yellow_stalk_centered_latched = False

        # Navigation command publisher:
        # num=0 stops mini_nav
        # num=1 starts mini_nav in DRIVE_ROW_LEFT
        self.nav_command_pub = self.create_publisher(
            Num,
            '/topic',
            10
        )

        # Detection subscriber.
        self.detection_sub = self.create_subscription(
            String,
            '/detected_object',
            self.detection_callback,
            10
        )

        # Actuation service clients.
        self.seed_client = self.create_client(
            MoveServo,
            '/seed_servo'
        )

        self.remove_client = self.create_client(
            MoveServo,
            '/remove_servo'
        )

        self.get_logger().info(
            'Corn robot FSM started in NAVIGATING state'
        )

    # ==============================================================
    # Navigation commands
    # ==============================================================

    def stop_navigation(self):
        nav_msg = Num()
        nav_msg.num = 0

        self.nav_command_pub.publish(nav_msg)

        self.get_logger().info(
            'Published navigation stop command: num=0'
        )

    def start_navigation(self):
        nav_msg = Num()
        nav_msg.num = 1

        self.nav_command_pub.publish(nav_msg)

        self.get_logger().info(
            'Published navigation start command: num=1'
        )

    # ==============================================================
    # Detection callback
    # ==============================================================

    def detection_callback(self, msg: String):
        detection = msg.data.strip()

        if detection != self.latest_detection:
            self.get_logger().info(
                f'Detection changed: "{self.latest_detection}" '
                f'-> "{detection}"'
            )

        self.latest_detection = detection

        # Reset each latch once its corresponding object is no longer
        # centered. This allows a later base or stalk to trigger an action.
        if detection != 'base:centered':
            self.base_centered_latched = False

        if detection != 'yellow_stalk:centered':
            self.yellow_stalk_centered_latched = False

        # Do not begin another action while an asynchronous service
        # sequence is running.
        if self.busy:
            return

        if self.state != RobotState.NAVIGATING:
            return

        # ----------------------------------------------------------
        # Empty planting location
        # ----------------------------------------------------------

        if (
            detection == 'base:centered'
            and not self.base_centered_latched
        ):
            self.base_centered_latched = True
            self.state = RobotState.SEEDING
            self.busy = True

            self.get_logger().info(
                'Base centered. Stopping navigation to dispense a seed.'
            )

            self.stop_navigation()
            self.seed_after_stop()
            return

        # ----------------------------------------------------------
        # Yellow stalk
        # ----------------------------------------------------------

        if (
            detection == 'yellow_stalk:centered'
            and not self.yellow_stalk_centered_latched
        ):
            self.yellow_stalk_centered_latched = True
            self.state = RobotState.REMOVING
            self.busy = True

            self.get_logger().info(
                'Yellow stalk centered. '
                'Stopping navigation to remove the stalk.'
            )

            self.stop_navigation()
            self.remove_after_stop()
            return

        # These messages require no action:
        #
        # base:not_centered
        # yellow_stalk:not_centered

    # ==============================================================
    # Finish actuation and restart navigation
    # ==============================================================

    def finish_action_and_restart(self):
        self.get_logger().info(
            'Actuation complete. Restarting navigation.'
        )

        self.start_navigation()

        self.state = RobotState.NAVIGATING
        self.busy = False

    # ==============================================================
    # Seed operation
    # ==============================================================

    def seed_after_stop(self):
        if not self.seed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                'Seed service is unavailable'
            )

            self.finish_action_and_restart()
            return

        request = MoveServo.Request()
        request.request = 'go'

        future = self.seed_client.call_async(request)
        future.add_done_callback(self.seed_done_callback)

    def seed_done_callback(self, future):
        try:
            response = future.result()

            self.get_logger().info(
                f'Seed service response: {response.response}'
            )

        except Exception as error:
            self.get_logger().error(
                f'Seed service call failed: {error}'
            )

        self.finish_action_and_restart()

    # ==============================================================
    # Removal operation
    # ==============================================================

    def remove_after_stop(self):
        if not self.remove_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                'Removal service is unavailable'
            )

            self.finish_action_and_restart()
            return

        request = MoveServo.Request()
        request.request = 'go'

        future = self.remove_client.call_async(request)
        future.add_done_callback(self.remove_done_callback)

    def remove_done_callback(self, future):
        try:
            response = future.result()

            self.get_logger().info(
                f'Removal service response: {response.response}'
            )

        except Exception as error:
            self.get_logger().error(
                f'Removal service call failed: {error}'
            )

        self.finish_action_and_restart()


def main(args=None):
    rclpy.init(args=args)

    node = CornRobotFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
