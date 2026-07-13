#!/usr/bin/env python3

from enum import Enum, auto
from typing import Callable

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

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

        self.nav_command_pub = self.create_publisher(
            Num,
            '/topic',
            10
        )

        # ----------------------------------------------------------
        # Detection subscriber
        # ----------------------------------------------------------

        self.detection_sub = self.create_subscription(
            String,
            '/detected_object',
            self.detection_callback,
            10
        )

        # ----------------------------------------------------------
        # Navigation service clients
        # ----------------------------------------------------------

        self.pause_navigation_client = self.create_client(
            Trigger,
            '/navigation/pause'
        )

        self.resume_navigation_client = self.create_client(
            Trigger,
            '/navigation/resume'
        )

        # ----------------------------------------------------------
        # Actuation service clients
        # ----------------------------------------------------------

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
        # centered. This allows a later plot or stalk to trigger the action.
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
            nav_msg = Num()
            nav_msg.num = 0
            self.nav_command_pub.publish(nav_msg)
            self.base_centered_latched = True
            self.state = RobotState.SEEDING

            self.get_logger().info(
                'Base centered. Pausing navigation to dispense a seed.'
            )

            self.pause_navigation(self.seed_after_pause)
            return

        # ----------------------------------------------------------
        # Yellow stalk
        # ----------------------------------------------------------

        if (
            detection == 'yellow_stalk:centered'
            and not self.yellow_stalk_centered_latched
        ):
            nav_msg = Num()
            nav_msg.num = 1
            self.nav_command_pub.publish(nav_msg)
            self.yellow_stalk_centered_latched = True
            self.state = RobotState.REMOVING

            self.get_logger().info(
                'Yellow stalk centered. '
                'Pausing navigation to remove the stalk.'
            )

            self.pause_navigation(self.remove_after_pause)
            return

        # The following messages require no service call:
        #
        # base:not_centered
        # yellow_stalk:not_centered
        #
        # Navigation continues normally until an object becomes centered.

    # ==============================================================
    # Navigation service calls
    # ==============================================================

    def pause_navigation(self, next_action: Callable[[], None]):
        if not self.pause_navigation_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                'Navigation pause service is unavailable'
            )

            self.state = RobotState.NAVIGATING
            self.busy = False
            return

        self.busy = True

        request = Trigger.Request()
        future = self.pause_navigation_client.call_async(request)

        def pause_done_callback(completed_future):
            try:
                response = completed_future.result()
            except Exception as error:
                self.get_logger().error(
                    f'Navigation pause call failed: {error}'
                )

                self.state = RobotState.NAVIGATING
                self.busy = False
                return

            if not response.success:
                self.get_logger().error(
                    f'Navigation did not pause: {response.message}'
                )

                self.state = RobotState.NAVIGATING
                self.busy = False
                return

            self.get_logger().info('Navigation paused')

            # Keep busy=True while performing the seed or removal action.
            next_action()

        future.add_done_callback(pause_done_callback)

    def resume_navigation(self):
        if not self.resume_navigation_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                'Navigation resume service is unavailable'
            )

            # Navigation may still be paused, so the FSM is no longer busy,
            # but an operator may need to manually resume navigation.
            self.state = RobotState.NAVIGATING
            self.busy = False
            return

        request = Trigger.Request()
        future = self.resume_navigation_client.call_async(request)

        def resume_done_callback(completed_future):
            try:
                response = completed_future.result()

                if response.success:
                    self.get_logger().info(
                        'Navigation resumed in DRIVE_STRAIGHT'
                    )
                else:
                    self.get_logger().error(
                        f'Navigation did not resume: {response.message}'
                    )

            except Exception as error:
                self.get_logger().error(
                    f'Navigation resume call failed: {error}'
                )

            self.state = RobotState.NAVIGATING
            self.busy = False

        future.add_done_callback(resume_done_callback)

    # ==============================================================
    # Seed operation
    # ==============================================================

    def seed_after_pause(self):
        if not self.seed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                'Seed service is unavailable'
            )

            self.resume_navigation()
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

        self.resume_navigation()

    # ==============================================================
    # Removal operation
    # ==============================================================

    def remove_after_pause(self):
        if not self.remove_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                'Removal service is unavailable'
            )

            self.resume_navigation()
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

        self.resume_navigation()


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
