#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from agrobot_interfaces.srv import MoveServo


class BeamBreakRemove(Node):
    """
    Calls the removal service once each time the beam changes
    from clear to broken.

    Topic:
        /beam_break/blocked

    Service:
        /beam_break_remove
    """

    def __init__(self):
        super().__init__('beam_break_remove')

        # Current beam state.
        self.beam_broken = False

        # True while waiting for the seed-removal service response.
        self.service_call_in_progress = False

        # Becomes True after the beam has been observed clear.
        # This prevents repeated actuation while the beam remains blocked.
        self.armed = True

        self.beam_subscription = self.create_subscription(
            Bool,
            '/beam_break/blocked',
            self.beam_break_callback,
            10
        )

        self.seed_remove_client = self.create_client(
            MoveServo,
            '/remove_servo'
        )

        self.get_logger().info(
            'Waiting for /beam_break_remove service...'
        )

        while not self.seed_remove_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                '/remove_servo service is not available yet.'
            )

        self.get_logger().info(
            'Beam-break removal coordinator is ready.'
        )

    def beam_break_callback(self, message: Bool):
        """
        Called whenever a new beam-break status is published.
        """

        self.beam_broken = message.data

        # The beam is clear again, so allow the next break to trigger
        # another actuation.
        if not self.beam_broken:
            if not self.armed:
                self.get_logger().info(
                    'Beam restored. Ready for next object.'
                )

            self.armed = True
            return

        # At this point, the beam is broken.
        #
        # Only actuate if:
        #   1. The system has been re-armed by seeing a clear beam.
        #   2. No service call is already running.
        if self.armed and not self.service_call_in_progress:
            self.armed = False
            self.call_seed_remove()

    def call_seed_remove(self):
        """
        Send an asynchronous request to the seed-removal service.
        """

        self.service_call_in_progress = True

        request = MoveServo.Request()
        request.request = 'go'

        self.get_logger().info(
            'Beam broken. Calling removal service.'
        )

        future = self.seed_remove_client.call_async(request)
        future.add_done_callback(self.seed_remove_response_callback)

    def seed_remove_response_callback(self, future):
        """
        Handle the response from the seed-removal service.
        """

        self.service_call_in_progress = False

        try:
            response = future.result()

            self.get_logger().info(
                f'Removal response: {response.response}'
            )

        except Exception as exception:
            self.get_logger().error(
                f'Removal service call failed: {exception}'
            )

        if self.beam_broken:
            self.get_logger().info(
                'Actuation complete. Waiting for beam to clear.'
            )
        else:
            self.get_logger().info(
                'Actuation complete. Ready for next beam break.'
            )


def main(args=None):
    rclpy.init(args=args)

    node = BeamBreakRemove()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
