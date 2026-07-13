#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero import DigitalInputDevice


class BeamBreakNode(Node):
    """
    Reads a digital beam-break sensor and publishes whether the beam is broken.

    Published topic:
        /beam_break/blocked

    Message:
        True  -> beam is broken
        False -> beam is clear
    """

    def __init__(self):
        super().__init__('beam_break_node')

        # GPIO Zero uses BCM GPIO numbering.
        #
        # Physical header pin 13 corresponds to BCM GPIO27.
        self.declare_parameter('gpio_pin', 27)

        # Most break-beam receivers are active-low:
        #     LOW  = beam broken
        #     HIGH = beam detected
        #
        # Change this parameter to false if your sensor behaves oppositely.
        self.declare_parameter('active_low', False)

        # Publishing frequency in hertz.
        self.declare_parameter('publish_rate', 20.0)

        # Simple software filtering. The signal must remain changed for this
        # many consecutive readings before the published state is changed.
        self.declare_parameter('required_stable_readings', 3)

        gpio_pin = (
            self.get_parameter('gpio_pin')
            .get_parameter_value()
            .integer_value
        )

        self.active_low = (
            self.get_parameter('active_low')
            .get_parameter_value()
            .bool_value
        )

        publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value()
            .double_value
        )

        self.required_stable_readings = (
            self.get_parameter('required_stable_readings')
            .get_parameter_value()
            .integer_value
        )

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero.')

        if self.required_stable_readings < 1:
            raise ValueError(
                'required_stable_readings must be at least 1.'
            )

        # pull_up=True enables the Raspberry Pi's internal pull-up resistor.
        #
        # This is appropriate for many active-low/open-collector beam-break
        # receivers. If your sensor actively drives both HIGH and LOW, you
        # may need to change pull_up to None after checking its datasheet.
        self.beam_input = DigitalInputDevice(
            pin=gpio_pin,
            pull_up=True
        )

        self.publisher = self.create_publisher(
            Bool,
            '/beam_break/blocked',
            10
        )

        self.last_raw_state = None
        self.stable_reading_count = 0
        self.filtered_beam_broken = False

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.read_and_publish
        )

        self.get_logger().info(
            f'Beam-break node started on BCM GPIO{gpio_pin}.'
        )

        self.get_logger().info(
            f'Beam-break active-low mode: {self.active_low}'
        )

    def read_beam_broken(self) -> bool:
        """
        Return True when the beam is broken.
        """

        pin_is_high = bool(self.beam_input.value)

        if self.active_low:
            # Active-low sensor:
            # LOW means the beam is broken.
            return not pin_is_high

        # Active-high sensor:
        # HIGH means the beam is broken.
        return pin_is_high

    def read_and_publish(self):
        raw_beam_broken = self.read_beam_broken()

        # Require several identical readings before accepting a state change.
        # This prevents brief electrical glitches from changing the FSM state.
        if raw_beam_broken == self.last_raw_state:
            self.stable_reading_count += 1
        else:
            self.last_raw_state = raw_beam_broken
            self.stable_reading_count = 1

        if (
            self.stable_reading_count
            >= self.required_stable_readings
        ):
            if raw_beam_broken != self.filtered_beam_broken:
                self.filtered_beam_broken = raw_beam_broken

                if self.filtered_beam_broken:
                    self.get_logger().info('Beam broken.')
                else:
                    self.get_logger().info('Beam restored.')

        message = Bool()
        message.data = self.filtered_beam_broken
        self.publisher.publish(message)

    def destroy_node(self):
        self.beam_input.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BeamBreakNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
