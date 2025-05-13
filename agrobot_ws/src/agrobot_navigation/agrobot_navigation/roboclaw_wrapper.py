import rclpy
from rclpy.node import Node
# from agro_interfaces.msg import Drive
from rclpy.qos import qos_profile_sensor_data
from roboclaw_3 import Roboclaw


class DriveSubscriber(Node):

    def __init__(self):
        super().__init__("roboclaw_wrapper")

        self.front_left = 0.0
        self.front_right = 0.0
        self.back_left = 0.0
        self.back_right = 0.0
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/roboclaw", 115200)
        self.roboclaw.Open()
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)
        self.get_logger().info("Roboclaw has been setup")

        # self.subscription = self.create_subscription(
        #     Drive, "drive", self.listener_callback, qos_profile_sensor_data
        # )
        # self.subscription  # prevent unused variable warning


    # def listener_callback(self, msg):
    #     ##############################################################
    #     # ADD CODE HERE TO EXECUTE WHEN A MESSAGE IS RECEIVED
    #     ##############################################################

    #     self.front_left = msg.fl_speed
    #     self.front_right = msg.fr_speed
    #     self.back_left = msg.bl_speed
    #     self.back_right = msg.br_speed
    #     self.get_logger().info(f"speeds: {self.front_left} {self.back_left} {self.front_right} {self.back_right}")

    #     if self.front_left >= 0:
    #         self.roboclaw.ForwardM2(self.address, int(self.front_left))
    #     else:
    #         self.roboclaw.BackwardM2(self.address, -1*int(self.front_left))

    #     if self.front_right >= 0:
    #         self.roboclaw.ForwardM1(self.address, int(self.front_right))
    #     else:
    #         self.roboclaw.BackwardM1(self.address, -1*int(self.front_right))

    #     ##############################################################
    #     # END CODE HERE TO EXECUTE WHEN A MESSAGE IS RECEIVED
    #     ##############################################################


def main(args=None):
    rclpy.init(args=args)
    try:
        drive_sub = DriveSubscriber()

        rclpy.spin(drive_sub)
    except:
        pass
        drive_sub.get_logger().info("drive shutting down")

    # drive_sub.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()