# agrobot_test/agrobot_test/mock_tof_sensor.py

import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import ToFData
from agrobot_interfaces.srv import UpdateToF
import time

class MockToFSensor(Node):
    """
    This node simulates a Time-of-Flight (ToF) sensor array.
    It continuously publishes ToFData messages and provides a service
    to update the sensor readings, allowing for virtual testing of other nodes.
    """
    def __init__(self):
        super().__init__('mock_tof_sensor')
        
        # Publisher for the ToF sensor data
        self.tof_publisher = self.create_publisher(ToFData, 'tof/data', 10)
        
        # Service to allow external updates to the sensor values
        self.update_srv = self.create_service(UpdateToF, 'update_tof_data', self.update_tof_callback)
        
        # Timer to publish data at a regular interval (e.g., 20 Hz)
        self.timer = self.create_timer(0.05, self.publish_tof_data)
        
        # Initial default sensor data
        self.current_tof_data = ToFData()
        self.current_tof_data.front = 200  # in mm
        self.current_tof_data.back = 200   # in mm
        self.current_tof_data.left = 150   # in mm
        self.current_tof_data.right = 100  # in mm
        
        self.get_logger().info("Mock ToF Sensor started. Ready to publish data.")
        self.get_logger().info(f"Default values: front={self.current_tof_data.front}, back={self.current_tof_data.back}, left={self.current_tof_data.left}, right={self.current_tof_data.right}")

    def update_tof_callback(self, request, response):
        """
        Service callback to update the current ToF sensor data.
        This allows simulating changes in the environment.
        """
        self.get_logger().info(f"Updating ToF data to: front={request.front}, left={request.left}, right={request.right}")
        self.current_tof_data.front = request.front
        self.current_tof_data.left = request.left
        self.current_tof_data.right = request.right
        # You can add back sensor if needed for your tests
        
        response.success = True
        return response

    def publish_tof_data(self):
        """
        Publishes the current ToF data. Called by the timer.
        """
        self.tof_publisher.publish(self.current_tof_data)

def main(args=None):
    rclpy.init(args=args)
    mock_tof_sensor = MockToFSensor()
    rclpy.spin(mock_tof_sensor)
    mock_tof_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
