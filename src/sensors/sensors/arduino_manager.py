from std_msgs.msg import Float32MultiArray
from rcl_interfaces.srv import DescribeParameters

import rclpy
import serial
from rclpy.node import Node, Publisher
import json


class ArduinoManager(Node):
    timestamp: int
    publishers: dict[str, Publisher]

    def __init__(self):
        super().__init__("arduino_manager")

        self.co2_sensor_publishers = {
            "co2_1": self.create_publisher(Float32MultiArray, "/co2_sensor_1", 10),
            "co2_2": self.create_publisher(Float32MultiArray, "/co2_sensor_2", 10),
            "co2_3": self.create_publisher(Float32MultiArray, "/co2_sensor_3", 10),
            "co2_4": self.create_publisher(Float32MultiArray, "/co2_sensor_4", 10),
            "co2_5": self.create_publisher(Float32MultiArray, "/co2_sensor_5", 10),
            "co2_6": self.create_publisher(Float32MultiArray, "/co2_sensor_6", 10),
            "co2_7": self.create_publisher(Float32MultiArray, "/co2_sensor_7", 10),
            "co2_8": self.create_publisher(Float32MultiArray, "/co2_sensor_8", 10),
        }

        self.timer = self.create_timer(2, self.read_serial)
        self.timestamp = 0

        self.cli = self.create_client(DescribeParameters, "science_rpc")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ScienceRPC service not available, waiting again...")
        self.req = DescribeParameters.Request()

        self.msg = Float32MultiArray()

    def send_request(self, device, function, parameter):
        self.req.device = device
        self.req.function = function
        self.req.parameter = parameter
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def read_serial(self):

        for sens_dev, pub in self.co2_sensor_publishers.items():
            try:
                self.msg.data[0] = float(self.send_request(sens_dev, "read_co2", ""))
                self.msg.data[1] = float(self.send_request(sens_dev, "read_temp", ""))
                self.msg.data[2] = float(self.send_request(sens_dev, "read_humid", ""))
                pub.publish(self.msg)
            except TypeError as e:
                self.get_logger().warn(f"Invalid cmd response: ({e})")


def main(args=None):
    rclpy.init(args=args)

    arduino_manager = ArduinoManager()

    rclpy.spin(arduino_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
