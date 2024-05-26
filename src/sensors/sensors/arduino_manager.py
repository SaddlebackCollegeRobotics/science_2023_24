from threading import Thread
from time import sleep
from std_msgs.msg import Float32MultiArray, UInt16
from rcl_interfaces.srv import DescribeParameters

import rclpy
from rclpy.node import Node, Publisher


class ArduinoManager(Node):
    timestamp: int
    publishers: dict[str, Publisher]

    def __init__(self):
        super().__init__("arduino_manager")

        # CO2, temperature, humidity
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
        self.tof_sensor_publishers = {
            "tof_1": self.create_publisher(UInt16, "/tof_sensor_1", 10),
            "tof_2": self.create_publisher(UInt16, "/tof_sensor_2", 10),
        }

        self.cli = self.create_client(DescribeParameters, "science_rpc")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ScienceRPC service not available, waiting again...")

        self.co2_msg = Float32MultiArray()
        self.tof_msg = UInt16()

    def send_co2_data_requests(self):
        while True:
            for sens_dev, pub in self.co2_sensor_publishers.items():
                self.co2_msg.data = [0.0] * 3
                try:
                    self.co2_msg.data[0] = float(
                        self.cli.call(
                            DescribeParameters.Request(
                                names=[sens_dev, "read_co2", ""])
                        )
                        .descriptors[0]
                        .name
                    )
                    self.co2_msg.data[1] = float(
                        self.cli.call(
                            DescribeParameters.Request(
                                names=[sens_dev, "read_temp", ""]
                            )
                        )
                        .descriptors[0]
                        .name
                    )
                    self.co2_msg.data[2] = float(
                        self.cli.call(
                            DescribeParameters.Request(
                                names=[sens_dev, "read_humid", ""]
                            )
                        )
                        .descriptors[0]
                        .name
                    )
                    pub.publish(self.co2_msg)
                except TypeError as e:
                    self.get_logger().warn(f"Invalid cmd response ({e})")
                    break
                except Exception as e:
                    self.get_logger().warn(
                        f"Unhandled exception type when getting co2 data! ({e})"
                    )
            sleep(2)

    def send_tof_data_requests(self):
        while True:
            for sens_dev, pub in self.tof_sensor_publishers.items():
                self.tof_msg.data = 0
                try:
                    self.tof_msg.data = int(
                        self.cli.call(
                            DescribeParameters.Request(
                                names=[sens_dev, "read", ""])
                        )
                        .descriptors[0]
                        .name
                    )
                    pub.publish(self.tof_msg)
                except TypeError as e:
                    self.get_logger().warn(f"Invalid cmd response ({e})")
                    break
                except Exception as e:
                    self.get_logger().warn(
                        f"Unhandled exception type when getting tof data! ({e})"
                    )
            sleep(0.25)


def main(args=None):
    rclpy.init(args=args)

    arduino_manager = ArduinoManager()

    spin_thread = Thread(target=rclpy.spin, args=(arduino_manager,))
    spin_thread.start()

    try:
        co2_req_thread = Thread(target=arduino_manager.send_co2_data_requests)
        co2_req_thread.start()
        arduino_manager.send_tof_data_requests()
    except KeyboardInterrupt:
        print("Exiting...")

    arduino_manager.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
