from zlib import crc32
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node

from serial import Serial


class ScienceServer(Node):

    def __init__(self):

        super().__init__("science_server")

        self.srv = self.create_service(
            DescribeParameters, "science_rpc", self.science_rpc_callback
        )

        self._arduino_serial = Serial("/dev/ttyACM0", 9600, timeout=1)

    def __del__(self):
        self._arduino_serial.close()

    def science_rpc_callback(self, request, response):
        command_str = ",".join(request.names) + ","
        command_bytes = bytes(command_str, encoding="utf-8")
        checksum = hex(crc32(command_bytes))
        self.get_logger().info(
            f'Incoming request: "{command_str}" (checksum = {checksum})'
        )

        final_cmd_str = command_str + str(checksum) + "\n"
        final_cmd_bytes = bytes(final_cmd_str, encoding="utf-8")

        self.get_logger().info(f'Sending data: "{final_cmd_str}"')

        self._arduino_serial.write(final_cmd_bytes)

        ret_data = ret_data = str(self._arduino_serial.readline()).split(",")

        if len(ret_data) < 4:
            response.descriptors = [ParameterDescriptor(name="INVALID")]
            return response

        # First field has leading "b'"
        # Forth field has a trailing "\\n'"
        ret_data[0] = ret_data[0].removeprefix("b'")
        ret_data[3] = ret_data[3].removesuffix("\\n'")

        self.get_logger().info(f'Received data: "{ret_data}"')

        # Validate the checksum
        recv_checksum = int(ret_data[3], base=16)
        recv_calc_checksum = crc32(
            bytes(",".join(ret_data[:3]) + ",", encoding="utf-8")
        )

        if recv_calc_checksum != recv_checksum:
            response.descriptors = [ParameterDescriptor(name="BAD CHECKSUM")]
            return response

        response.descriptors = [ParameterDescriptor(name=ret_data[2])]
        return response


def main():
    rclpy.init()

    science_server = ScienceServer()

    try:
        rclpy.spin(science_server)
    except KeyboardInterrupt:
        print("Shutting down...")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
