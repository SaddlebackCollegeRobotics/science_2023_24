from science_interfaces.srv import ScienceRPC

import rclpy
from rclpy.node import Node

from serial import Serial


class ScienceServer(Node):

    def __init__(self):

        super().__init__('science_server')

        self.srv = self.create_service(ScienceRPC, 'science_rpc', self.science_rpc_callback)

        # self.serial = Serial('/dev/ttyACM0', 9600, timeout=1)

    def science_rpc_callback(self, request, response):
        
        command = str(request.cmd_address) + ' ' + str(request.value)
        self.get_logger().info(f'Incoming request: {command}')
        # self.serial.write(command)

        return response


def main():
    rclpy.init()

    science_server = ScienceServer()

    try:
        rclpy.spin(science_server)
    except KeyboardInterrupt:
        print('Shutting down...')    

    rclpy.shutdown()


if __name__ == '__main__':
    main()