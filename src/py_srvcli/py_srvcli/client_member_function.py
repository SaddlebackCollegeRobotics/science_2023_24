from science_interfaces.srv import ScienceRPC
import rclpy
from rclpy.node import Node


class ScienceClient(Node):

    def __init__(self):
        
        super().__init__('science_client')
        
        self.client = self.create_client(ScienceRPC, 'science_rpc')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Science RPC service not available, waiting again...')
        
        self.request = ScienceRPC.Request()

    def send_request(self, cmd_address, value):

        self.request.cmd_address = cmd_address
        self.request.value = value

        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.rpc_response_callback)
    
    def rpc_response_callback(self, future):

        try:
            response = future.result()
            self.get_logger().info(f'Result: {str(response.return_value)}')
        except Exception as e:
            self.get_logger().warning(f'Error: Service call failed!')

    def print_menu(self):

        science_menu = "\nScience Server Menu:\n1. Move Drill Platform\n2. Set Drill Speed (-1.0, 1.0)\n3. Move Science Platform\n4. Set Scoop (-1, 1)\n5. Exit\n"

        print(science_menu)
        print('Enter <cmd> <val>: ', end='')


def main():
    rclpy.init()

    science_client = ScienceClient()

    while True:

        try:
            science_client.print_menu()
            command = input()

            arg_list = command.split(' ')

            cmd_address = int(arg_list[0])

            if cmd_address == 5:
                break

            if cmd_address < 1 or cmd_address > 5:
                print('Invalid command')
                continue

            value = float(arg_list[1])

            science_client.send_request(cmd_address, value)
        
        except Exception as e:
            print('Invalid input')
            continue


    science_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()