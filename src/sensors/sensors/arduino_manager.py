import rclpy
import serial
from rclpy.node import Node, Publisher
import json

from std_msgs.msg import Float32MultiArray

ser = serial.Serial('/dev/ttyUSB0')


class ArduinoManager(Node):
    timestamp: int
    publishers: dict[str, Publisher]

    def __init__(self):
        super().__init__('arduino_manager')

        self.co2_sensor_publishers = {
            'scd_1': self.create_publisher(Float32MultiArray, '/co2_sensor_1', 10),
            'scd_2': self.create_publisher(Float32MultiArray, '/co2_sensor_2', 10),
            'scd_3': self.create_publisher(Float32MultiArray, '/co2_sensor_3', 10),
            'scd_4': self.create_publisher(Float32MultiArray, '/co2_sensor_4', 10),
            'scd_5': self.create_publisher(Float32MultiArray, '/co2_sensor_5', 10),
            'scd_6': self.create_publisher(Float32MultiArray, '/co2_sensor_6', 10),
            'scd_7': self.create_publisher(Float32MultiArray, '/co2_sensor_7', 10),
            'scd_8': self.create_publisher(Float32MultiArray, '/co2_sensor_8', 10),
        }

        self.timer = self.create_timer(2, self.read_serial)
        self.timestamp = 0

    def read_serial(self):
        data = ser.readline()

        address, message = data.decode().split('|')

        if (address in self.co2_sensor_publishers):
            msg = Float32MultiArray()
            self.timestamp += 2

            try:
                temperature, humidity, co2 = message.split(',')

                msg.data.append(temperature)
                msg.data.append(humidity)
                msg.data.append(co2)
                msg.data.append(self.timestamp)

                self.co2_sensor_publishers[address].publish(msg)
            except:
                print('Could not parse json.')


def main(args=None):
    rclpy.init(args=args)

    arduino_manager = ArduinoManager()

    rclpy.spin(arduino_manager)

    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
