import rclpy
import serial
from rclpy.node import Node
import json

from std_msgs.msg import Float32MultiArray

ser = serial.Serial('/dev/ttyUSB0')


class AdafruitSensorPublisher(Node):

    def __init__(self):
        super().__init__('adafruit_sensor')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/scd30_sensor_1', 10)
        self.timer = self.create_timer(2, self.publish_sensor_data)

    def publish_sensor_data(self):
        msg = Float32MultiArray()
        data = ser.readline()
        try:
            js = json.loads(data.decode())
            msg.data.append(js['temperature'])
            msg.data.append(js['relative_humidity'])
            msg.data.append(js['co2'])
            self.publisher_.publish(msg)
        except:
            print('Could not parse json.')


def main(args=None):
    rclpy.init(args=args)

    adafruit_sensor_publisher = AdafruitSensorPublisher()

    rclpy.spin(adafruit_sensor_publisher)

    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
