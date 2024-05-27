from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    querying_service = Node(
        package="py_srvcli",
        executable="service",
    )

    sensor_data_publisher = Node(
        package="sensors",
        executable="arduino_manager",
    )

    return LaunchDescription(
        [
            querying_service,
            sensor_data_publisher,
        ]
    )
