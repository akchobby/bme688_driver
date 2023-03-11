#!/usr/bin/env python3

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gas_sensor import GasSensor


def main():

    # Read in parameter file
    file_path = "/".join([get_package_share_directory("bme688_driver"),"sensor_config.yaml"])
    with open(file_path, 'r') as stream:
        sensor_config = yaml.full_load(stream)

    # ROS init
    rclpy.init()

    # Nodes and executors
    executor = rclpy.executors.SingleThreadedExecutor()
    sensor_node = GasSensor(kwargs=sensor_config)
    executor.add_node(sensor_node)
    executor.spin()
    executor.shutdown()

if __name__ == '__main__':
    main()
