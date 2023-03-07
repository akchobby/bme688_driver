import rclpy
from .gas_sensor import GasSensor


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    sensor_node = GasSensor()
    executor.add_node(sensor_node)
    executor.spin()
    executor.shutdown()

if __name__ == "__main__":
    main()