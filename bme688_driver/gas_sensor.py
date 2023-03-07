import bme680
import time
import rclpy
from rclpy.node import Node
from gas_sensor_msgs.msg import Data

class GasSensor(Node):

    def __init__(self, bus_id=5):

        super().__init__('bme688_publisher')
        self.publisher_ = self.create_publisher(Data, '/bme688/data', 10)

        try:
            self.sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY, i2c_bus=bus_id)
            self.sensor.set_humidity_oversample(bme680.OS_2X)
            self.sensor.set_pressure_oversample(bme680.OS_4X)
            self.sensor.set_temperature_oversample(bme680.OS_8X)
            self.sensor.set_filter(bme680.FILTER_SIZE_3)

            for name in dir(self.sensor.data):
                value = getattr(self.sensor.data, name)
                if not name.startswith('_'):
                    self.get_logger().info(f"Name: {name}, Value: {value}")
            
            # these params can be tweaked for your use case
            self.sensor.set_gas_heater_temperature(320)
            self.sensor.set_gas_heater_duration(150)
            self.sensor.select_gas_heater_profile(0)

        except Exception as e:

            self.get_logger().error(f"Sensor not initialized due to :  {e}")
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)
    
    def publisher_callback(self):
        out_msg = Data()
        out_msg.header.stamp = self.get_clock().now().to_msg()

        if self.sensor.get_sensor_data():
            #'{0:.2f} C,{1:.2f} hPa,{2:.2f} %RH'
            out_msg.stph.temperature = self.sensor.data.temperature
            out_msg.stph.pressure = self.sensor.data.pressure
            out_msg.stph.humidity = self.sensor.data.humidity

            if self.sensor.data.heat_stable:
                #{1} Ohms 
                out_msg.gas.stability = Data.gas.STABLE
                out_msg.gas.resistance = self.sensor.data.gas_resistance
            else:
                out_msg.gas.stability = Data.gas.UNSTABLE

            self.publisher_.publish(out_msg)
        
    def stamp_to_float(self, stamp):
        return stamp.sec + (stamp.nanosec * 1e-9)