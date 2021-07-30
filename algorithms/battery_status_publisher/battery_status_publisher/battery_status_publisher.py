import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import BatteryState
import numpy as np
import random

class BatteryStatusPublisher(Node):
    def __init__(self):
        super().__init__('battery_status_publisher')

        self.battery_state_pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self.battery_state = BatteryState()
        self.battery_state.voltage = 100.0
        self.charge_loss = 0.1
        self.dt = 0.1
        self.battery_timer = self.create_timer(
            self.dt,
            self.publish_voltage
        )
    
    def discharge_battery(self):
        self.battery_state.voltage -= self.charge_loss

    def publish_voltage(self):
        self.discharge_battery()
        print(self.battery_state.voltage)
        self.battery_state_pub.publish(self.battery_state)
    

def main():
    rclpy.init()
    battery_status_publisher = BatteryStatusPublisher()
    try:
        rclpy.spin(battery_status_publisher)
    except KeyboardInterrupt:
        rclpy.logerr("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
