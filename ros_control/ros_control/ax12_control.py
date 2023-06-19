#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import ServoData
from topics_services.msg import Telemetric
from pyax12.connection import Connection

gripper = 11
z_as = 10
servo_rf = 1
servo_lf = 0
servo_rb = 3
servo_lb = 2

set_speed_gripper = 0
set_speed_z_as = 0
set_speed_rf = 0
set_speed_lf = 0
set_speed_rb = 0
set_speed_lb = 0
set_position_rf = 0
set_position_lf = 0
set_position_rb = 0
set_position_lb = 0

position_rf = 0
position_lf = 0
position_rb = 0
position_lb = 0
temperature_gripper = 0
temperature_z_as = 0
temperature_rf = 0
temperature_lf = 0
temperature_rb = 0
temperature_lb = 0
voltage_gripper = 0
voltage_z_as = 0
voltage_rf = 0
voltage_lf = 0
voltage_rb = 0
voltage_lb = 0
load_gripper = 0
load_z_as = 0
load_rf = 0
load_lf = 0
load_rb = 0
load_lb = 0

delay = 0

class Ax12ControlNode(Node):
    def __init__(self):
        super().__init__("Ax12_control")
        self.publisher = self.create_publisher(Telemetric, '/Telemetric', 20)
        self.subscription = self.create_subscription(ServoData, '/ServoData', self.callback, 20)
        self.timer_ = self.create_timer(0.5, self.publish_telemetric_data)

        self.serial_connection = Connection(port="/dev/ttyAMA0", baudrate=1000000, rpi_gpio=True)

        delay = self.serial_connection.get_return_delay_time(gripper)
        self.get_logger().info('found gripper. delay: {}'.format(delay))
        
        delay = self.serial_connection.get_return_delay_time(z_as)
        self.get_logger().info('found z-as. delay: {}'.format(delay))

        delay = self.serial_connection.get_return_delay_time(servo_rf)
        self.get_logger().info('found servo_rf. delay: {}'.format(delay))

        delay = self.serial_connection.get_return_delay_time(servo_lf)
        self.get_logger().info('found servo_lf. delay: {}'.format(delay))

        delay = self.serial_connection.get_return_delay_time(servo_rb)
        self.get_logger().info('found servo_rb. delay: {}'.format(delay))

        delay = self.serial_connection.get_return_delay_time(servo_lb)
        self.get_logger().info('found servo_lb. delay: {}'.format(delay))

        self.get_logger().info("Ax12_control has started")

    def callback(self, msg):
        # Set servo position based on received data
        self.get_logger().info('Received position: {}'.format(msg.cur_pos_fr))
        self.serial_connection.goto(servo_rf, position_rf, set_speed_rf, degrees= False)
        self.serial_connection.goto(servo_lf, position_lf, set_speed_lf, degrees= False)
        self.serial_connection.goto(servo_rb, position_rb, set_speed_rb, degrees= False)
        self.serial_connection.goto(servo_lb, position_lb, set_speed_lb, degrees= False)

        self.serial_connection.set_speed(gripper, set_speed_gripper)
        self.serial_connection.set_speed(z_as, set_speed_z_as)
        
    def publish_telemetric_data(self):
        load_gripper = self.serial_connection.get_present_load(gripper)
        load_z_as = self.serial_connection.get_present_load(z_as)
        load_rf = self.serial_connection.get_present_load(servo_rf)
        load_lf = self.serial_connection.get_present_load(servo_lf)
        load_rb = self.serial_connection.get_present_load(servo_rb)
        load_lb = self.serial_connection.get_present_load(servo_lb)

        position_rf = self.serial_connection.get_present_position(servo_rf, degrees= False)
        position_rf = self.serial_connection.get_present_position(servo_lf, degrees= False)
        position_rf = self.serial_connection.get_present_position(servo_rb, degrees= False)
        position_rf = self.serial_connection.get_present_position(servo_lb, degrees= False)

        temperature_gripper = self.serial_connection.get_present_temperature(gripper)
        temperature_z_as = self.serial_connection.get_present_temperature(z_as)
        temperature_rf = self.serial_connection.get_present_temperature(servo_rf)
        temperature_lf = self.serial_connection.get_present_temperature(servo_lf)
        temperature_rb = self.serial_connection.get_present_temperature(servo_rb)
        temperature_lb = self.serial_connection.get_present_temperature(servo_lb)

        voltage_gripper = self.serial_connection.get_present_voltage(gripper)
        voltage_gripper = self.serial_connection.get_present_voltage(z_as)
        voltage_gripper = self.serial_connection.get_present_voltage(servo_rf)
        voltage_gripper = self.serial_connection.get_present_voltage(servo_lf)
        voltage_gripper = self.serial_connection.get_present_voltage(servo_rb)
        voltage_gripper = self.serial_connection.get_present_voltage(servo_lb)

        msg = Telemetric()
        msg.cur_pos_fr = position_rf
        msg.cur_pos_fl = position_lf
        msg.cur_pos_br = position_rb
        msg.cur_pos_bl = position_lb
        msg.temp_fr = temperature_rf
        msg.temp_fl = temperature_lf
        msg.temp_br = temperature_rb
        msg.temp_bl = temperature_lb
        msg.temp_z_as = temperature_z_as
        msg.temp_gripper = temperature_gripper
        msg.voltage_fr = voltage_rf
        msg.voltage_fl = voltage_lf
        msg.voltage_br = voltage_rb
        msg.voltage_bl = voltage_lb
        msg.voltage_z_as = voltage_z_as
        msg.voltage_gripper = voltage_gripper
        msg.load_fr = load_rf
        msg.load_fl = load_lf
        msg.load_br = load_rb
        msg.load_bl = load_lb
        msg.load_z_as = load_z_as
        msg.load_gripper = load_gripper
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    Ax12Control = Ax12ControlNode()
    rclpy.spin(Ax12Control)
    Ax12Control.serial_connection.close()
    Ax12Control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()



