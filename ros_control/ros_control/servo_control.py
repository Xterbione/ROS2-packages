#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import ServoData
from topics_services.msg import Telemetric
from dynamixel_sdk import *
from .submodules.Ax12 import *

Ax12.DEVICENAME = '/dev/ttyUSB0'
Ax12.BAUDRATE = 1_000_000
# sets baudrate and opens com port
Ax12.connect()
# create AX12 instance with ID 10 
motor_id_1 = 11
motor_id_2 = 10
motor_id_3 = 1
motor_id_4 = 0
motor_id_5 = 3
motor_id_6 = 2
gripper = Ax12(motor_id_1)
z_axis = Ax12(motor_id_2)
wheelRF = Ax12(motor_id_3)
wheelLF = Ax12(motor_id_4)
wheelRB = Ax12(motor_id_5)
wheelLB = Ax12(motor_id_6)

max_torque1 = 0
max_torque2 = 0
max_torque3 = 0
max_torque4 = 0
max_torque5 = 0
max_torque6 = 0
set_speed1 = 0
set_speed2 = 0
set_speed3 = 0
set_speed4 = 0
set_speed5 = 0
set_speed6 = 0
set_position1 = 0
set_position2 = 0
set_position3 = 0
set_position4 = 0
set_position5 = 0
set_position6 = 0

position1 = 0
position2 = 0
position3 = 0
position4 = 0
position5 = 0
position6 = 0
temperature1 = 0
temperature2 = 0
temperature3 = 0
temperature4 = 0
temperature5 = 0
temperature6 = 0
voltage1 = 0
voltage2 = 0
voltage3 = 0
voltage4 = 0
voltage5 = 0
voltage6 = 0
load1 = 0
load2 = 0
load3 = 0
load4 = 0
load5 = 0
load6 = 0

class ServoControllerNode(Node):
    # Default settings for AX-12A servo
    PROTOCOL_VERSION = 1.0
    DEBUG = True

    def __init__(self):
        super().__init__("servo_controller")
        self.publisher_ = self.create_publisher(Telemetric, '/Telemetric', 60)
        self.subscription = self.create_subscription(ServoData, '/ServoData', self.callback, 60)
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux 
        self.timer_ = self.create_timer(60, self.update_telemetric_data)

    def publish_telemetric_data(self):
        msg = Telemetric()
        msg.cur_pos_fr = position3
        msg.cur_pos_fl = position4
        msg.cur_pos_br = position5
        msg.cur_pos_bl = position6
        msg.cur_pos_z_as = 0
        msg.cur_pos_gripper = 0
        msg.temp_fr = float(temperature3)
        msg.temp_fl = float(temperature4)
        msg.temp_br = float(temperature5)
        msg.temp_bl = float(temperature6)
        msg.temp_z_as = float(temperature2)
        msg.temp_gripper = float(temperature1)
        msg.voltage_fr = voltage3
        msg.voltage_fl = voltage4
        msg.voltage_br = voltage5
        msg.voltage_bl = voltage6
        msg.voltage_z_as = voltage1
        msg.voltage_gripper = voltage2
        msg.load_fr = load3
        msg.load_fl = load4
        msg.load_br = load5
        msg.load_bl = load6
        msg.load_z_as = load1
        msg.load_gripper = load2
        self.publisher_.publish(msg)

    def callback(self, msg):
        # Set servo position based on received data
        set_position3 = msg.set_pos3
        set_position4 = msg.set_pos4
        set_position5 = msg.set_pos5
        set_position6 = msg.set_pos6
        set_speed1 = msg.set_speed1
        set_speed2 = msg.set_speed2
        set_speed3 = msg.set_speed3
        set_speed4 = msg.set_speed4
        set_speed5 = msg.set_speed5
        set_speed6 = msg.set_speed6

        gripper.set_moving_speed(set_speed1)
        z_axis.set_moving_speed(set_speed2)

        wheelRF.set_moving_speed(set_speed3)
        wheelRF.set_goal_position(set_position3)

        wheelLF.set_moving_speed(set_speed4)
        wheelLF.set_goal_position(set_position4)

        wheelRB.set_moving_speed(set_speed5)
        wheelRB.set_goal_position(set_position5)

        wheelLB.set_moving_speed(set_speed6)
        wheelLB.set_goal_position(set_position6)

    def update_telemetric_data(self):
        # Get servo data and publish
        position3 = wheelRF.get_present_position()
        position4 = wheelLF.get_present_position()
        position5 = wheelRB.get_present_position()
        position6 = wheelLB.get_present_position()
        temperature1 = z_axis.get_temperature()
        temperature2 = gripper.get_temperature()
        temperature3 = wheelRF.get_temperature()
        temperature4 = wheelLF.get_temperature()
        temperature5 = wheelRB.get_temperature()
        temperature6 = wheelLB.get_temperature()
        voltage1 = z_axis.get_voltage()
        voltage2 = gripper.get_voltage()
        voltage3 = wheelRF.get_voltage()
        voltage4 = wheelLF.get_voltage()
        voltage5 = wheelRB.get_voltage()
        voltage6 = wheelLB.get_voltage()
        load1 = z_axis.get_load()
        load1 = gripper.get_load()
        load1 = wheelRF.get_load()
        load1 = wheelLF.get_load()
        load1 = wheelRB.get_load()
        load1 = wheelLB.get_load()
        self.publish_telemetric_data()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoControllerNode()
    rclpy.spin(servo_controller)
    servo_controller.port_handler.closePort()
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()