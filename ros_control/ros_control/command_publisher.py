import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
from topics_services.msg import Telemetric
from topics_services.msg import ShimData
from topics_services.msg import ToEsp32
from topics_services.msg import ServoData
import serial
import threading


class CommandPublisherNode(Node):
    def __init__(self):
        super().__init__('command_publisher')
        # Initialize the serial port communication
        self.serial_port = serial.Serial(port='/dev/ttyUSB2', baudrate=115200)
        self.dc_data_changed = False
        self.servo_data_changed = False

        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()
        self.current_state_publisher = self.create_publisher(UInt8, '/current_state_topic', 30)
        self.kill_autonome_publisher = self.create_publisher(String, '/kill_command', 30)
        self.servo_publisher = self.create_publisher(ShimData, '/ShimData', 30)
        self.dc_publisher = self.create_publisher(ToEsp32, '/to_esp32', 30)
        self.get_servo_pos_subscriber = self.create_subscription(Telemetric, 'Telemetric', self.servos_pos_callback, 30)
        
        #defines commands
        self.state_command_id = 1
        self.servo_command_id = 2
        self.servo_reset_id = 6
        self.gripper_command_id = 7
        self.z_command_id = 8
        self.left_motor_id = 9
        self.right_motor_id = 10
        self.req_weight_comand_id = 11
    
        #defines the value of states
        self.not_active_state = 0
        self.non_autonomous_state_free = 1      # geen rubsband eronder dus servo's kunnen naar eindstop
        self.non_autonomous_state_rubs_free = 2 # rubsband erop en de hoogte is aanpasbaar
        self.non_autonomous_state_rubs = 3      # rubsband erop en de hoogte is vast op dezelfde hoogte als de rubsband
        self.autonomous_state = 4

        #define the init state
        self.current_state = self.not_active_state

        #defines servo values
        self.servo_max_pos = 240
        self.servo_min_pos = -130
        self.servo_min_pos_rubs = 50
        self.servo_null = 0
        self.servo_once = True
        self.desired_min_speed_servo = -200
        self.desired_max_speed_servo = 200
        self.current_servo_pos = Telemetric()

        #servo gripper
        self.gripper_min_speed = 0
        self.gripper_max_speed = 1425
        self.set_speed_gripper = 0

        #servo z-as
        self.z_min_speed = 0
        self.z_max_speed = 2047
        self.set_speed_z = 0

        #servo hoogte hoek en snelheid
        self.x_servo = 0
        self.y_servo = 0
        self.z_servo = 100
        self.speed_servo = 0

        #defines dc values
        self.desired_min_dc = -235
        self.desired_max_dc = 235
        self.dc_once_l = True
        self.dc_once_r = True
        self.speed_l = 0 
        self.speed_r = 0 
        self.state_t = 0
        self.state = 0
        self.matrix = 0

        self.servo_mode_normal = 0
        
    # def stop_reading_commands(self):
        # self.is_reading_commands = False
    
    def servos_pos_callback(self, msg):
        # Store the received telemetry data in the instance variable
        self.servo_pos_data = msg
        cur_pos_fr = msg.cur_pos_fr
        cur_pos_fl = msg.cur_pos_fl
        cur_pos_br = msg.cur_pos_br
        cur_pos_bl = msg.cur_pos_bl
        mean = (cur_pos_fr + cur_pos_fl + cur_pos_br + cur_pos_bl) / 4

        # self.get_logger().info(
        #     f"Received Telemetric data: "
        #     f"cur_pos_fr={cur_pos_fr}, "
        #     f"cur_pos_fl={cur_pos_fl}, "
        #     f"cur_pos_br={cur_pos_br}, "
        #     f"cur_pos_bl={cur_pos_bl}"
        # )
        self.get_logger().info(f"Mean: {mean}")
        if self.servo_once is False and mean == 0:
            self.servo_once = True
            self.set_servos_to_position(mean)

    def serial_read_loop(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                command = self.serial_port.readline().decode().strip()
                command_parts = command.split()

                if len(command_parts) < 2:
                    self.get_logger().info("Wrong amount of parts recieved")
                else:
                    command_id = int(command_parts[0])
                    value = int(command_parts[1])
                    self.publish_command(command_id, value)
                
                if self.dc_data_changed:
                    msg = ToEsp32()
                    msg.speed_l = self.speed_l
                    msg.speed_r = self.speed_r
                    self.dc_publisher.publish(msg)
                    self.dc_data_changed = False
                
                if self.servo_data_changed:
                    msg = ShimData()
                    msg.x = self.x_servo
                    msg.y = self.y_servo
                    msg.z = self.z_servo
                    msg.speed = self.speed_servo
                    msg.set_speed1 = self.set_speed_gripper
                    msg.set_speed2 = self.set_speed_z
                    self.servo_publisher.publish(msg)
                    self.servo_data_changed = False

    def publish_command(self, command_id, value):
        if self.current_state == 0 and command_id != 1:
            self.get_logger().info("Currently not active!")
            return

        #change states
        if command_id == self.state_command_id:
            msg = UInt8()
            if value == self.not_active_state:
                msg.data = self.not_active_state
                self.current_state_publisher.publish(msg)
                self.current_state = self.not_active_state
            elif value == self.non_autonomous_state_free:
                msg.data = self.non_autonomous_state_free
                self.current_state_publisher.publish(msg)
                self.current_state = self.non_autonomous_state_free
            elif value == self.non_autonomous_state_rubs:
                msg.data = self.non_autonomous_state_rubs
                self.current_state_publisher.publish(msg)
                self.current_state = self.non_autonomous_state_rubs
            elif value == self.non_autonomous_state_rubs_free:
                msg.data = self.non_autonomous_state_rubs_free
                self.current_state_publisher.publish(msg)
                self.current_state = self.non_autonomous_state_rubs_free
            elif value == self.autonomous_state:
                msg.data = self.autonomous_state
                self.current_state_publisher.publish(msg)
                self.current_state = self.autonomous_state

        #set_servo_position        
        elif command_id == self.servo_command_id:
            # self.get_logger().info("servo command recieved")
            speed = self.convert_range(value, self.desired_min_speed_servo, self.desired_max_speed_servo)

            msg = ShimData()
            if speed > 0:
                self.servo_once = False
                msg.z = self.servo_max_pos
                msg.speed = speed
                # self.get_logger().info("set sevro position with speed: " + str(msg.data))
                self.servo_publisher.publish(msg)

            elif speed < 0:
                self.servo_once = False
                if self.current_state == self.non_autonomous_state_free:
                    msg.z = self.servo_min_pos
                elif self.current_state == self.non_autonomous_state_rubs:
                    self.z_servo = self.rubs_height
                
                msg.speed = speed
                self.servo_publisher.publish(msg)

            elif speed == 0 and self.current_state != self.non_autonomous_state_rubs:
                if self.servo_once != True:
                    self.servo_once = True
                    mean = self.calculate_mean_servo_position()
                    msg.speed = 50
                    self.z_position = mean
                    self.servo_data_changed = True  
    
        #reset_servo_position
        elif command_id == self.servo_reset_id and self.current_state != self.non_autonomous_state_rubs or self.current_state != self.non_autonomous_state_rubs_free:
            self.z_position = 0
            self.servo_speed = 50
            self.servo_data_changed = True

        #set_gripper_speed
        elif command_id == self.gripper_command_id:
            # self.get_logger().info("servo command recieved 7")
            msg = ServoData()
            converted_speed = self.convert_range(value, self.gripper_min_speed, self.gripper_max_speed)
            if converted_speed < 1000 and converted_speed > 200:
                msg.set_speed2 = 200
                self.arm_publisher.publish(msg)
            elif converted_speed < 200:
                msg.set_speed2 = converted_speed
                self.arm_publisher.publish(msg)
            elif converted_speed > 1225 and converted_speed < 3000:
                msg.set_speed2 = 1225
                self.arm_publisher.publish(msg)
            elif converted_speed < 1225 and converted_speed > 1025:
                msg.set_speed2 = converted_speed
                self.arm_publisher.publish(msg)

        #set_z_speed
        elif command_id == self.z_command_id:
            # self.get_logger().info("servo command recieved 8")
            msg = ServoData()
            converted_speed = self.convert_range(value, self.z_min_speed, self.z_max_speed)
            msg.set_speed1 = converted_speed
            self.arm_publisher.publish

        #set_l_motor_speed
        elif command_id == self.left_motor_id:
            if value == 0 and self.dc_once_l != True:
                speed_l = 0
                self.dc_data_changed = True
                self.dc_once_l = True
            else:
                speed_l = self.convert_range(value, self.desired_min_dc, self.desired_max_dc)
                self.dc_once_l = False
            
            if speed_l > 0:
                self.speed_l = speed_l + 20
                self.dc_data_changed = True
            elif speed_l < 0: 
                self.speed_l = speed_l - 20
                self.dc_data_changed = True                

        #set_r_motor_speed
        elif command_id == self.right_motor_id:
            if value == 0 and self.dc_once_r != True:
                speed_r = 0
                self.dc_data_changed = True
                self.dc_once_r = True
            else:
                speed_r = self.convert_range(value, self.desired_min_dc, self.desired_max_dc)
                self.dc_once_r = False

            if speed_r > 0:
                self.speed_r = speed_r + 20
                self.dc_data_changed = True
            elif speed_r < 0:
                self.speed_r = speed_r - 20
                self.dc_data_changed = True
            
        else:
            self.get_logger().info("wrong")
        ### TODO IMPLEMENT GET/REQ WEIGHT###
        # elif command_id == self.req_weight_comand_id:
        #     msg = UInt8()
        #     msg = value

                
    def calculate_mean_servo_position(self):
    # Retrieve servo position data from the instance variable or any other source
        cur_pos_fr = self.servo_pos_data.cur_pos_fr
        cur_pos_fl = self.servo_pos_data.cur_pos_fl
        cur_pos_br = self.servo_pos_data.cur_pos_br
        cur_pos_bl = self.servo_pos_data.cur_pos_bl
        mean = (cur_pos_fr + cur_pos_fl + cur_pos_br + cur_pos_bl) / 4
        return mean
        


    def convert_range(self, value, desired_min, desired_max):
        # Define the original range (-127 to 128) 
        original_min = -127
        original_max = 128
        
        # Perform the range conversion using linear scaling and mapping
        original_range = original_max - original_min
        desired_range = desired_max - desired_min
        scaled_value = (value - original_min) * desired_range / original_range + desired_min

        # Round the scaled value to the nearest integer
        converted_value = int(round(scaled_value))

        return converted_value

def main(args=None):
    rclpy.init(args=args)
    controller_node = CommandPublisherNode()
    rclpy.spin(controller_node)
    # controller_node.stop_reading_commands()
    controller_node.serial_port.close()
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
