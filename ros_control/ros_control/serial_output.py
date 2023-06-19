import rclpy
from rclpy.node import Node
from topics_services.msg import ToEsp32
from topics_services.msg import Esp32Data
import serial

prev_speed_l = 0
prev_speed_r = 0
prev_speed_t = 0
prev_state = 0
prev_matrix = 0

class serial_outputNode(Node):
    def __init__(self):
        super().__init__('serial_output')
        # Initialize the serial port communication
        self.serial_port = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

        # Make subscriber and publisher
        self.subscription = self.create_subscription(ToEsp32, '/to_esp32', self.callback, 20)
        self.publisher = self.create_publisher(Esp32Data, '/esp32_data', 20)
        
        self.timer = self.create_timer(10, self.read_esp32)

        self.get_logger().info('init serial output complete!')

    def callback(self, msg):
        self.get_logger().info('recieved something!')
        global prev_speed_l, prev_speed_r, prev_speed_t, prev_state, prev_matrix

        set_speed_l = msg.speed_l
        set_speed_r = msg.speed_r
        set_speed_t = msg.speed_t
        set_state = msg.state
        set_matrix = msg.matrix

        if set_speed_l != prev_speed_l or set_speed_r != prev_speed_r or set_speed_t != prev_speed_t:
            self.set_motor_values(set_speed_l, set_speed_r, set_speed_t)

        if set_state != prev_state:
            self.give_state(set_state)

        if set_matrix != prev_matrix:
            self.set_matrix(set_matrix)

        prev_speed_l = set_speed_l
        prev_speed_r = set_speed_r
        prev_speed_t = set_speed_t
        prev_state = set_state
        prev_matrix = set_matrix

    def send_msg(self, command):
        # Write the command to the serial port
        self.serial_port.write(command.encode())

    def read_esp32(self):
        self.get_logger().info('reading sensors!')
        self.send_msg("f\r")

        response = self.serial_port.readline().decode().strip()
        self.get_logger().info('got sensor response!')
        tokens = response.split()

        if len(tokens) < 4 or tokens[0] != "f":
            self.get_logger().error('Error: Invalid response for frequencies')
            token_f = False
        else:
            token_f = True
        
        if token_f:
            freq_l = int(tokens[1])
            freq_m = int(tokens[2])
            freq_h = int(tokens[3])

        self.get_logger().info('freq_l: {}'.format(freq_l))
        self.get_logger().info('freq_m: {}'.format(freq_m))
        self.get_logger().info('freq_h: {}'.format(freq_h))

        self.send_msg("k\r")

        response = self.serial_port.readline().decode().strip()
        tokens = response.split()

        if len(tokens) < 3 or tokens[0] != "k":
            self.get_logger().error('Error: Invalid response for current and voltage')
            token_k = False
        else:
            token_k = True

        if token_k:
            voltage = float(tokens[1])
            current = float(tokens[2])

        self.get_logger().info('voltage: {}'.format(voltage))
        self.get_logger().info('current: {}'.format(current))

        msg = Esp32Data()
        if token_f:
            msg.frequency_l = freq_l
            msg.frequency_m = freq_m
            msg.frequency_h = freq_h
        
        if token_k:
            msg.voltage = voltage
            msg.current = current
        
        if token_k or token_f:
            self.publisher.publish(msg)
            self.get_logger().info('published data')

    def set_motor_values(self, motor_l, motor_r, track):
        command = f"m {motor_l} {motor_r} {track}\r"
        self.get_logger().info('send motor command: {}'.format(command))
        self.send_msg(command)

    def set_matrix(self, matrix):
        command = f"x {matrix}\r"
        self.get_logger().info('send matrix command: {}'.format(command))
        self.send_msg(command)

    def give_state(self, state):
        command = f"s {state}\r"
        self.get_logger().info('send state command: {}'.format(command))
        self.send_msg(command)

def main(args=None):
    rclpy.init(args=args)
    serial_output_node = serial_outputNode()
    rclpy.spin(serial_output_node)
    serial_output_node.serial_port.close()
    serial_output_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()