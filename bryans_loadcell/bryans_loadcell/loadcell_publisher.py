import time
import threading
import RPi.GPIO as GPIO

from hx711 import HX711
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LoadCellPublisher(Node):
    def __init__(self):
        super().__init__('loadcell_publisher')
        self.publisher_ = self.create_publisher(String, 'loadcell_data', 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.reference_unit = 682  # Set your reference unit here

        self.hx = HX711(dout=22, pd_sck=6)
        self.hx.set_reading_format("MSB", "MSB")
        self.hx.reset()
        self.hx.tare()
        self.get_logger().info('Tare done! Add weight now...')

    def publish_message(self):
        msg = String()
        try:
            val = self.hx.get_weight(5)
            weight = val / self.reference_unit
            msg.data = f"Weight: {weight} grams"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
        except KeyboardInterrupt:
            self.clean_and_exit()

    def clean_and_exit(self):
        self.get_logger().info("Cleaning...")
        self.hx.power_down()
        self.hx.power_up()
        GPIO.cleanup()
        self.get_logger().info("Bye!")
        sys.exit()


def main(args=None):
    rclpy.init(args=args)
    loadcell_publisher = LoadCellPublisher()
    rclpy.spin(loadcell_publisher)
    loadcell_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
