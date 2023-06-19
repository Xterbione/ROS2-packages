import rclpy
from rclpy.node import Node
from topics_services.msg import ServoData
from topics_services.msg import ShimData
import math

center_right = 370
center_left = 650

maximum_right = 600
minimum_right = 240
maximum_left = 880
minimum_left = 520

class positioningshimNode(Node):
    def __init__(self):
        super().__init__('positioningshim')
        self.subscription = self.create_subscription(ShimData, '/ShimData', self.callback, 10)
        self.publisher = self.create_publisher(ServoData, '/ServoData', 10)

    def callback(self, msg):
        # Process the received message
        x_angle = msg.x #position driven
        y_angle = msg.y #position driven
        z_angle = msg.z #speed driven
        speed_z = msg.speed

        # Calculate x and y distances based on angles
        x_distance = math.sin(math.radians(x_angle)) * 230
        y_distance = math.sin(math.radians(y_angle)) * 250

        pos_wheel_rf = center_right + z_angle + y_distance - x_distance  # servo right front
        pos_wheel_lf = center_left - z_angle + y_distance + x_distance  # servo left front
        pos_wheel_rb = center_right - z_angle - y_distance - x_distance  # servo right back
        pos_wheel_lb = center_left + z_angle - y_distance + x_distance  # servo left back

        new_msg = ServoData()

        if pos_wheel_rf < maximum_right and pos_wheel_rf > minimum_right:
            new_msg.set_pos3 = pos_wheel_rf
        if pos_wheel_lf < maximum_right and pos_wheel_lf > minimum_right:
            new_msg.set_pos4 = pos_wheel_lf
        if pos_wheel_rb < maximum_left and pos_wheel_rb > minimum_left:
            new_msg.set_pos5 = pos_wheel_rb
        if pos_wheel_lb < maximum_left and pos_wheel_lb > minimum_left:
            new_msg.set_pos6 = pos_wheel_lb
        
        # new_msg.set_pos3 = 370 + z_angle + y_distance - x_distance  # servo right front
        # new_msg.set_pos4 = 650 - z_angle + y_distance + x_distance  # servo left front
        # new_msg.set_pos5 = 370 - z_angle - y_distance - x_distance  # servo right back
        # new_msg.set_pos6 = 650 + z_angle - y_distance + x_distance  # servo left back
        new_msg.set_speed3 = speed_z
        new_msg.set_speed4 = speed_z
        new_msg.set_speed5 = speed_z
        new_msg.set_speed6 = speed_z

        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    positioningshim = positioningshimNode()
    rclpy.spin(positioningshim)
    positioningshim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
