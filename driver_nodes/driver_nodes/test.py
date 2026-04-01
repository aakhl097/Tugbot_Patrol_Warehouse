#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')

        self.publisher_ = self.create_publisher(
            Float32,
            '/bhf',
            10
        )

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.value = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.value

        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: {msg.data}')

        self.value = 1.5


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
