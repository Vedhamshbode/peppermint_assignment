#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelForwarder(Node):
    def __init__(self):
        super().__init__('cmd_vel_forwarder')
        
        # Create the publisher for /diff_cont/cmd_vel_unstamped
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        # Create the subscriber for /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # self.get_logger().info('Received Twist message, forwarding to /diff_cont/cmd_vel_unstamped.')
        
        # Publish the received message to the /diff_cont/cmd_vel_unstamped topic
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_forwarder = CmdVelForwarder()

    rclpy.spin(cmd_vel_forwarder)

    cmd_vel_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
