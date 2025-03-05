#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_service.srv import IdSw  # Example service type
from geometry_msgs.msg import Twist
import time

class MyClientNode(Node):
    def __init__(self):
        super().__init__('my_client_node')
        self.client = self.create_client(IdSw, 'process_voice')  # Service name
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = IdSw.Request()  # Create a service request

    def send_request(self):
        self.request.start = True
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Sending request to start recording voice commands!')
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = MyClientNode()

    response = client_node.send_request()
    if response is not None:
        client_node.get_logger().info(f'Result:{response.data}')
        words = response.data.split() 
        print("words", words)
        if words[1] == "forward" or words[1] == "forward." :
            print("going forward")
            twist_msg = Twist()

            twist_msg.linear.x = 2.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            time.sleep(4)
            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
        elif words[1] == "backward" or words[1] == "backward.":
            print("going backward")
            twist_msg = Twist()

            twist_msg.linear.x = -2.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            time.sleep(4)
            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            print("went backward")

        elif words[1] == "left" or words[1] == "left.":
            print("turning left")
            twist_msg = Twist()

            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.5

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            time.sleep(4)
            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            print("turned left")


        elif words[1] == "right." or words[1] == "right":
            print("turning right")
            twist_msg = Twist()

            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = -0.5

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            time.sleep(4)
            twist_msg.linear.x = 0.0  # Move forward at 0.5 m/s
            twist_msg.linear.y = 0.0  
            twist_msg.linear.z = 0.0  
            twist_msg.angular.x = 0.0  
            twist_msg.angular.y = 0.0 
            twist_msg.angular.z = 0.0

            # Publish the Twist message
            client_node.publisher_.publish(twist_msg)
            print("turned right")



    else:
        client_node.get_logger().error('Service call failed')

    # Shutdown the node
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
