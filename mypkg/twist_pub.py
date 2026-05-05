#!/uer/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from rclpy import qos

import random

class TwistNode(Node):
        def __init__(self):
                super().__init__("twist_publisher_node")
                self.vel_pub = self.create_publisher(Twist, "cmd_val", 
                                                     qos_profile=qos.qos_profile_system_default)
                self.vel_timer = self.create_timer(0.2, self.vel_timer_callback)

                self.vel_cmd = Twist()
                print("twist is started")

        def vel_timer_callback(self):

                self.ramdom_vel()
                self.vel_pub.publish(self.vel_cmd)
                
        def ramdom_vel(self):
                linear_x = random.random()
                angular_z = random.random()

                print("linear_x  : ", linear_x)
                print("angular_z  : ", angular_z)
                print(" ")

                self.vel_cmd.linear.x = linear_x
                self.vel_cmd.angular.z = angular_z


def main():
        rclpy.init()

        node_vel = TwistNode()
        rclpy.spin(node_vel)

        rclpy.shutdown()

if __name__ == "__main__":
        main()
        
