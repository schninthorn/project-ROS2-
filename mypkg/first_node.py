#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy import qos
import random


class NumberNode(Node):

    def __init__(self):
        super().__init__("number_publisher_node")

        self.num_pub = self.create_publisher(
            Int64,
            "number_topic",
            qos.qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

    def timer_callback(self):
        random_number = random.randint(1, 100)

        msg = Int64()
        msg.data = random_number

        self.num_pub.publish(msg)
        print("random_number : ",random_number)
        print("")
        #self.get_logger().info(f"Publishing: {random_number}")


def main():
    rclpy.init()
    node = NumberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
      
                                              
