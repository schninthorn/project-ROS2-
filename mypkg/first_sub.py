import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

from rclpy import qos

class numderSub(Node):
        def __init__(self):
                super().__init__("number_subscription_node")

                self.numder_sub = self.create_subscription(Int64, "number_topic", self.numder_callback
                                                           ,qos_profile=qos.qos_profile_sensor_data)
                
                self.numder_sub

        def numder_callback(self,msg_in):
                new_msg = msg_in.data*2
                
                print("recieved msg : ", msg_in.data)
                #print(msg_in.data)
                print("converted msg : ", new_msg)
                print(" ")
                #print(new_msg)


def main():
        rclpy.init() 

        ns = numderSub()
        rclpy.spin(ns)

        rclpy.shutdown()

if __name__ == "__main__":
        main()


