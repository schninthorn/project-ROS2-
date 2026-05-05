import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

import sys

class turtleClient(Node):
    def __init__(self):
            super().__init__("turtle_service_Client_node")
            self.client = self.create_client(SetBool, 'star_stop_service')

            while not self.client.wait_for_service(timeout_sec = 1.0):
                  print("...waiting for service...")

            self.requst = SetBool.Request()

    def requst_to_server(self):
            inp = sys.argv[1]

            if(inp == 'true') or (inp == '1'):
                self.requst.data = True
            elif(inp == 'False') or (inp == '0'):
                self.requst.data = False

            self.future = self.client.call_async(self.requst)

def main():
    rclpy.init()

    tc = turtleClient()
    tc.requst_to_server()


    while rclpy.ok():
        rclpy.spin_once(tc)
        if tc.future.done():
            try:
                  response = tc.future.result()
            except Exception as e :
                    print(e)
            else:
                    print('response : ',response.message)
            break
    
    ##rclpy.spin(tc)
    rclpy.shutdown

if __name__ == "__main__":
      main()


        
