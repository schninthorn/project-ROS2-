#include <micro_ros_arduino.h> 

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t encoder_pub;
std_msgs__msg__Int32 encoder_msg;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t  executor;
rcl_node_t node;
rcl_timer_t timer1;

#define ENCA 6
#define ENCB 7
volatile long pos = 0; 

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }else {
    pos--;
  }

}

void timer1_callback(rcl_timer_t *timer1, int64_t last_call_time) {

  encoder_msg.data = pos;
  rcl_publish(&encoder_pub, &encoder_msg, NULL);

}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  rclc_support_init(&support, 0, NULL, &allocator);
  
  rclc_node_init_default(&node, "encode_read_node", "", &support);

  rclc_publisher_init_default(
    &encoder_pub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_value"
  );

  rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(10),
    timer1_callback
  );

  rclc_executor_init(
    &executor,
    &support.context,
    1 ,
    &allocator
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer1);

}



void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(pos);
  //delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
