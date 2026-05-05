#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>


// ===== Encoder =====
#define ENCA_l 5
#define ENCB_l 6
#define ENCA_r 11
#define ENCB_r 10
volatile long pos_l = 0; 
volatile long pos_r = 0;  

long pulseLeft  = 0;
long pulseRight = 0;
long oldLeft    = -999;
long oldRight   = -999;

// ===== Motor ซ้าย =====
#define l_dir1_pin 2
#define l_dir2_pin 3
#define l_pwm_pin  4

// ===== Motor ขวา =====
#define r_dir1_pin 7
#define r_dir2_pin 8
#define r_pwm_pin  9

// ===== ตัวแปร Motor =====
float factor_left  = 0.0;
float factor_right = 0.0;
int dir_left  = 0;
int dir_right = 0;

// ===== micro-ROS =====
rcl_publisher_t   enc_left_pub;
rcl_publisher_t   enc_right_pub;
rcl_subscription_t left_sub;
rcl_subscription_t right_sub;

std_msgs__msg__Int32   enc_left_msg;
std_msgs__msg__Int32   enc_right_msg;
std_msgs__msg__Float32 left_msg;
std_msgs__msg__Float32 right_msg;

rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t executor;
rcl_node_t      node;
rcl_timer_t     timer1;

void readEncoder_l(){
  int b_l = digitalRead(ENCB_l);
  if(b_l > 0){
    pos_l++;
  }else {
    pos_l--;
  }
}

void readEncoder_r(){
  int b_r = digitalRead(ENCB_r);
  if(b_r > 0){
    pos_r++;
  }else {
    pos_r--;
  }
}

// ===== Motor Control =====
void drive_left(int direction, int pwm) {
  if (direction > 0) {
    digitalWrite(l_dir1_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, pwm);
  } 
  else if (direction < 0) {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, HIGH);
    analogWrite(l_pwm_pin, pwm);
  } 
  else {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, 0);
  }
}

void drive_right(int direction, int pwm) {
  if (direction > 0) {
    digitalWrite(r_dir1_pin, HIGH);
    digitalWrite(r_dir2_pin, LOW);
    analogWrite(r_pwm_pin, pwm);
  } 
  else if (direction < 0) {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, HIGH);
    analogWrite(r_pwm_pin, pwm);
  } 
  else {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, LOW);
    analogWrite(r_pwm_pin, 0);
  }
}


void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_node", "", &support);

  rclc_publisher_init_default(&enc_left_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder_left");
  rclc_publisher_init_default(&enc_right_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder_right");

  rclc_subscription_init_default(&left_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "power_left");
  rclc_subscription_init_default(&right_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "power_right");

  rclc_timer_init_default(&timer1, &support,
    RCL_MS_TO_NS(100), timer1_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer1);
  rclc_executor_add_subscription(&executor, &left_sub,  &left_msg,  &left_callback,  ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &right_sub, &right_msg, &right_callback, ON_NEW_DATA);
}

void destroy_entities() {
  rcl_publisher_fini(&enc_left_pub,  &node);
  rcl_publisher_fini(&enc_right_pub, &node);
  rcl_subscription_fini(&left_sub,  &node);
  rcl_subscription_fini(&right_sub, &node);
  rcl_timer_fini(&timer1);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}


// ===== Callback: รับคำสั่ง Motor ซ้าย =====
void left_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  factor_left = msg->data;

  if (factor_left > 0)      dir_left = 1;
  else if (factor_left < 0) dir_left = -1;
  else                      dir_left = 0;
}

// ===== Callback: รับคำสั่ง Motor ขวา =====
void right_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  factor_right = msg->data;

  if (factor_right > 0)      dir_right = 1;
  else if (factor_right < 0) dir_right = -1;
  else                       dir_right = 0;
}

// ===== Callback: Publish Encoder ทุก 100ms =====
void timer1_callback(rcl_timer_t *timer1, int64_t last_call_time) {
  enc_left_msg.data  = pulseLeft;
  enc_right_msg.data = pulseRight;

  rcl_publish(&enc_left_pub,  &enc_left_msg,  NULL);
  rcl_publish(&enc_right_pub, &enc_right_msg, NULL);
}

bool micro_ros_init_successful = false;

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);
  pinMode(l_pwm_pin,  OUTPUT);

  pinMode(r_dir1_pin, OUTPUT);
  pinMode(r_dir2_pin, OUTPUT);
  pinMode(r_pwm_pin,  OUTPUT);

  pinMode(ENCA_r, INPUT);
  pinMode(ENCB_r, INPUT);

  pinMode(ENCA_l, INPUT);
  pinMode(ENCB_l, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_r), readEncoder_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_l), readEncoder_l, RISING);

  set_microros_transports();          // เรียกแค่ครั้งเดียว
  micro_ros_init_successful = false;

}

// ===== Loop =====
void loop() {
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
    if (!micro_ros_init_successful) {
      create_entities();
      micro_ros_init_successful = true;
    } else {
      // โค้ดเดิม
      noInterrupts();
      long newLeft  = pos_l;
      long newRight = pos_r;
      interrupts();

      if (newLeft != oldLeft)   { oldLeft   = newLeft;  pulseLeft  = newLeft; }
      if (newRight != oldRight) { oldRight  = newRight; pulseRight = newRight; }

      int pwm_l = constrain(abs(factor_left)  * 255, 0, 255);
      int pwm_r = constrain(abs(factor_right) * 255, 0, 255);

      drive_left (dir_left,  pwm_l);
      drive_right(dir_right, pwm_r);

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
  } else if (micro_ros_init_successful) {
    // agent หลุด → destroy แล้วรอ reconnect
    destroy_entities();
    micro_ros_init_successful = false;

    // หยุดมอเตอร์ด้วย
    drive_left(0, 0);
    drive_right(0, 0);
  }
}