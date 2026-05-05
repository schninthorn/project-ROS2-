#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <rmw_microros/rmw_microros.h>

// ===== Pin Definition =====
#define L_PWM_PIN  4
#define L_IN1_PIN  2
#define L_IN2_PIN  3
#define L_ENCA_PIN 5
#define L_ENCB_PIN 6

#define R_PWM_PIN  9
#define R_IN1_PIN  7
#define R_IN2_PIN  8
#define R_ENCA_PIN 11
#define R_ENCB_PIN 10

const int PWM_MIN = 140;
const int PWM_MAX = 255;

// ===== Encoder =====
volatile long encoderLeft  = 0;
volatile long encoderRight = 0;

void encoderISR_L() {
  int b = digitalRead(L_ENCB_PIN);
  if (b > 0) encoderLeft--;
  else       encoderLeft++;
}

void encoderISR_R() {
  int b = digitalRead(R_ENCB_PIN);
  if (b > 0) encoderRight++;
  else       encoderRight--;
}

// ===== Motor Left =====
void setMotorLeft(int pwmVal) {
  if (pwmVal > 0) {
    int actualPWM = map(pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(L_IN1_PIN, LOW);
    digitalWrite(L_IN2_PIN, HIGH);
    analogWrite(L_PWM_PIN, actualPWM);
  } else if (pwmVal < 0) {
    int actualPWM = map(-pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(L_IN1_PIN, HIGH);
    digitalWrite(L_IN2_PIN, LOW);
    analogWrite(L_PWM_PIN, actualPWM);
  } else {
    digitalWrite(L_IN1_PIN, LOW);
    digitalWrite(L_IN2_PIN, LOW);
    analogWrite(L_PWM_PIN, 0);
  }
}

// ===== Motor Right (สลับทิศทางจาก Left) =====
void setMotorRight(int pwmVal) {
  if (pwmVal > 0) {
    int actualPWM = map(pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(R_IN1_PIN, HIGH);   // สลับ: HIGH แทน LOW
    digitalWrite(R_IN2_PIN, LOW);    // สลับ: LOW  แทน HIGH
    analogWrite(R_PWM_PIN, actualPWM);
  } else if (pwmVal < 0) {
    int actualPWM = map(-pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(R_IN1_PIN, LOW);    // สลับ: LOW  แทน HIGH
    digitalWrite(R_IN2_PIN, HIGH);   // สลับ: HIGH แทน LOW
    analogWrite(R_PWM_PIN, actualPWM);
  } else {
    digitalWrite(R_IN1_PIN, LOW);
    digitalWrite(R_IN2_PIN, LOW);
    analogWrite(R_PWM_PIN, 0);
  }
}

// ===== PID Parameters: ชุด Left =====
float Kp_L = 3.0;
float Ki_L = 5.0;
float Kd_L = 0.0;

// ===== PID Parameters: ชุด Right =====
float Kp_R = 3.0;
float Ki_R = 5.0;
float Kd_R = 0.0;

const int   LOOP_MS  = 100;
const float LOOP_SEC = LOOP_MS / 1000.0;
const float PULSES_PER_REV = 973.0;

// ===== PID State: Left =====
float targetRPM_L = 0.0;
float integral_L  = 0.0;
float prevError_L = 0.0;
long  prevCount_L = 0;

// ===== PID State: Right =====
float targetRPM_R = 0.0;
float integral_R  = 0.0;
float prevError_R = 0.0;
long  prevCount_R = 0;

// ===== PWM output (for feedback) =====
float pwmOutL = 0.0;
float pwmOutR = 0.0;

// ===== micro-ROS =====
rcl_publisher_t    rpm_left_pub;
rcl_publisher_t    rpm_right_pub;
rcl_publisher_t    enc_left_pub;
rcl_publisher_t    enc_right_pub;
rcl_publisher_t    feedback_vel_pub;

rcl_subscription_t target_left_sub;
rcl_subscription_t target_right_sub;
rcl_subscription_t pid_gain_L_sub;    // PID gain ชุด Left
rcl_subscription_t pid_gain_R_sub;    // PID gain ชุด Right

std_msgs__msg__Float32          rpm_left_msg;
std_msgs__msg__Float32          rpm_right_msg;
std_msgs__msg__Int32            enc_left_msg;
std_msgs__msg__Int32            enc_right_msg;
std_msgs__msg__Float32          target_left_msg;
std_msgs__msg__Float32          target_right_msg;
geometry_msgs__msg__Twist       feedback_vel_msg;
geometry_msgs__msg__Vector3     pid_gain_L_msg;   // รับ Kp/Ki/Kd ของ Left
geometry_msgs__msg__Vector3     pid_gain_R_msg;   // รับ Kp/Ki/Kd ของ Right

rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t executor;
rcl_node_t      node;
rcl_timer_t     pid_timer;

bool micro_ros_init_successful = false;

// ===== PID Compute Helper (รับ gain แยกแต่ละชุด) =====
int computePID(float target, float actual,
               float &integral, float &prevError,
               float kp, float ki, float kd) {
  float error   = target - actual;
  integral     += error * LOOP_SEC;
  integral      = constrain(integral, -100, 100);
  float deriv   = (error - prevError) / LOOP_SEC;
  prevError     = error;
  float output  = kp * error + ki * integral + kd * deriv;
  return (int)constrain(output, -255, 255);
}

// ===== counter_RPM =====
void counter_RPM(float rpmL, float rpmR) {
  feedback_vel_msg.linear.x  = rpmL;
  feedback_vel_msg.linear.y  = rpmR;
  feedback_vel_msg.angular.x = pwmOutL;
  feedback_vel_msg.angular.y = pwmOutR;
  feedback_vel_msg.angular.z = Ki_L;   // ใช้ Ki_L เป็นตัวแทน (หรือจะส่งค่าอื่นก็ได้)
}

// ===== counter_tick =====
void counter_tick(long countL, long countR) {
  enc_left_msg.data  = (int32_t)countL;
  enc_right_msg.data = (int32_t)countR;
  rcl_publish(&enc_left_pub,  &enc_left_msg,  NULL);
  rcl_publish(&enc_right_pub, &enc_right_msg, NULL);
}

// ===== Callbacks =====
void target_left_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  targetRPM_L = msg->data;
  integral_L  = 0;
  prevError_L = 0;
}

void target_right_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  targetRPM_R = msg->data;
  integral_R  = 0;
  prevError_R = 0;
}

// pid_gain_left_callback: รับ Kp/Ki/Kd ของมอเตอร์ซ้าย
// Publish: ros2 topic pub /pid_gain_left geometry_msgs/msg/Vector3 "{x: 3.0, y: 5.0, z: 0.0}"
void pid_gain_L_callback(const void * msgin) {
  const geometry_msgs__msg__Vector3 * gain = (const geometry_msgs__msg__Vector3 *)msgin;
  Kp_L = gain->x;
  Ki_L = gain->y;
  Kd_L = gain->z;
}

// pid_gain_right_callback: รับ Kp/Ki/Kd ของมอเตอร์ขวา
// Publish: ros2 topic pub /pid_gain_right geometry_msgs/msg/Vector3 "{x: 3.0, y: 5.0, z: 0.0}"
void pid_gain_R_callback(const void * msgin) {
  const geometry_msgs__msg__Vector3 * gain = (const geometry_msgs__msg__Vector3 *)msgin;
  Kp_R = gain->x;
  Ki_R = gain->y;
  Kd_R = gain->z;
}

// ===== Timer Callback: PID Loop ทุก 100ms =====
void pid_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

  noInterrupts();
  long countL = encoderLeft;
  long countR = encoderRight;
  interrupts();

  long deltaL = countL - prevCount_L;
  long deltaR = countR - prevCount_R;
  prevCount_L = countL;
  prevCount_R = countR;

  float actualRPM_L = (deltaL / PULSES_PER_REV) * (60.0 / LOOP_SEC);
  float actualRPM_R = (deltaR / PULSES_PER_REV) * (60.0 / LOOP_SEC);

  // --- PID Left (ใช้ Kp_L, Ki_L, Kd_L) ---
  if (targetRPM_L == 0.0) {
    setMotorLeft(0);
    integral_L  = 0;
    prevError_L = 0;
    pwmOutL = 0;
  } else {
    int pwmL = computePID(targetRPM_L, actualRPM_L,
                          integral_L, prevError_L,
                          Kp_L, Ki_L, Kd_L);
    setMotorLeft(pwmL);
    pwmOutL = (pwmL == 0) ? 0 : map(abs(pwmL), 1, 255, PWM_MIN, PWM_MAX);
  }

  // --- PID Right (ใช้ Kp_R, Ki_R, Kd_R) ---
  if (targetRPM_R == 0.0) {
    setMotorRight(0);
    integral_R  = 0;
    prevError_R = 0;
    pwmOutR = 0;
  } else {
    int pwmR = computePID(targetRPM_R, actualRPM_R,
                          integral_R, prevError_R,
                          Kp_R, Ki_R, Kd_R);
    setMotorRight(pwmR);
    pwmOutR = (pwmR == 0) ? 0 : map(abs(pwmR), 1, 255, PWM_MIN, PWM_MAX);
  }

  // --- Publish RPM ---
  rpm_left_msg.data  = actualRPM_L;
  rpm_right_msg.data = actualRPM_R;
  rcl_publish(&rpm_left_pub,  &rpm_left_msg,  NULL);
  rcl_publish(&rpm_right_pub, &rpm_right_msg, NULL);

  // --- counter_tick & counter_RPM ---
  counter_tick(countL, countR);
  counter_RPM(actualRPM_L, actualRPM_R);
  rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL);
}

// ===== Create Entities =====
void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_pid_node", "", &support);

  // Publishers
  rclc_publisher_init_default(&rpm_left_pub,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actual_rpm_left");
  rclc_publisher_init_default(&rpm_right_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actual_rpm_right");
  rclc_publisher_init_default(&enc_left_pub,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),   "encoder_left");
  rclc_publisher_init_default(&enc_right_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),   "encoder_right");
  rclc_publisher_init_default(&feedback_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "feedback_vel");

  // Subscribers
  rclc_subscription_init_default(&target_left_sub,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_left");
  rclc_subscription_init_default(&target_right_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_command_right");
  rclc_subscription_init_default(&pid_gain_L_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "pid_gain_left");
  rclc_subscription_init_default(&pid_gain_R_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "pid_gain_right");

  // Timer 100ms
  rclc_timer_init_default(&pid_timer, &support,
    RCL_MS_TO_NS(100), pid_timer_callback);

  // Executor: 1 timer + 4 subscribers = 5
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &pid_timer);
  rclc_executor_add_subscription(&executor, &target_left_sub,
    &target_left_msg,  &target_left_callback,  ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &target_right_sub,
    &target_right_msg, &target_right_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_gain_L_sub,
    &pid_gain_L_msg, &pid_gain_L_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_gain_R_sub,
    &pid_gain_R_msg, &pid_gain_R_callback, ON_NEW_DATA);
}

// ===== Destroy Entities =====
void destroy_entities() {
  rcl_publisher_fini(&rpm_left_pub,      &node);
  rcl_publisher_fini(&rpm_right_pub,     &node);
  rcl_publisher_fini(&enc_left_pub,      &node);
  rcl_publisher_fini(&enc_right_pub,     &node);
  rcl_publisher_fini(&feedback_vel_pub,  &node);
  rcl_subscription_fini(&target_left_sub,  &node);
  rcl_subscription_fini(&target_right_sub, &node);
  rcl_subscription_fini(&pid_gain_L_sub,   &node);
  rcl_subscription_fini(&pid_gain_R_sub,   &node);
  rcl_timer_fini(&pid_timer);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_IN1_PIN, OUTPUT);
  pinMode(L_IN2_PIN, OUTPUT);
  pinMode(L_ENCA_PIN, INPUT_PULLUP);
  pinMode(L_ENCB_PIN, INPUT_PULLUP);

  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_IN1_PIN, OUTPUT);
  pinMode(R_IN2_PIN, OUTPUT);
  pinMode(R_ENCA_PIN, INPUT_PULLUP);
  pinMode(R_ENCB_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENCA_PIN), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA_PIN), encoderISR_R, RISING);

  set_microros_transports();
  micro_ros_init_successful = false;
}

// ===== Loop =====
void loop() {
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
    if (!micro_ros_init_successful) {
      create_entities();
      micro_ros_init_successful = true;
    } else {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
  } else if (micro_ros_init_successful) {
    setMotorLeft(0);
    setMotorRight(0);
    targetRPM_L = 0;
    targetRPM_R = 0;
    integral_L  = 0;
    integral_R  = 0;
    destroy_entities();
    micro_ros_init_successful = false;
  }
}