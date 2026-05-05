#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

// ===== Pin Definition =====
#define PIN_PWM   4
#define PIN_IN1   2
#define PIN_IN2   3
#define PIN_ENCA  5
#define PIN_ENCB  6

const int PWM_MIN = 140;
const int PWM_MAX = 255;

// ===== Encoder =====
volatile long encoderCount = 0;

void encoderISR() {
  int b = digitalRead(PIN_ENCB);
  if (b > 0) encoderCount--;
  else       encoderCount++;
}

// ===== Motor =====
void setMotor(int pwmVal) {
  if (pwmVal > 0) {
    int actualPWM = map(pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_PWM, actualPWM);
  } else if (pwmVal < 0) {
    int actualPWM = map(-pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, actualPWM);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
  }
}

// ===== PID Parameters =====
float Kp = 3.0;
float Ki = 5.0;
float Kd = 0.0;

float targetRPM  = 0.0;
float integral   = 0;
float prevError  = 0;
long  prevCount  = 0;

const int   LOOP_MS  = 100;
const float LOOP_SEC = LOOP_MS / 1000.0;
const float PULSES_PER_REV = 973.0;

// ===== micro-ROS =====
rcl_publisher_t    rpm_pub;       // publish actual RPM
rcl_publisher_t    enc_pub;       // publish encoder count
rcl_subscription_t target_sub;   // subscribe target RPM

std_msgs__msg__Float32 rpm_msg;
std_msgs__msg__Int32   enc_msg;
std_msgs__msg__Float32 target_msg;

rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t executor;
rcl_node_t      node;
rcl_timer_t     pid_timer;

bool micro_ros_init_successful = false;

// ===== Callback: รับ target RPM จาก ROS =====
void target_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  targetRPM = msg->data;
  integral  = 0;   // reset integral เมื่อ setpoint เปลี่ยน
  prevError = 0;
}

// ===== Timer Callback: PID Loop + Publish ทุก 100ms =====
void pid_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  // อ่าน encoder
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();

  long deltaPulses = currentCount - prevCount;
  prevCount = currentCount;

  float actualRPM = (deltaPulses / PULSES_PER_REV) * (60.0 / LOOP_SEC);

  // PID
  if (targetRPM == 0.0) {
    setMotor(0);
    integral  = 0;
    prevError = 0;
  } else {
    float error      = targetRPM - actualRPM;
    integral        += error * LOOP_SEC;
    integral         = constrain(integral, -100, 100);
    float derivative = (error - prevError) / LOOP_SEC;
    prevError        = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    int   pwmOut = (int)constrain(output, -255, 255);
    setMotor(pwmOut);
  }

  // Publish actual RPM
  rpm_msg.data = actualRPM;
  rcl_publish(&rpm_pub, &rpm_msg, NULL);

  // Publish encoder count
  enc_msg.data = (int32_t)currentCount;
  rcl_publish(&enc_pub, &enc_msg, NULL);
}

// ===== Create micro-ROS Entities =====
void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_pid_node", "", &support);

  // Publishers
  rclc_publisher_init_default(&rpm_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actual_rpm");
  rclc_publisher_init_default(&enc_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder_count");

  // Subscriber
  rclc_subscription_init_default(&target_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "target_rpm");

  // Timer 100ms สำหรับ PID loop
  rclc_timer_init_default(&pid_timer, &support,
    RCL_MS_TO_NS(100), pid_timer_callback);

  // Executor: 1 timer + 1 subscriber
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &pid_timer);
  rclc_executor_add_subscription(&executor, &target_sub,
    &target_msg, &target_callback, ON_NEW_DATA);
}

// ===== Destroy micro-ROS Entities =====
void destroy_entities() {
  rcl_publisher_fini(&rpm_pub,    &node);
  rcl_publisher_fini(&enc_pub,    &node);
  rcl_subscription_fini(&target_sub, &node);
  rcl_timer_fini(&pid_timer);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM,  OUTPUT);
  pinMode(PIN_IN1,  OUTPUT);
  pinMode(PIN_IN2,  OUTPUT);
  pinMode(PIN_ENCA, INPUT_PULLUP);
  pinMode(PIN_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), encoderISR, RISING);

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
    // Agent หลุด → หยุดมอเตอร์ + destroy
    setMotor(0);
    targetRPM = 0;
    integral  = 0;
    destroy_entities();
    micro_ros_init_successful = false;
  }
}