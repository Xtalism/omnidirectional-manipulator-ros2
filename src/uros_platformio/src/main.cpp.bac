#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <algorithm>

rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;

std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LeftEncoder_C1 23
#define LeftEncoder_C2 22
#define RightEncoder_C1 19
#define RightEncoder_C2 18

#define IN1_FL_PIN 16    // Front-Left motor forward
#define IN2_FL_PIN 17    // Front-Left motor backward
#define IN1_FR_PIN 0     // Front-Right motor forward
#define IN2_FR_PIN 4     // Front-Right motor backward
#define IN1_RL_PIN 14    // Rear-Left motor forward
#define IN2_RL_PIN 12    // Rear-Left motor backward
#define IN1_RR_PIN 26    // Rear-Right motor forward
#define IN2_RR_PIN 27    // Rear-Right motor backward

int motorSpeedFL = 0;
int motorSpeedFR = 0;
int motorSpeedRL = 0;
int motorSpeedRR = 0;

int LeftEncoderCount = 0;
int RightEncoderCount = 0;

void LeftEncoderCallback();
void RightEncoderCallback();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setMecanumMotorSpeeds(int speedFL, int speedFR, int speedRL, int speedRR); // prototype function

void error_loop(){
  while(1){
    delay(500);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  } else if (value < -maxLimit) {
    return -maxLimit;
  }
  return value;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));

    right_encoder_msg.data = RightEncoderCount;
    left_encoder_msg.data = -LeftEncoderCount;
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_x = msg->linear.x;    // Forward/Backward
  float linear_y = msg->linear.y;    // Strafe Left/Right
  float angular_z = msg->angular.z;  // Rotation

  // Mecanum kinematics (simplified, adjust signs/factors as needed for your specific robot build)
  // Vx + Vy + Vz (Front Right)
  // Vx - Vy + Vz (Rear Right)
  // Vx - Vy - Vz (Front Left)
  // Vx + Vy - Vz (Rear Left)
  // The scaling factor (e.g., *1000) and offset (e.g., +/-40) are applied after kinematic calculation.

  float rawSpeedFL = linear_x - linear_y - angular_z;
  float rawSpeedFR = linear_x + linear_y + angular_z;
  float rawSpeedRL = linear_x + linear_y - angular_z;
  float rawSpeedRR = linear_x - linear_y + angular_z;

  float max_raw_speed = 0.0;
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedFL));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedFR));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedRL));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedRR));

  if (max_raw_speed > 1.0) {
    rawSpeedFL /= max_raw_speed;
    rawSpeedFR /= max_raw_speed;
    rawSpeedRL /= max_raw_speed;
    rawSpeedRR /= max_raw_speed;
  }

  int scale_factor = 1000;
  int offset = 40;

  motorSpeedFL = static_cast<int>(rawSpeedFL * scale_factor);
  motorSpeedFR = static_cast<int>(rawSpeedFR * scale_factor);
  motorSpeedRL = static_cast<int>(rawSpeedRL * scale_factor);
  motorSpeedRR = static_cast<int>(rawSpeedRR * scale_factor);

  // Apply offset
  if (motorSpeedFL > 0) motorSpeedFL += offset; else if (motorSpeedFL < 0) motorSpeedFL -= offset;
  if (motorSpeedFR > 0) motorSpeedFR += offset; else if (motorSpeedFR < 0) motorSpeedFR -= offset;
  if (motorSpeedRL > 0) motorSpeedRL += offset; else if (motorSpeedRL < 0) motorSpeedRL -= offset;
  if (motorSpeedRR > 0) motorSpeedRR += offset; else if (motorSpeedRR < 0) motorSpeedRR -= offset;
  
  setMecanumMotorSpeeds(motorSpeedFL, motorSpeedFR, motorSpeedRL, motorSpeedRR);
}

void setMecanumMotorSpeeds(int speedFL, int speedFR, int speedRL, int speedRR) {
  int max_pwm = 250;

  int actualSpeedFL = limitToMaxValue(speedFL, max_pwm);
  if (actualSpeedFL > 0) {
    digitalWrite(IN2_FL_PIN, LOW);
    analogWrite(IN1_FL_PIN, abs(actualSpeedFL));
  } else {
    digitalWrite(IN1_FL_PIN, LOW);
    analogWrite(IN2_FL_PIN, abs(actualSpeedFL));
  }

  int actualSpeedFR = limitToMaxValue(speedFR, max_pwm);
  if (actualSpeedFR > 0) {
    digitalWrite(IN2_FR_PIN, LOW);
    analogWrite(IN1_FR_PIN, abs(actualSpeedFR));
  } else {
    digitalWrite(IN1_FR_PIN, LOW);
    analogWrite(IN2_FR_PIN, abs(actualSpeedFR));
  }

  int actualSpeedRL = limitToMaxValue(speedRL, max_pwm);
  if (actualSpeedRL > 0) {
    digitalWrite(IN2_RL_PIN, LOW);
    analogWrite(IN1_RL_PIN, abs(actualSpeedRL));
  } else {
    digitalWrite(IN1_RL_PIN, LOW);
    analogWrite(IN2_RL_PIN, abs(actualSpeedRL));
  }

  int actualSpeedRR = limitToMaxValue(speedRR, max_pwm);
  if (actualSpeedRR > 0) {
    digitalWrite(IN2_RR_PIN, LOW);
    analogWrite(IN1_RR_PIN, abs(actualSpeedRR));
  } else {
    digitalWrite(IN1_RR_PIN, LOW);
    analogWrite(IN2_RR_PIN, abs(actualSpeedRR));
  }

  bool all_zero = (actualSpeedFL == 0 && actualSpeedFR == 0 && actualSpeedRL == 0 && actualSpeedRR == 0);
  
  if (all_zero) {
    analogWrite(IN1_FL_PIN, 0); digitalWrite(IN1_FL_PIN, LOW);
    analogWrite(IN2_FL_PIN, 0); digitalWrite(IN2_FL_PIN, LOW);
    analogWrite(IN1_FR_PIN, 0); digitalWrite(IN1_FR_PIN, LOW);
    analogWrite(IN2_FR_PIN, 0); digitalWrite(IN2_FR_PIN, LOW);
    analogWrite(IN1_RL_PIN, 0); digitalWrite(IN1_RL_PIN, LOW);
    analogWrite(IN2_RL_PIN, 0); digitalWrite(IN2_RL_PIN, LOW);
    analogWrite(IN1_RR_PIN, 0); digitalWrite(IN1_RR_PIN, LOW);
    analogWrite(IN2_RR_PIN, 0); digitalWrite(IN2_RR_PIN, LOW);
  }
}

void setup() {
  set_microros_transports();
  // set_microros_wifi_transports("MotherBase", "@6830135@", "192.168.148.143", 8888);

  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  pinMode(IN1_FL_PIN, OUTPUT); pinMode(IN2_FL_PIN, OUTPUT);
  pinMode(IN1_FR_PIN, OUTPUT); pinMode(IN2_FR_PIN, OUTPUT);
  pinMode(IN1_RL_PIN, OUTPUT); pinMode(IN2_RL_PIN, OUTPUT);
  pinMode(IN1_RR_PIN, OUTPUT); pinMode(IN2_RR_PIN, OUTPUT);

  digitalWrite(IN1_FL_PIN, LOW); digitalWrite(IN2_FL_PIN, LOW); analogWrite(IN1_FL_PIN,0); analogWrite(IN2_FL_PIN,0);
  digitalWrite(IN1_FR_PIN, LOW); digitalWrite(IN2_FR_PIN, LOW); analogWrite(IN1_FR_PIN,0); analogWrite(IN2_FR_PIN,0);
  digitalWrite(IN1_RL_PIN, LOW); digitalWrite(IN2_RL_PIN, LOW); analogWrite(IN1_RL_PIN,0); analogWrite(IN2_RL_PIN,0);
  digitalWrite(IN1_RR_PIN, LOW); digitalWrite(IN2_RR_PIN, LOW); analogWrite(IN1_RR_PIN,0); analogWrite(IN2_RR_PIN,0);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

  delay(2000);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "omnidirectional_manipulator_esp32", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_motor_ticks"));

  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_motor_ticks"));

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator)); // Number of handles : 1 time + 3 subcribers = 4 (publishers not counted)
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void LeftEncoderCallback() {
  static boolean LeftEncoder_C1_Last = LOW;
  static boolean LeftEncoder_C2_Last = LOW;
  boolean LeftEncoder_C1_Current = digitalRead(LeftEncoder_C1);
  boolean LeftEncoder_C2_Current = digitalRead(LeftEncoder_C2);

  if (LeftEncoder_C1_Current != LeftEncoder_C1_Last) { // A channel has changed
    if (LeftEncoder_C1_Current == HIGH) { // Rising edge on A
      if (LeftEncoder_C2_Current == LOW) { // B is low
        LeftEncoderCount++; // Clockwise
      } else {
        LeftEncoderCount--; // Counter-clockwise
      }
    } else { // Falling edge on A
      if (LeftEncoder_C2_Current == HIGH) { // B is high
        LeftEncoderCount++; // Clockwise
      } else {
        LeftEncoderCount--; // Counter-clockwise
      }
    }
  }
  LeftEncoder_C1_Last = LeftEncoder_C1_Current;
  LeftEncoder_C2_Last = LeftEncoder_C2_Current; // Not strictly needed
}

void RightEncoderCallback() {
  static boolean RightEncoder_C1_Last = LOW;
  static boolean RightEncoder_C2_Last = LOW;
  boolean RightEncoder_C1_Current = digitalRead(RightEncoder_C1);
  boolean RightEncoder_C2_Current = digitalRead(RightEncoder_C2);

  if (RightEncoder_C1_Current != RightEncoder_C1_Last) {
    if (RightEncoder_C1_Current == HIGH) {
      if (RightEncoder_C2_Current == LOW) {
        RightEncoderCount++; 
      } else {
        RightEncoderCount--;
      }
    } else {
      if (RightEncoder_C2_Current == HIGH) {
        RightEncoderCount++;
      } else {
        RightEncoderCount--;
      }
    }
  }
  RightEncoder_C1_Last = RightEncoder_C1_Current;
}