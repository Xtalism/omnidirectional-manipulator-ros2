#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <algorithm> // For std::max and std::abs

Servo myservo;  // create servo object to control a servo
const int servoPin = 12;

#define BATTERY_PIN 36

rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int8 servo_msg;

rcl_publisher_t battery_publisher;
std_msgs__msg__Int8 battery_msg;

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

#define LEFT_LED_PIN 4
#define RIGHT_LED_PIN 16

// Encoder Pins
#define LeftEncoder_C1 23
#define LeftEncoder_C2 22
#define RightEncoder_C1 19
#define RightEncoder_C2 18

// Pin Definitions for 4 Mecanum Motors
// Assuming IN1_xx_PIN is for "forward" PWM and IN2_xx_PIN is for "backward" PWM
#define IN1_FL_PIN 14    // Front-Left motor forward
#define IN2_FL_PIN 27    // Front-Left motor backward
#define IN1_FR_PIN 25    // Front-Right motor forward
#define IN2_FR_PIN 26    // Front-Right motor backward
#define IN1_RL_PIN 33    // Rear-Left motor forward (Example Pin - CHANGE IF NEEDED)
#define IN2_RL_PIN 32    // Rear-Left motor backward (Example Pin - CHANGE IF NEEDED)
#define IN1_RR_PIN 15    // Rear-Right motor forward (Example Pin - CHANGE IF NEEDED)
#define IN2_RR_PIN 2     // Rear-Right motor backward (Example Pin - CHANGE IF NEEDED)

// Motor control variables
int motorSpeedFL = 0;
int motorSpeedFR = 0;
int motorSpeedRL = 0;
int motorSpeedRR = 0;

// Encoder variables
int LeftEncoderCount = 0;
int RightEncoderCount = 0;

void LeftEncoderCallback();
void RightEncoderCallback();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function prototypes
void setMecanumMotorSpeeds(int speedFL, int speedFR, int speedRL, int speedRR); // Changed function name
int8_t get_battery_percentage();
int map_to_percentage(int raw_value);

void error_loop(){
  while(1){
    digitalWrite(LEFT_LED_PIN, !digitalRead(LEFT_LED_PIN)); // Blink one LED for error
    digitalWrite(RIGHT_LED_PIN, !digitalRead(RIGHT_LED_PIN)); // Blink other LED for error
    delay(500); // Slower blink
  }
}

int limitToMaxValue(int value, int maxLimit) {
  // Ensure value is within -maxLimit to maxLimit, then take abs for analogWrite
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
    left_encoder_msg.data = -LeftEncoderCount; // Assuming left encoder counts opposite if robot moves forward
  
    int8_t battery_percentage = get_battery_percentage();
    battery_msg.data = battery_percentage;
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
  }
}

int8_t get_battery_percentage() {
  // Read the voltage from the BATTERY_PIN
  int raw_value = analogRead(BATTERY_PIN);

  // Convert the raw value to battery percentage (0 to 100)
  int battery_percentage = map_to_percentage(raw_value);

  return static_cast<int8_t>(battery_percentage);
}

int map_to_percentage(int raw_value) {
  // Assuming the raw_value represents the battery voltage in the range of 0 to 4095
  // Adjust the following values based on your battery voltage range and voltage divider setup (if any).
  int min_voltage = 2400;    // Minimum voltage reading (corresponding to 0% battery)
  int max_voltage = 3720; // Maximum voltage reading (corresponding to 100% battery)

  // Map the raw value to the battery percentage
  long mapped_value = map(raw_value, min_voltage, max_voltage, 0, 100);
  // Constrain the value to be within 0-100
  if (mapped_value < 0) mapped_value = 0;
  if (mapped_value > 100) mapped_value = 100;
  return static_cast<int>(mapped_value);
}


void LEDs_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;

  int8_t value = msg->data;

  switch (value) {
    case 0:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 2:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    case 3:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    default:
      // Optional: handle other cases or do nothing
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
  }
}

void servo_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t angle = msg->data;
  // Assuming servo expects 0-180, and input is -90 to 90 or similar.
  // Adjust mapping as needed. For now, let's assume input is 0-40.
  int servo_position = map(angle, 0, 40, 0, 40); // Example mapping
  servo_position = constrain(servo_position, 0, 40); // Limit to 0-40
  myservo.write(servo_position);
}

// Twist message callback for Mecanum drive
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  int linear_x = msg->linear.x;    // Forward/Backward
  float linear_y = msg->linear.y;    // Strafe Left/Right
  float angular_z = msg->angular.z;  // Rotation

  digitalWrite(LEFT_LED_PIN, (msg->linear.x <= 0) ? LOW : HIGH);

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

  // Normalize speeds if any exceeds 1.0 (or another max input value from Twist)
  // This helps maintain correct movement ratios when commands are large.
  float max_raw_speed = 0.0;
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedFL));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedFR));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedRL));
  max_raw_speed = std::max(max_raw_speed, std::abs(rawSpeedRR));

  if (max_raw_speed > 1.0) { // Assuming Twist components are typically in [-1, 1] range
    rawSpeedFL /= max_raw_speed;
    rawSpeedFR /= max_raw_speed;
    rawSpeedRL /= max_raw_speed;
    rawSpeedRR /= max_raw_speed;
  }

  // Scale to motor command range (e.g., 0-250 for PWM) and apply offset
  int scale_factor = 1000; // Your original scaling factor
  int offset = 40;         // Your original offset

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
  int max_pwm = 250; // Your defined max PWM

  // Front-Left Motor
  int actualSpeedFL = limitToMaxValue(speedFL, max_pwm);
  if (actualSpeedFL > 0) {
    digitalWrite(IN2_FL_PIN, LOW);
    analogWrite(IN1_FL_PIN, abs(actualSpeedFL));
  } else {
    digitalWrite(IN1_FL_PIN, LOW);
    analogWrite(IN2_FL_PIN, abs(actualSpeedFL));
  }

  // Front-Right Motor
  int actualSpeedFR = limitToMaxValue(speedFR, max_pwm);
  if (actualSpeedFR > 0) {
    digitalWrite(IN2_FR_PIN, LOW);
    analogWrite(IN1_FR_PIN, abs(actualSpeedFR));
  } else {
    digitalWrite(IN1_FR_PIN, LOW);
    analogWrite(IN2_FR_PIN, abs(actualSpeedFR));
  }

  // Rear-Left Motor
  int actualSpeedRL = limitToMaxValue(speedRL, max_pwm);
  if (actualSpeedRL > 0) {
    digitalWrite(IN2_RL_PIN, LOW);
    analogWrite(IN1_RL_PIN, abs(actualSpeedRL));
  } else {
    digitalWrite(IN1_RL_PIN, LOW);
    analogWrite(IN2_RL_PIN, abs(actualSpeedRL));
  }

  // Rear-Right Motor
  int actualSpeedRR = limitToMaxValue(speedRR, max_pwm);
  if (actualSpeedRR > 0) {
    digitalWrite(IN2_RR_PIN, LOW);
    analogWrite(IN1_RR_PIN, abs(actualSpeedRR));
  } else {
    digitalWrite(IN1_RR_PIN, LOW);
    analogWrite(IN2_RR_PIN, abs(actualSpeedRR));
  }

  // Stop all motors if all speeds are effectively zero (after offset and limiting)
  // This check might be redundant if zero inputs to limitToMaxValue result in zero output for analogWrite.
  // However, explicit stop is safer.
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
  // set_microros_wifi_transports("Pixel_5234", "deyz1234", "192.168.148.143", 8888);

  pinMode(LEFT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, HIGH);  

  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(RIGHT_LED_PIN, HIGH);  
  pinMode(BATTERY_PIN, INPUT);

  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  // Initialize all motor pins
  pinMode(IN1_FL_PIN, OUTPUT); pinMode(IN2_FL_PIN, OUTPUT);
  pinMode(IN1_FR_PIN, OUTPUT); pinMode(IN2_FR_PIN, OUTPUT);
  pinMode(IN1_RL_PIN, OUTPUT); pinMode(IN2_RL_PIN, OUTPUT);
  pinMode(IN1_RR_PIN, OUTPUT); pinMode(IN2_RR_PIN, OUTPUT);

  // Ensure all motors are stopped initially
  digitalWrite(IN1_FL_PIN, LOW); digitalWrite(IN2_FL_PIN, LOW); analogWrite(IN1_FL_PIN,0); analogWrite(IN2_FL_PIN,0);
  digitalWrite(IN1_FR_PIN, LOW); digitalWrite(IN2_FR_PIN, LOW); analogWrite(IN1_FR_PIN,0); analogWrite(IN2_FR_PIN,0);
  digitalWrite(IN1_RL_PIN, LOW); digitalWrite(IN2_RL_PIN, LOW); analogWrite(IN1_RL_PIN,0); analogWrite(IN2_RL_PIN,0);
  digitalWrite(IN1_RR_PIN, LOW); digitalWrite(IN2_RR_PIN, LOW); analogWrite(IN1_RR_PIN,0); analogWrite(IN2_RR_PIN,0);


  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  
  delay(2000); // Crucial delay
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "mecanum_esp32_robot", "", &support)); // Renamed node

  // LED subscriber
  RCCHECK(rclc_subscription_init_default(
    &LEDs_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "LEDs"));
    
//servo subscriber
  RCCHECK(rclc_subscription_init_default(
      &servo_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "/servo"));

  // Create twist subscriber
  RCCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "battery"));

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

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    
  // create executor
  // Number of handles: 1 timer + 3 subscribers = 4
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;

  // Indicate successful setup with LED blinks
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  delay(200);
  for(int i=0; i<3; ++i) {
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(RIGHT_LED_PIN, HIGH);
    delay(150);
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
    delay(150);
  }
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void LeftEncoderCallback() {
  // Basic quadrature encoder logic (adjust if needed)
  // This assumes C1 is A channel and C2 is B channel
  // Determine direction based on the state of C2 when C1 changes
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
  // LeftEncoder_C2_Last = LeftEncoder_C2_Current; // Not strictly needed for this logic but good for full state
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