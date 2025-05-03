#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
 
// #include <WiFi.h>
 
// char ssid[] = "IZZI-1A55";
// char password[] = "109397F31A55";
// char agent_ip[] = "192.168.0.9";  // IP del agente micro-ROS
 
// Pines motores y encoders
#define ENA 12
#define IN1 14
#define IN2 27
#define ENB 13
#define IN3 16
#define IN4 17
#define EncoderA_CLK 4
#define EncoderA_DT 5
#define EncoderB_CLK 18
#define EncoderB_DT 19
 
volatile long encoderPositionR = 0;
volatile long encoderPositionL = 0;
float PPR_R = 990.0, PPR_L = 990.0;
 
rcl_node_t node;
rcl_publisher_t pub_left;
rcl_publisher_t pub_right;
rcl_subscription_t sub_cmd_vel;
rcl_timer_t timer;
 
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 msg_rad_sL, msg_rad_sR;
 
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
 
unsigned long last_time = 0;
 
void IRAM_ATTR readEncoderR() {
  if (digitalRead(EncoderA_CLK) == digitalRead(EncoderA_DT)) encoderPositionR++;
  else encoderPositionR--;
}
 
void IRAM_ATTR readEncoderL() {
  if (digitalRead(EncoderB_CLK) == digitalRead(EncoderB_DT)) encoderPositionL++;
  else encoderPositionL--;
}
 
// Callback para /cmd_vel
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
 
  float v = msg->linear.x;
  float w = msg->angular.z;
  float L = 0.15;  
 
  float vL = v - (w * L / 2.0);
  float vR = v + (w * L / 2.0);
 
  int pwmL = map(abs(vL * 100), 0, 100, 0, 255);
  int pwmR = map(abs(vR * 100), 0, 100, 0, 255);
 
  // Motor izquierdo
  digitalWrite(IN1, vL >= 0 ? HIGH : LOW);
  digitalWrite(IN2, vL >= 0 ? LOW : HIGH);
  analogWrite(ENA, pwmL);
 
  // Motor derecho
  digitalWrite(IN3, vR >= 0 ? HIGH : LOW);
  digitalWrite(IN4, vR >= 0 ? LOW : HIGH);
  analogWrite(ENB, pwmR);
}
 
// Timer callback para publicar velocidades
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;
 
  noInterrupts();
  long currentPositionR = encoderPositionR;
  long currentPositionL = encoderPositionL;
  encoderPositionR = 0;
  encoderPositionL = 0;
  interrupts();
 
  float rad_sR = (currentPositionR / PPR_R) * (2 * PI);
  float rad_sL = (currentPositionL / PPR_L) * (2 * PI);
 
  msg_rad_sR.data = rad_sR;
  msg_rad_sL.data = rad_sL;
 
  // Publicaciones con manejo del valor de retorno
  rcl_ret_t ret = rcl_publish(&pub_right, &msg_rad_sR, NULL);
  if (ret != RCL_RET_OK) {
    Serial.print("Error al publicar en pub_right: ");
    Serial.println(ret);
  }
 
  ret = rcl_publish(&pub_left, &msg_rad_sL, NULL);
  if (ret != RCL_RET_OK) {
    Serial.print("Error al publicar en pub_left: ");
    Serial.println(ret);
  }
}
 
void setup() {
  // ⚠️ Corregido: ahora usa agent_ip como char[]
  // set_microros_wifi_transports(ssid, password, agent_ip, 8888);
  set_microros_transports();
 
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EncoderA_CLK, INPUT_PULLUP);
  pinMode(EncoderA_DT, INPUT_PULLUP);
  pinMode(EncoderB_CLK, INPUT_PULLUP);
  pinMode(EncoderB_DT, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(EncoderA_CLK), readEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB_CLK), readEncoderL, CHANGE);
 
  delay(2000);
 
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
 
  // Publishers
  rclc_publisher_init_default(&pub_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_velocity/right");
  rclc_publisher_init_default(&pub_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_velocity/left");
 
  // Subscriber
  rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
 
  // Timer
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);
 
  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}
 
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}