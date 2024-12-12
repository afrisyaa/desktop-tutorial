#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <PID_v1.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
// Pins
#define ENCODER_PIN_A 12
#define ENCODER_PIN_B 13
#define MOTOR_PWM_PIN 15
#define MOTOR_DIR_PIN 2

// Constants
const float reduction_ratio = 13.7;    // Gear reduction
const int encoder_ppr = 13;            // Pulses per revolution of motor shaft
const float wheel_diameter = 120.0;    // Wheel diameter in mm
const int pwm_frequency = 5000;        // PWM frequency
const int pwm_resolution = 8;          // PWM resolution (8 bits)

// Variables
volatile long encoder_count = 0;       // Encoder pulse count
float target_rpm = 0.0;                // Target RPM from the ROS topic
float current_rpm = 0.0;               // Measured RPM
float motor_pwm = 0.0;                 // PID output

// PID variables
double pid_input, pid_output, pid_setpoint;
PID pid(&pid_input, &pid_output, &pid_setpoint, 2.0, 0.5, 0.1, DIRECT); // Tune Kp, Ki, Kd

// ROS Node and Subscriber
rcl_publisher_t rpm_publisher;
rcl_subscription_t target_rpm_subscriber;
std_msgs__msg__Float32 target_rpm_msg;
std_msgs__msg__Float32 current_rpm_msg;

// Encoder ISR
void IRAM_ATTR encoder_isr() {
  int b = digitalRead(ENCODER_PIN_B);
  encoder_count += (b > 0) ? 1 : -1;
}

// Function to calculate RPM
void calculate_rpm() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  long delta_count = encoder_count;
  encoder_count = 0;

  float motor_rps = (float)delta_count / encoder_ppr / ((now - last_time) / 1000.0);
  current_rpm = motor_rps * 60.0 / reduction_ratio;

  last_time = now;
}

// ROS Target RPM Callback
void target_rpm_callback(const void *msg_in) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
  target_rpm = msg->data;
  pid_setpoint = target_rpm;
}

// Motor control function
void set_motor_speed(float pwm_value) {
  if (pwm_value >= 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH); // Forward direction
  } else {
    digitalWrite(MOTOR_DIR_PIN, LOW);  // Reverse direction
    pwm_value = -pwm_value;           // Make PWM value positive
  }
  analogWrite(MOTOR_PWM_PIN,pwm_value);
}

// Setup function
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  // Initialize encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr, CHANGE);

  // Initialize motor pins
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW); // Default direction

  pinMode(MOTOR_PWM_PIN,OUTPUT);

  // PID Setup
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // Allow positive and negative outputs for direction control

  // Initialize micro-ROS
  set_microros_transports();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_motor_controller", "", &support);

  // Publisher
  rclc_publisher_init_default(
    &rpm_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
    "currentrpm");

  // Subscriber
  rclc_subscription_init_default(
    &target_rpm_subscriber, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
    "targetrpm");

  // Executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &target_rpm_subscriber, &target_rpm_msg, &target_rpm_callback, ON_NEW_DATA);
}

// Main loop
void loop() {
  static unsigned long last_pid_time = 0;

  // micro-ROS spin
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Calculate RPM every 100ms
  if (millis() - last_pid_time >= 100) {
    calculate_rpm();

    // PID control
    pid_input = current_rpm;
    pid.Compute();
    set_motor_speed(pid_output); // Update motor speed and direction

    // Publish current RPM
    current_rpm_msg.data = current_rpm;
    rcl_publish(&rpm_publisher, &current_rpm_msg, NULL);

    last_pid_time = millis();
  }
}
