#include <PID_v1.h>
#include <CAN.h>

// Pins
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 21
#define MOTOR_PWM_PIN 22
#define MOTOR_DIR_PIN 23

#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4

// Constants
const float reduction_ratio = 13.7;  // Gear reduction
const int encoder_ppr = 13;          // Pulses per revolution of motor shaft
const float wheel_diameter = 120.0;  // Wheel diameter in mm

int currentState;
int lastState;

// Variables
volatile long encoder_count = 0;  // Encoder pulse count
float target_rpm = 100.0;           // Target RPM
float current_rpm = 0.0;          // Measured RPM
float motor_pwm = 0.0;            // PID output

// PID variables
double pid_input, pid_output, pid_setpoint;
PID pid(&pid_input, &pid_output, &pid_setpoint, 0.1, 0.0, 0.0, DIRECT);  // Tune Kp, Ki, Kd

// Encoder ISR
void IRAM_ATTR encoder_isr() {
  currentState = digitalRead(ENCODER_PIN_A);
  if (currentState != lastState && currentState == 1) {
    if (digitalRead(ENCODER_PIN_B) != currentState)
      encoder_count--;
    else
      encoder_count++;
  }
  lastState = currentState;
}

// Function to calculate RPM
void calculate_rpm() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  static long last_encoder_count = 0;

  long delta_count = encoder_count - last_encoder_count;
  last_encoder_count = encoder_count;

  float time_diff = (now - last_time) / 1000.0;  // Time difference in seconds
  if (time_diff > 0) {
    float motor_rps = (float)delta_count / encoder_ppr / time_diff;
    current_rpm = motor_rps * 60.0 / reduction_ratio;  // Convert to RPM
  }

  last_time = now;
}

// Motor control function
void set_motor_speed(float pwm_value) {
  if (pwm_value >= 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH);  // Forward direction
  } else {
    digitalWrite(MOTOR_DIR_PIN, LOW);  // Reverse direction
    pwm_value = -pwm_value;            // Make PWM value positive
  }
  pwm_value = constrain(pwm_value, 0, 255);  // Limit PWM to 0-255
  analogWrite(MOTOR_PWM_PIN, pwm_value);
}

// Setup function
void setup() {
  // Initialize Serial
  Serial.begin(115200);

  Serial.println("CAN Receiver");

  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  if(!CAN.begin(500E3)){
    Serial.println("Starting CAN failed!");
    while(1);
  }

  // Initialize encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr, CHANGE);

  // Initialize motor pins
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);  // Default direction
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // PID Setup
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // Allow positive and negative outputs for direction control
}

// Main loop
void loop() {
  static unsigned long last_pid_time = 0;

  int packetSize = CAN.parsePacket();

  if(packetSize){
    Serial.print("Recieved");

    if(CAN.packetExtended()){
      Serial.print("extended");
    }

    if(CAN.packetRtr()){
      Serial.print("RTR");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if(CAN.packetRtr()){
      Serial.print(" and request length ");
      Serial.println(CAN.packetDlc());
    }else{
      Serial.print(" and length ");
      Serial.println(packetSize);

      while(CAN.available()){
        Serial.print((char) CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  }

  // Calculate RPM every 100ms
  if (millis() - last_pid_time >= 100) {
    calculate_rpm();

    // PID control
    pid_input = current_rpm;
    pid_setpoint = target_rpm;
    pid.Compute();
    motor_pwm+=pid_output;
    set_motor_speed(motor_pwm);  // Update motor speed and direction

    //Print debug information
    Serial.print("TargetRPM:");
    Serial.print(target_rpm);
    Serial.print(",");
    Serial.print("CurrentRPM:");
    Serial.print(current_rpm);
    Serial.print(",");
    Serial.print("PID Output:");
    Serial.print(pid_output);
    Serial.print(",");
    Serial.print("enc:");
    Serial.println(encoder_count);
    
    last_pid_time = millis();
  }

  // Simulate input for testing (replace this with Serial input or buttons)
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'w') {
      target_rpm += 10.0;  // Increase RPM
    } else if (command == 's') {
      target_rpm = max(0.0, target_rpm - 10.0);  // Decrease RPM
    } else if (command == 'x') {
      target_rpm = 0.0;  // Stop motor
    }
  }
}