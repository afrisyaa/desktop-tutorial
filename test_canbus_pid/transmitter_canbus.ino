//transmitter
#include <CAN.h>

// CAN Pins
#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4

// Function Prototypes
void sendTargetRPM(uint16_t deviceID, float targetRPM);
void sendPIDValues(uint16_t deviceID, double Kp, double Ki, double Kd);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("CAN Sender");

  // Set the CAN pins
  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  // Start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // Example usage: Send target RPM to device with ID 0x101
  sendTargetRPM(0x101, 120.0);

  // Example: Set PID values for the same device
  sendPIDValues(0x101, 0.2, 0.1, 0.05);  // Only call when PID values need updating

  delay(2000); // Wait for 2 seconds
}

// Function to send target RPM
void sendTargetRPM(uint16_t deviceID, float targetRPM) {
  Serial.print("Sending Target RPM to ID 0x");
  Serial.print(deviceID, HEX);
  Serial.print(": ");
  Serial.println(targetRPM);

  CAN.beginPacket(deviceID);
  CAN.write('T');  // Command identifier for Target RPM
  CAN.write((uint8_t *)&targetRPM, sizeof(targetRPM)); // Send target RPM as 4 bytes
  CAN.endPacket();
}

// Function to send PID values
void sendPIDValues(uint16_t deviceID, double Kp, double Ki, double Kd) {
  Serial.print("Sending PID values to ID 0x");
  Serial.print(deviceID, HEX);
  Serial.print(": Kp=");
  Serial.print(Kp);
  Serial.print(", Ki=");
  Serial.print(Ki);
  Serial.print(", Kd=");
  Serial.println(Kd);

  CAN.beginPacket(deviceID);
  CAN.write('P');  // Command identifier for PID
  CAN.write((uint8_t *)&Kp, sizeof(Kp));  // Send Kp
  CAN.write((uint8_t *)&Ki, sizeof(Ki));  // Send Ki
  CAN.write((uint8_t *)&Kd, sizeof(Kd));  // Send Kd
  CAN.endPacket();
}
