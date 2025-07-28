#include <Wire.h>
#include <Servo.h>

// I2C Address for the Hiwonder Motor Driver
#define I2C_ADDR 0x34

// Motor Driver Register Addresses
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_SPEED_ADDR 51

// Motor Type Configuration
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// Servo Pin Definitions
#define PAN_SERVO_PIN 9
#define TILT_SERVO_PIN 10

// Global Variables
Servo panServo;
Servo tiltServo;
String inputString = "";
bool stringComplete = false;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  inputString.reserve(200);

  // Initialize I2C
  Wire.begin();
  delay(100); // Allow time for I2C to initialize

  // Configure Motor Driver
  uint8_t motorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
  uint8_t motorEncoderPolarity = 0;
  WireWriteDataArray(MOTOR_TYPE_ADDR, &motorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &motorEncoderPolarity, 1);

  // Attach Servos
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  // Center servos on startup
  panServo.write(90);
  tiltServo.write(90);
}

void loop() {
  // Process complete serial commands
  if (stringComplete) {
    inputString.trim(); // Remove any leading/trailing whitespace
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void parseCommand(String command) {
  char commandType = command.charAt(0);
  command.remove(0, 1); // Remove command type character

  switch (commandType) {
    case 'D': // Drive Command
      handleDriveCommand(command);
      break;
    case 'T': // Turret Command
      handleTurretCommand(command);
      break;
    case 'S': // Stop Command
      stopMotors();
      break;
    default:
      // Unknown command
      break;
  }
}

void handleDriveCommand(String data) {
    int firstComma = data.indexOf(',');
    if (firstComma == -1) return;

    String throttleStr = data.substring(0, firstComma);
    String steeringStr = data.substring(firstComma + 1);

    float throttle = throttleStr.toFloat();
    float steering = steeringStr.toFloat();

    // Clamp values between -1.0 and 1.0
    throttle = constrain(throttle, -1.0, 1.0);
    steering = constrain(steering, -1.0, 1.0);

    // This is a common way to mix throttle and steering for a tracked vehicle.
    // The values are scaled to the motor driver's expected range.
    // Let's assume a max speed value of 50 for this driver.
    float leftSpeed = (throttle + steering) * 50.0;
    float rightSpeed = (throttle - steering) * 50.0;

    // The Hiwonder driver seems to use negative for forward on the left motor
    // and positive for forward on the right motor based on the example.
    // We will need to confirm this with testing.
    int8_t motorSpeeds[4] = {
        (int8_t)constrain(-leftSpeed, -100, 100),
        (int8_t)constrain(rightSpeed, -100, 100),
        0, // Unused channel
        0  // Unused channel
    };

    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)motorSpeeds, 4);
}

void handleTurretCommand(String data) {
    int firstComma = data.indexOf(',');
    if (firstComma == -1) return;

    String panStr = data.substring(0, firstComma);
    String tiltStr = data.substring(firstComma + 1);

    int panAngle = panStr.toInt();
    int tiltAngle = tiltStr.toInt();

    panAngle = constrain(panAngle, 0, 180);
    tiltAngle = constrain(tiltAngle, 0, 180);

    panServo.write(panAngle);
    tiltServo.write(tiltAngle);
}


void stopMotors() {
  int8_t stopSpeeds[4] = {0, 0, 0, 0};
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)stopSpeeds, 4);
}

// Helper function to write a data array to the I2C device
bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  if (Wire.endTransmission() != 0) {
    return false;
  }
  return true;
} 