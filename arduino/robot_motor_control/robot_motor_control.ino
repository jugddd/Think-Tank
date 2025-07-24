/*
 * Mobile Tracked Robot - Arduino Motor Control Sketch
 * 
 * This sketch runs on an Arduino Uno and serves as a real-time motor controller
 * for a mobile tracked robot. It receives high-level commands from a Raspberry Pi
 * via USB Serial and translates them into low-level I²C commands for a 
 * Hiwonder 4-Channel Motor Controller.
 * 
 * Hardware Connections:
 * - Arduino Uno connected to Raspberry Pi via USB
 * - Hiwonder 4-Channel Motor Controller connected via I²C (SDA/SCL)
 * - Motor Controller I²C Address: 0x34
 * - Motors connected to channels M1 (left) and M2 (right)
 * 
 * Communication Protocol:
 * - Serial: 9600 baud, receives "M,L,R\n" format commands
 * - I²C: Sends motor speed commands to Hiwonder controller
 * 
 * Author: Generated for Think-Tank Robot Project
 * Version: 1.0
 */

#include <Wire.h>

// Hardware Configuration
const int MOTOR_CONTROLLER_ADDRESS = 0x34;  // Hiwonder controller I²C address
const int SERIAL_BAUDRATE = 9600;           // Serial communication speed
const int COMMAND_TIMEOUT_MS = 1000;        // Stop motors if no command received

// Hiwonder Motor Controller Register Map
// Based on common Hiwonder 4-channel controller protocols
// Motors connected to M1 (left) and M3 (right) channels
const byte MOTOR1_SPEED_REG = 0x00;     // Left motor speed register (M1)
const byte MOTOR1_DIR_REG = 0x01;       // Left motor direction register (M1)
const byte MOTOR2_SPEED_REG = 0x04;     // Right motor speed register (M3)  
const byte MOTOR2_DIR_REG = 0x05;       // Right motor direction register (M3)
const byte MOTOR_ENABLE_REG = 0x08;     // Motor enable/disable register

// Motor Direction Constants
const byte MOTOR_FORWARD = 0x01;
const byte MOTOR_REVERSE = 0x02;
const byte MOTOR_STOP = 0x00;

// Global Variables
unsigned long lastCommandTime = 0;      // Timestamp of last command
int leftMotorSpeed = 0;                  // Current left motor speed (-255 to 255)
int rightMotorSpeed = 0;                 // Current right motor speed (-255 to 255)

void setup() {
  // Initialize Serial Communication
  Serial.begin(SERIAL_BAUDRATE);
  
  // Initialize I²C Communication
  Wire.begin();                          // Join I²C bus as master
  Wire.setClock(100000);                 // Set I²C clock to 100kHz (standard mode)
  
  // Initialize Motor Controller
  initializeMotorController();
  
  // Startup message
  Serial.println("===========================================");
  Serial.println("Arduino Motor Controller - READY");
  Serial.println("Waiting for commands from Raspberry Pi...");
  Serial.println("Supported commands:");
  Serial.println("  M,<left>,<right> - Motor control (-255 to 255)");
  Serial.println("  E - Emergency stop");
  Serial.println("  S - Status report");
  Serial.println("===========================================");
  
  // Stop all motors on startup
  setMotorSpeeds(0, 0);
  lastCommandTime = millis();
}

void loop() {
  // Check for incoming serial commands and process them immediately
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Safety timeout - stop motors if no commands received
  checkCommandTimeout();
}

/*
 * Initialize the Hiwonder Motor Controller
 * Sets up the controller in a known state with motors disabled
 */
void initializeMotorController() {
  Serial.println("Initializing Hiwonder Motor Controller...");
  
  // Enable the motor controller
  writeI2CRegister(MOTOR_ENABLE_REG, 0x01);
  delay(50);
  
  // Set both motors to stop
  writeI2CRegister(MOTOR1_SPEED_REG, 0);
  writeI2CRegister(MOTOR1_DIR_REG, MOTOR_STOP);
  writeI2CRegister(MOTOR2_SPEED_REG, 0);
  writeI2CRegister(MOTOR2_DIR_REG, MOTOR_STOP);
  
  Serial.println("Motor Controller initialized successfully");
}



/*
 * Process a complete command string
 * Supported commands:
 * - "M,<left_speed>,<right_speed>" - Motor control (-255 to 255)
 * - "E" - Emergency stop
 * - "S" - Status report
 */
void processCommand(String command) {
  command.trim();  // Remove any whitespace
  
  // Handle emergency stop command
  if (command.equals("E")) {
    emergencyStop();
    lastCommandTime = millis();
    return;
  }
  
  // Handle status request command
  if (command.equals("S")) {
    printStatus();
    return;
  }
  
  // Handle motor command
  if (command.startsWith("M,")) {
    // Parse the motor command - FIXED BUG: Correct comma positions
    int firstComma = command.indexOf(',');           // Find first comma after 'M'
    int secondComma = command.indexOf(',', firstComma + 1);  // Find second comma
    
    if (secondComma == -1) {
      Serial.println("ERROR: Invalid command format. Missing second value.");
      return;
    }
    
    // Extract motor speeds - FIXED BUG: Correct substring extraction
    String leftSpeedStr = command.substring(firstComma + 1, secondComma);
    String rightSpeedStr = command.substring(secondComma + 1);  // Go to end of string
    
    int leftSpeed = leftSpeedStr.toInt();
    int rightSpeed = rightSpeedStr.toInt();
    
    // Validate speed ranges
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Update motor speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
    lastCommandTime = millis();
    
    // Debug output
    Serial.print("Command received -> Left: ");
    Serial.print(leftSpeed);
    Serial.print(", Right: ");
    Serial.println(rightSpeed);
    return;
  }
  
  // Unknown command
  Serial.println("ERROR: Unknown command. Use M,<left>,<right>, E, or S");
}

/*
 * Set motor speeds for both left and right motors
 * Handles speed and direction conversion for Hiwonder controller
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotorSpeed = leftSpeed;
  rightMotorSpeed = rightSpeed;
  
  // Set left motor (M1)
  setIndividualMotor(MOTOR1_SPEED_REG, MOTOR1_DIR_REG, leftSpeed);
  
  // Set right motor (M2)  
  setIndividualMotor(MOTOR2_SPEED_REG, MOTOR2_DIR_REG, rightSpeed);
}

/*
 * Set an individual motor's speed and direction
 */
void setIndividualMotor(byte speedReg, byte dirReg, int speed) {
  byte motorSpeed;
  byte motorDirection;
  
  if (speed == 0) {
    motorSpeed = 0;
    motorDirection = MOTOR_STOP;
  } else if (speed > 0) {
    motorSpeed = (byte)speed;
    motorDirection = MOTOR_FORWARD;
  } else {
    motorSpeed = (byte)(-speed);  // Convert negative to positive
    motorDirection = MOTOR_REVERSE;
  }
  
  // Send commands to motor controller
  writeI2CRegister(speedReg, motorSpeed);
  delay(1);  // Small delay between I²C commands
  writeI2CRegister(dirReg, motorDirection);
}

/*
 * Write a value to a register on the Hiwonder Motor Controller
 */
void writeI2CRegister(byte registerAddress, byte value) {
  Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(value);
  byte result = Wire.endTransmission();
  
  if (result != 0) {
    Serial.print("I²C ERROR: Failed to write to register 0x");
    Serial.print(registerAddress, HEX);
    Serial.print(", Error code: ");
    Serial.println(result);
  }
}

/*
 * Safety function: Stop motors if no commands received within timeout period
 */
void checkCommandTimeout() {
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    if (leftMotorSpeed != 0 || rightMotorSpeed != 0) {
      Serial.println("TIMEOUT: No commands received, stopping motors for safety");
      setMotorSpeeds(0, 0);
    }
  }
}

/*
 * Emergency stop function - can be called to immediately stop all motors
 */
void emergencyStop() {
  setMotorSpeeds(0, 0);
  Serial.println("EMERGENCY STOP: All motors stopped");
}

/*
 * Status reporting function - prints current motor states
 */
void printStatus() {
  Serial.print("Motor Status -> Left: ");
  Serial.print(leftMotorSpeed);
  Serial.print(", Right: ");
  Serial.print(rightMotorSpeed);
  Serial.print(", Last Command: ");
  Serial.print(millis() - lastCommandTime);
  Serial.println("ms ago");
} 