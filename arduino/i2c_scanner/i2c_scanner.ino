/*
 * I²C Scanner Utility
 * 
 * This utility sketch scans for I²C devices connected to the Arduino.
 * Use this to verify your Hiwonder Motor Controller is properly connected
 * and responding at the expected address (0x34).
 * 
 * Usage:
 * 1. Upload this sketch to your Arduino
 * 2. Open Serial Monitor (Tools > Serial Monitor)
 * 3. Set baud rate to 9600
 * 4. Watch for detected devices
 * 
 * Expected output should show: "Found device at 0x34"
 * If no devices are found, check your I²C wiring.
 */

#include <Wire.h>

void setup() {
  Wire.begin();                    // Initialize I²C as master
  Serial.begin(9600);              // Initialize serial communication
  
  // Wait a moment for the serial monitor to connect
  delay(1000);
  
  Serial.println("I²C Device Scanner");
  Serial.println("==================");
  
  byte error, address;
  int deviceCount = 0;
  
  // Scan all possible I²C addresses (1-126)
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found I²C device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x34) {
        Serial.print(" (Hiwonder Motor Controller)");
      } else if (address == 0x48) {
        Serial.print(" (Possibly ADS1115 ADC)");
      } else if (address == 0x68) {
        Serial.print(" (Possibly MPU6050 IMU or RTC)");
      } else if (address == 0x76 || address == 0x77) {
        Serial.print(" (Possibly BMP280 pressure sensor)");
      }
      
      Serial.println();
      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  Serial.println("------------------");
  if (deviceCount == 0) {
    Serial.println("Scan complete: No I²C devices found!");
    Serial.println("Check your wiring (SDA->A4, SCL->A5, Power).");
  } else {
    Serial.print("Scan complete: Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s).");
    
    // Check specifically for Hiwonder controller
    boolean foundHiwonder = false;
    Wire.beginTransmission(0x34);
    if (Wire.endTransmission() == 0) {
      foundHiwonder = true;
    }
    
    if (foundHiwonder) {
      Serial.println("✓ Hiwonder Motor Controller detected at 0x34!");
      Serial.println("  Your hardware setup looks correct.");
    } else {
      Serial.println("⚠ Hiwonder Motor Controller NOT found at 0x34");
      Serial.println("  Check controller power and I²C address settings.");
    }
  }
  Serial.println("==================");
}

void loop() {
  // Do nothing. The scan is complete.
} 