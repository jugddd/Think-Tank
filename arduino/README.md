# Arduino Motor Control System

This directory contains the Arduino sketch for the mobile tracked robot's real-time motor control system.

## Overview

The Arduino Uno acts as a dedicated real-time motor controller, receiving high-level commands from the Raspberry Pi via USB Serial and translating them into precise I²C commands for the Hiwonder 4-Channel Motor Controller.

## Hardware Setup

### Connections

```
Raspberry Pi ←→ Arduino Uno ←→ Hiwonder Motor Controller ←→ Motors
     USB         I²C (SDA/SCL)                              M1, M2
```

#### Arduino to Hiwonder Controller
- **SDA**: Pin A4 on Arduino Uno → SDA on Hiwonder Controller
- **SCL**: Pin A5 on Arduino Uno → SCL on Hiwonder Controller  
- **GND**: Ground connection between Arduino and Controller
- **VCC**: 5V power (if needed by your specific controller model)

#### Motor Connections
- **M1 Channel**: Left track motor
- **M3 Channel**: Right track motor

## Software Installation

### 1. Arduino IDE Setup

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Connect your Arduino Uno via USB
3. Select **Tools > Board > Arduino Uno**
4. Select **Tools > Port** and choose your Arduino's port (usually `/dev/ttyACM0` on Linux)

### 2. Upload the Sketch

1. Open `robot_motor_control/robot_motor_control.ino` in Arduino IDE
2. Click **Upload** button (or press Ctrl+U)
3. Wait for compilation and upload to complete
4. Open **Tools > Serial Monitor** to verify the sketch is running

### Expected Output
```
===========================================
Arduino Motor Controller - READY
Waiting for commands from Raspberry Pi...
Supported commands:
  M,<left>,<right> - Motor control (-255 to 255)
  E - Emergency stop
  S - Status report
===========================================
Initializing Hiwonder Motor Controller...
Motor Controller initialized successfully
```

## Communication Protocol

### Serial Commands (Pi → Arduino)
- **Baud Rate**: 9600
- **Command Types**:

#### Motor Control Commands
- **Format**: `M,<left_speed>,<right_speed>\n`
- **Speed Range**: -255 to 255
- **Examples**:
  - `M,255,255\n` - Full speed forward
  - `M,-128,-128\n` - Half speed reverse
  - `M,200,-200\n` - Spin turn right
  - `M,0,0\n` - Stop

#### Utility Commands
- **Emergency Stop**: `E\n` - Immediately stops all motors
- **Status Report**: `S\n` - Requests current motor status and timing info

### I²C Commands (Arduino → Hiwonder Controller)
- **Address**: 0x34
- **Protocol**: Register-based communication
- **Registers**:
  - `0x00` - Motor 1 Speed (0-255) [M1 Channel]
  - `0x01` - Motor 1 Direction (0=Stop, 1=Forward, 2=Reverse) [M1]
  - `0x04` - Motor 3 Speed (0-255) [M3 Channel]
  - `0x05` - Motor 3 Direction (0=Stop, 1=Forward, 2=Reverse) [M3]
  - `0x08` - Motor Enable (1=Enable, 0=Disable)

## Safety Features

### Timeout Protection
- **Automatic Stop**: Motors stop if no commands received for 1000ms
- **Safety Message**: Prints timeout warning to serial monitor

### Error Handling
- **I²C Error Detection**: Reports failed I²C transmissions
- **Command Validation**: Validates incoming serial command format
- **Speed Limiting**: Constrains speeds to valid range (-255 to 255)

### Emergency Stop
- Call `emergencyStop()` function to immediately stop all motors
- All motors set to 0 speed and STOP direction

## Configuration

### Motor Controller Address
If your Hiwonder controller uses a different I²C address, modify:
```cpp
const int MOTOR_CONTROLLER_ADDRESS = 0x34;  // Change this value
```

### Motor Channel Mapping
To swap left/right motors, modify the register assignments:
```cpp
const byte MOTOR1_SPEED_REG = 0x02;     // Swap these values
const byte MOTOR1_DIR_REG = 0x03;       // to reverse motor mapping
const byte MOTOR2_SPEED_REG = 0x00;
const byte MOTOR2_DIR_REG = 0x01;
```

### Command Timeout
To adjust the safety timeout period:
```cpp
const int COMMAND_TIMEOUT_MS = 1000;    // Milliseconds before auto-stop
```

## Testing

### 1. Serial Communication Test
Open Arduino IDE Serial Monitor and manually send commands:
```
M,100,100    (should move both motors forward)
M,-100,-100  (should move both motors reverse)
M,0,0        (should stop motors)
E            (emergency stop - immediate motor stop)
S            (status report - shows current motor speeds and timing)
```

### 2. I²C Scanner
Use this simple sketch to verify Hiwonder controller is detected:

```cpp
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I²C Scanner");
}

void loop() {
  for(byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
  delay(5000);
}
```

Expected output should show: `Found device at 0x34`

## Troubleshooting

### Motor Not Moving
1. **Check Power**: Ensure motor power supply (dual-LiPo) is connected to Hiwonder controller
2. **Check I²C**: Run I²C scanner to verify controller responds at address 0x34
3. **Check Wiring**: Verify SDA/SCL connections between Arduino and controller
4. **Check Motor Connections**: Ensure motors are properly connected to M1/M2 channels

### I²C Errors
```
I²C ERROR: Failed to write to register 0x00, Error code: 2
```
- **Error Code 2**: Address not acknowledged (controller not found)
- **Error Code 3**: Data not acknowledged (invalid register)
- **Error Code 4**: Other error

**Solutions**:
- Verify I²C wiring (SDA/SCL)
- Check controller power
- Verify controller address with I²C scanner
- Check for loose connections

### Serial Communication Issues
- **No response**: Check USB cable and port selection in Arduino IDE
- **Garbled output**: Verify baud rate is set to 9600 in Serial Monitor
- **Command errors**: Ensure commands end with newline character (`\n`)

### Motor Behavior Issues
- **Motors run opposite direction**: Swap motor wires or modify direction constants
- **Weak movement**: Check motor power supply voltage and current capacity
- **Erratic behavior**: Add capacitors near motor terminals to reduce electrical noise

## Advanced Configuration

### Custom Motor Controller
If using a different motor controller, modify the register map:
```cpp
const byte MOTOR1_SPEED_REG = 0x00;     // Update these values
const byte MOTOR1_DIR_REG = 0x01;       // based on your controller's
const byte MOTOR2_SPEED_REG = 0x02;     // datasheet
const byte MOTOR2_DIR_REG = 0x03;
```

### Performance Tuning
- **I²C Speed**: Adjust `Wire.setClock()` for faster communication
- **Command Rate**: Modify delay in main loop for different update rates
- **Debouncing**: Add filtering for noisy serial input

## Version History

- **v1.1**: 
  - Fixed critical command parsing bug in motor control
  - Added emergency stop (E) and status report (S) commands
  - Simplified serial input handling using `Serial.readStringUntil()`
  - Removed unnecessary delays for improved responsiveness
  - I²C scanner now runs once instead of continuously
- **v1.0**: Initial release with basic motor control and safety features

## Support

For issues specific to the Arduino code, check:
1. Serial Monitor output for error messages
2. I²C scanner results
3. Motor controller datasheet for register specifications
4. Arduino Uno pin connections and power supply 