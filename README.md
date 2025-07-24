# Mobile Tracked Robot Control System

A comprehensive Python application for controlling a tracked robot via PS4 DualShock 4 controller, running on Raspberry Pi with Arduino-based motor control.

## System Architecture

```
┌─────────────────┐    Bluetooth     ┌─────────────────┐
│  PS4 Controller │ ◄─────────────── │   Raspberry Pi  │
└─────────────────┘                  │                 │
                                     │  main_robot.py  │
                                     │                 │
                                     └─────────┬───────┘
                                               │ USB Serial
                                               │ (/dev/ttyACM0)
                                               ▼
                                     ┌─────────────────┐
                                     │   Arduino Uno   │
                                     │  Motor Control  │
                                     │     Sketch      │
                                     └─────────┬───────┘
                                               │ I²C Bus
                                               │ (SDA/SCL)
                                               ▼
                                     ┌─────────────────┐
                                     │   Hiwonder     │
                                     │  4-Ch Motor    │
                                     │  Controller    │
                                     │  (Addr: 0x34)  │
                                     └─────────┬───────┘
                                               │
                                     ┌─────────┴───────┐
                                     ▼                 ▼
                               ┌───────────┐   ┌───────────┐
                               │  Motor M1 │   │  Motor M2 │
                               │   (Left)  │   │  (Right)  │
                               └───────────┘   └───────────┘
```

## Hardware Requirements

- **Raspberry Pi 5** (8GB) with Raspberry Pi OS (64-bit Desktop)
- **Arduino Uno** connected via USB
- **Hiwonder 4-Channel I2C Motor Controller** (I²C address: 0x34)
- **Two DC encoder motors** connected to M1 and M2 channels
- **Sony PS4 DualShock 4 controller** (paired via Bluetooth)
- **AI Hat** (connected to Pi's GPIO for future vision processing)
- **Power system**: USB power bank (logic) + dual-LiPo (motors)

## Software Installation

### 1. Setup Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv robot_venv

# Activate virtual environment
source robot_venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. PS4 Controller Pairing

First, pair your PS4 controller with the Raspberry Pi:

```bash
# Install Bluetooth utilities if not already installed
sudo apt update
sudo apt install bluetooth bluez

# Put controller in pairing mode (hold PS + Share buttons)
# Then scan and pair
bluetoothctl
scan on
pair <controller_mac_address>
trust <controller_mac_address>
connect <controller_mac_address>
exit
```

### 3. Arduino Setup

The Arduino motor control sketch is included in this repository at `arduino/robot_motor_control/robot_motor_control.ino`.

#### Hardware Connections
- **SDA**: Arduino Pin A4 → Hiwonder Controller SDA
- **SCL**: Arduino Pin A5 → Hiwonder Controller SCL
- **Ground**: Connect Arduino GND to Hiwonder Controller GND

#### Upload the Sketch
1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Connect Arduino Uno via USB to your computer (not the Pi initially)
3. Open `arduino/robot_motor_control/robot_motor_control.ino`
4. Select **Tools > Board > Arduino Uno**
5. Select **Tools > Port** (usually `/dev/ttyACM0` on Linux)
6. Click **Upload**

#### Verify Installation
Open **Tools > Serial Monitor** (9600 baud) to see:
```
===========================================
Arduino Motor Controller - READY
Waiting for commands from Raspberry Pi...
===========================================
```

#### Troubleshooting
If you need to verify I²C connectivity, use the included scanner:
- Upload `arduino/i2c_scanner/i2c_scanner.ino`
- Check Serial Monitor for "Found device at 0x34"

See `arduino/README.md` for detailed Arduino setup and troubleshooting guide.

### 4. Permissions Setup

Ensure the Pi user has access to the serial port:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

## Usage

### Running the Robot Controller

```bash
# Activate virtual environment
source robot_venv/bin/activate

# Run the main control script
python3 main_robot.py
```

### Control Mapping

| Input | Function |
|-------|----------|
| **Left Stick Y-axis** | Forward/Backward movement |
| **Right Stick X-axis** | Left/Right turning |
| **Circle Button** | Emergency stop (hold to stop, release to resume) |
| **PS Button** | Connect/wake controller |

### Tank Drive Behavior

The robot uses **tank-style steering**:
- **Forward/Backward**: Both motors run at same speed
- **Turning**: Differential speed between left and right motors
- **Combined movements**: Forward+Turn results in curved motion

**Examples:**
- Left stick up → Both motors forward
- Right stick right → Left motor faster, right motor slower (turn right)
- Left stick up + Right stick right → Forward curve to the right

## System Features

### Real-time Control
- **20Hz command rate** (50ms update intervals)
- **Thread-safe** controller input handling
- **Automatic emergency stop** on controller disconnect

### Safety Features
- Emergency stop button (Circle)
- Automatic motor stop on disconnect
- Graceful shutdown with Ctrl+C
- Input deadzone filtering (prevents drift)

### Debug Output
The system provides real-time feedback:
```
Sending -> Left:  255, Right:  150
Sending -> Left: -128, Right: -128
EMERGENCY STOP - Motors stopped
```

### Future Vision Integration
The code includes placeholder functions for AI/computer vision:
- Object detection and tracking
- Autonomous navigation
- Obstacle avoidance
- Integration with AI Hat hardware acceleration

## Architecture Details

### Classes

1. **`PS4ControllerHandler`**: Manages PS4 controller input in background thread
2. **`TankDriveMixer`**: Implements drive/turn mixing algorithm  
3. **`RobotController`**: Main coordinator for serial communication and control loop

### Communication Protocol

**Pi → Arduino**: Serial commands at 9600 baud
```
Format: M,<left_speed>,<right_speed>\n
Example: M,255,-128\n
Range: -255 to 255 for each motor
```

**Arduino → Motor Controller**: I²C commands to address 0x34

### Configuration

Key parameters can be adjusted in the script:
```python
SERIAL_PORT = "/dev/ttyACM0"        # Arduino USB port
SERIAL_BAUDRATE = 9600              # Serial communication speed
COMMAND_RATE_HZ = 20                # Control loop frequency
DRIVE_SENSITIVITY = 1.0             # Forward/back sensitivity
TURN_SENSITIVITY = 0.8              # Turn sensitivity  
DEADZONE_THRESHOLD = 0.1            # Stick deadzone
```

## Troubleshooting

### Common Issues

**"Failed to connect to Arduino"**
- Check USB connection
- Verify Arduino sketch is uploaded and running
- Check serial port permissions: `ls -l /dev/ttyACM0`

**"Failed to setup PS4 controller"**
- Ensure controller is paired via Bluetooth
- Try re-pairing the controller
- Check `pyPS4Controller` installation

**"Controller disconnected"**
- Controller may have gone to sleep
- Press PS button to reconnect
- Check Bluetooth connection stability

### Testing Serial Connection

```bash
# Test serial port access
ls -l /dev/ttyACM*

# Monitor serial communication
python3 -c "
import serial
s = serial.Serial('/dev/ttyACM0', 9600)
s.write(b'M,0,0\n')
print('Command sent successfully')
s.close()
"
```

### Testing Controller

```bash
# Test controller input
python3 -c "
from pyPS4Controller.controller import Controller
class TestController(Controller):
    def on_x_press(self): print('X pressed')
controller = TestController()
controller.listen()
"
```

## Development Notes

- The code is structured for easy extension with vision processing
- All motor commands go through the tank drive mixer for consistency
- Thread-safe design allows for future multi-threaded features
- Comprehensive error handling and logging

## License

This robot control system is designed for educational and research purposes. 