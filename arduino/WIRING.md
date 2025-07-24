# Hardware Wiring Guide

This document provides detailed wiring instructions for connecting the Arduino Uno to the Hiwonder 4-Channel Motor Controller and the complete robot system.

## System Overview

```
┌─────────────────┐    USB Cable      ┌─────────────────┐
│  Raspberry Pi   │ ←──────────────── │   Arduino Uno   │
│                 │                   │                 │
│  main_robot.py  │                   │ Motor Control   │
│                 │                   │     Sketch      │
└─────────────────┘                   └─────────┬───────┘
                                                │ I²C
                                                │ (SDA/SCL)
                                                ▼
                                      ┌─────────────────┐
                                      │   Hiwonder     │
                                      │  4-Channel     │
                                      │ Motor Controller│
                                      │   (Addr: 0x34) │
                                      └─────────┬───────┘
                                                │ Motor Outputs
                                      ┌─────────┴───────┐
                                      ▼                 ▼
                                ┌───────────┐     ┌───────────┐
                                │ Motor M1  │     │ Motor M2  │
                                │  (Left)   │     │ (Right)   │
                                └───────────┘     └───────────┘
```

## Arduino Uno Pin Connections

### I²C Connection to Hiwonder Controller

| Arduino Pin | Function | Hiwonder Controller |
|-------------|----------|---------------------|
| A4          | SDA      | SDA                |
| A5          | SCL      | SCL                |
| GND         | Ground   | GND                |
| 5V          | Power*   | VCC (if needed)    |

**Note**: Some Hiwonder controllers are powered separately. Check your specific model's requirements.

### USB Connection to Raspberry Pi

| Arduino Pin | Function | Raspberry Pi |
|-------------|----------|--------------|
| USB Port    | Power + Data | USB Port (/dev/ttyACM0) |

## Detailed Wiring Diagram

```
Arduino Uno                    Hiwonder 4-Ch Controller
┌─────────────┐               ┌─────────────────────┐
│             │               │                     │
│         A4  │──────SDA──────│ SDA                 │
│         A5  │──────SCL──────│ SCL                 │
│        GND  │──────GND──────│ GND                 │
│         5V  │──────VCC──────│ VCC (if required)   │
│             │               │                     │
│    USB Port │               │   M1   M2   M3   M4 │
└──────┬──────┘               └─────┬───┬───────────┘
       │                            │   │
       │ USB Cable                  │   │ Motor Cables
       │                            │   │
┌──────▼──────┐               ┌─────▼───▼─────┐
│             │               │               │
│ Raspberry   │               │  Left  Right  │
│    Pi       │               │ Motor  Motor  │
│             │               │               │
└─────────────┘               └───────────────┘
```

## Power System Wiring

### Logic Power (5V)
- Raspberry Pi powered by USB power bank
- Arduino powered via USB from Raspberry Pi
- Hiwonder controller logic powered by Arduino 5V (if needed)

### Motor Power (High Current)
- Hiwonder controller motor power (PWR terminal) connected to dual-LiPo battery
- **IMPORTANT**: Motor power and logic power are separate systems

```
Power Distribution:
┌─────────────┐    5V USB     ┌─────────────┐    5V (if needed)    ┌─────────────┐
│ USB Power   │──────────────→│ Raspberry   │──────────────────────→│ Hiwonder    │
│    Bank     │               │     Pi      │                      │ Controller  │
└─────────────┘               └─────────────┘                      │   (Logic)   │
                                      │                            └─────────────┘
                               USB 5V │                                     ▲
                                      ▼                                     │
                              ┌─────────────┐                              │ PWR
                              │   Arduino   │──────────I²C──────────────────┘
                              │     Uno     │
                              └─────────────┘

┌─────────────┐    High Current
│ Dual-LiPo   │──────────────────────────────────────────────────────────────┐
│  Battery    │                                                              │
│   System    │                                                              ▼
└─────────────┘                                                    ┌─────────────┐
                                                                   │ Hiwonder    │
                                                                   │ Controller  │
                                                                   │ (Motor PWR) │
                                                                   └─────┬───────┘
                                                                         │
                                                               ┌─────────┴────────┐
                                                               ▼                  ▼
                                                         ┌──────────┐     ┌──────────┐
                                                         │ Left     │     │ Right    │
                                                         │ Motor    │     │ Motor    │
                                                         └──────────┘     └──────────┘
```

## Connection Steps

### Step 1: Arduino to Hiwonder Controller
1. **Power off** all systems
2. Connect SDA: Arduino A4 → Hiwonder SDA
3. Connect SCL: Arduino A5 → Hiwonder SCL  
4. Connect GND: Arduino GND → Hiwonder GND
5. Connect VCC: Arduino 5V → Hiwonder VCC (only if your controller needs 5V logic power)

### Step 2: Motors to Hiwonder Controller
1. Connect left motor to M1 channel (red to +, black to -)
2. Connect right motor to M2 channel (red to +, black to -)
3. Ensure motor polarity is consistent

### Step 3: Power Connections
1. Connect dual-LiPo battery to Hiwonder PWR terminals
2. Connect USB power bank to Raspberry Pi
3. Connect Arduino to Raspberry Pi via USB cable

### Step 4: Verification
1. Power on USB power bank (logic power)
2. Power on dual-LiPo system (motor power)
3. Check Arduino IDE Serial Monitor for startup messages
4. Run I²C scanner to verify controller detection

## Safety Considerations

### Power Safety
- **Never connect motor power to logic circuits**
- Use separate power supplies for logic (5V) and motors (LiPo voltage)
- Add fuses to motor power lines for protection
- Ensure proper battery management for LiPo system

### Wiring Safety
- Double-check all connections before powering on
- Use proper gauge wire for motor currents
- Secure all connections to prevent shorts
- Add strain relief to USB and motor cables

### Testing Safety
- Start with motors disconnected
- Test I²C communication first
- Verify motor direction before full operation
- Keep emergency stop easily accessible

## Troubleshooting

### No I²C Communication
- Check SDA/SCL pin connections (A4/A5 on Uno)
- Verify ground connection between Arduino and controller
- Check controller power supply
- Run I²C scanner sketch to detect devices

### Motors Not Responding
- Verify motor power supply is connected and charged
- Check motor wire connections to M1/M2 channels
- Ensure controller is receiving valid I²C commands
- Test with simple Arduino sketch sending known-good commands

### Erratic Behavior
- Check for loose connections
- Add capacitors across motor terminals (100nF ceramic + 100µF electrolytic)
- Verify adequate power supply current capacity
- Check for electromagnetic interference from motors

## Wire Specifications

### I²C Lines (SDA/SCL)
- **Wire Type**: 22-24 AWG hookup wire
- **Length**: Keep under 1 meter for reliable communication
- **Shielding**: Not required for short runs

### Motor Power Lines
- **Wire Type**: Stranded copper, rated for motor current
- **Gauge**: 16-18 AWG for typical robot motors
- **Length**: Minimize to reduce voltage drop
- **Connectors**: Secure screw terminals or high-current connectors

### Logic Power (5V)
- **Wire Type**: 22-24 AWG
- **Current**: Adequate for Arduino + controller logic (~200mA)
- **Protection**: Inline fuse recommended (500mA) 