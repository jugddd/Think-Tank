#!/usr/bin/env python3
"""
Robot Communication Test Script
===============================

This script tests the complete communication chain:
Pi Python → Arduino → Hiwonder Controller → Motors

No PS4 controller required - sends predefined test commands.
"""

import serial
import time
import sys

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 9600

def test_robot_communication():
    """Test the complete robot communication chain."""
    
    print("=" * 50)
    print("Robot Communication Test")
    print("=" * 50)
    
    try:
        # Connect to Arduino
        print(f"Connecting to Arduino on {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=2)
        time.sleep(2)  # Wait for Arduino to reset
        
        print("✓ Arduino connection established")
        
        # Test sequence
        test_commands = [
            ("M,0,0", "Stop motors"),
            ("S", "Status request"),
            ("M,100,100", "Forward motion"),
            ("M,-50,-50", "Reverse motion"),
            ("M,80,-80", "Turn right"), 
            ("M,-80,80", "Turn left"),
            ("E", "Emergency stop"),
            ("M,0,0", "Final stop")
        ]
        
        print("\nRunning test sequence...")
        print("-" * 30)
        
        for i, (command, description) in enumerate(test_commands, 1):
            print(f"Test {i}: {description}")
            print(f"  Sending: {command}")
            
            # Send command
            ser.write((command + '\n').encode())
            ser.flush()
            
            # Wait for Arduino response
            time.sleep(0.5)
            
            # Read any response from Arduino
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    print(f"  Arduino: {response}")
            
            # Delay between commands
            time.sleep(1.5)
            print()
        
        # Final status check
        print("Final status check...")
        ser.write(b'S\n')
        ser.flush()
        time.sleep(0.5)
        
        while ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response:
                print(f"Final status: {response}")
        
        ser.close()
        print("✓ Test completed successfully!")
        print("\nIf you saw 'Command received' messages without I²C errors,")
        print("your robot communication system is working perfectly!")
        
        return True
        
    except serial.SerialException as e:
        print(f"✗ Serial connection failed: {e}")
        print("\nCheck:")
        print("1. Arduino connected via USB")
        print("2. Motor control sketch uploaded")
        print("3. Correct port permissions")
        return False
    
    except KeyboardInterrupt:
        print("\n✗ Test interrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.write(b'M,0,0\n')  # Stop motors
            ser.close()
        return False
    
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    success = test_robot_communication()
    sys.exit(0 if success else 1) 