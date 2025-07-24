#!/usr/bin/env python3
"""
Continuous Robot Command Test
=============================

This script sends continuous commands to prevent Arduino timeout.
Perfect for testing when motors aren't powered but you want to verify
the complete communication chain is working.
"""

import serial
import time
import sys

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 9600

def continuous_test():
    """Send continuous commands to test robot without timeout."""
    
    print("=" * 60)
    print("Continuous Robot Communication Test")
    print("Motors don't need to be powered - testing communication only")
    print("=" * 60)
    
    try:
        # Connect to Arduino
        print(f"Connecting to Arduino on {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        
        print("✓ Arduino connection established")
        print("\nSending continuous commands (Ctrl+C to stop)...")
        print("-" * 40)
        
        # Test sequence that repeats
        test_sequence = [
            ("M,0,0", "Stop"),
            ("M,50,50", "Slow forward"),
            ("M,0,0", "Stop"),  
            ("M,-30,-30", "Slow reverse"),
            ("M,0,0", "Stop"),
            ("M,40,-40", "Turn right"),
            ("M,0,0", "Stop"),
            ("M,-40,40", "Turn left"),
            ("M,0,0", "Stop"),
        ]
        
        command_count = 0
        cycle_count = 0
        
        while True:
            for command, description in test_sequence:
                command_count += 1
                
                print(f"[{command_count:3d}] {description:12} -> {command}")
                
                # Send command
                ser.write((command + '\n').encode())
                ser.flush()
                
                # Read Arduino responses
                time.sleep(0.2)  # Give Arduino time to respond
                while ser.in_waiting > 0:
                    response = ser.readline().decode().strip()
                    if response:
                        if "Command received" in response:
                            print(f"      ✓ {response}")
                        elif "ERROR" in response or "TIMEOUT" in response:
                            print(f"      ⚠ {response}")
                        else:
                            print(f"      • {response}")
                
                # Wait between commands (but not too long to avoid timeout)
                time.sleep(0.8)  # Total cycle time: 1 second (well under timeout)
            
            cycle_count += 1
            print(f"\n--- Completed cycle {cycle_count} ({command_count} total commands) ---\n")
            
            # Brief pause between cycles
            time.sleep(1)
    
    except serial.SerialException as e:
        print(f"\n✗ Serial connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check Arduino USB connection")
        print("2. Verify motor control sketch is uploaded")
        print("3. Ensure no other programs are using the serial port")
        return False
    
    except KeyboardInterrupt:
        print(f"\n\n✓ Test stopped by user after {command_count} commands")
        print("Sending final stop command...")
        try:
            ser.write(b'M,0,0\n')
            ser.flush()
            time.sleep(0.1)
            ser.close()
        except:
            pass
        return True
    
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        return False

def quick_test():
    """Quick test to verify basic communication."""
    
    print("Running quick communication test...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=2)
        time.sleep(2)
        
        # Send a few test commands
        test_cmds = ["M,0,0", "S", "E", "M,10,10", "M,0,0"]
        
        for cmd in test_cmds:
            print(f"Sending: {cmd}")
            ser.write((cmd + '\n').encode())
            ser.flush()
            time.sleep(0.5)
            
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    print(f"  Response: {response}")
        
        ser.close()
        print("✓ Quick test completed")
        return True
        
    except Exception as e:
        print(f"✗ Quick test failed: {e}")
        return False

if __name__ == "__main__":
    print("Choose test mode:")
    print("1. Quick test (few commands)")
    print("2. Continuous test (runs until Ctrl+C)")
    
    try:
        choice = input("\nEnter choice (1 or 2): ").strip()
        
        if choice == "1":
            success = quick_test()
        elif choice == "2":
            success = continuous_test()
        else:
            print("Invalid choice. Running quick test...")
            success = quick_test()
            
    except KeyboardInterrupt:
        print("\nExiting...")
        success = True
    
    sys.exit(0 if success else 1) 