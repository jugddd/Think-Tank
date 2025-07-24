#!/usr/bin/env python3
"""
Main Robot Control Application
==============================

This script controls a mobile tracked robot via PS4 controller input.
The robot uses tank-style steering with differential drive motors.

Hardware Configuration:
- Raspberry Pi 5 (primary computer)
- Arduino Uno (real-time motor controller)
- Hiwonder 4-Channel I2C Motor Controller (address 0x34)
- Two DC encoder motors (M1, M2)
- Sony PS4 DualShock 4 controller (Bluetooth)

Author: Robotics Control System
Version: 1.0
"""

import time
import threading
import signal
import sys
from typing import Tuple, Optional
import logging

try:
    import serial
    from pyPS4Controller.controller import Controller
except ImportError as e:
    print(f"ERROR: Missing required packages. Please install with:")
    print("pip install pyserial pyPS4Controller")
    print(f"Import error: {e}")
    sys.exit(1)


# Configuration Constants
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT = 1.0
COMMAND_RATE_HZ = 20  # 20 commands per second
MOTOR_SPEED_MAX = 255
MOTOR_SPEED_MIN = -255

# Controller input scaling factors
DRIVE_SENSITIVITY = 1.0  # Forward/backward sensitivity
TURN_SENSITIVITY = 0.8   # Turning sensitivity
DEADZONE_THRESHOLD = 0.1  # Ignore small stick movements


class PS4ControllerHandler(Controller):
    """
    Custom PS4 controller handler for robot control.
    Processes analog stick inputs for tank-style driving.
    """
    
    def __init__(self):
        """Initialize the controller handler with default values."""
        Controller.__init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False)
        
        # Control state variables
        self.drive_speed = 0.0      # Forward/backward (-1.0 to 1.0)
        self.turn_speed = 0.0       # Left/right turn (-1.0 to 1.0)
        self.emergency_stop = False
        self.controller_connected = False
        
        # Thread safety lock
        self.state_lock = threading.Lock()
        
        print("PS4 Controller handler initialized")
    
    def on_connect(self):
        """Called when controller connects."""
        with self.state_lock:
            self.controller_connected = True
        print("PS4 Controller connected successfully!")
    
    def on_disconnect(self):
        """Called when controller disconnects."""
        with self.state_lock:
            self.controller_connected = False
            self.drive_speed = 0.0
            self.turn_speed = 0.0
        print("PS4 Controller disconnected!")
    
    def on_L3_y_changed(self, value):
        """Left stick Y-axis: Forward/backward movement."""
        # Convert raw value to a normalized float [-1.0, 1.0]
        # The library's Y-axis is inverted (negative is up/forward)
        normalized_value = -value / 32767.0
        
        # Apply deadzone
        if abs(normalized_value) < DEADZONE_THRESHOLD:
            normalized_value = 0.0
            
        with self.state_lock:
            self.drive_speed = normalized_value * DRIVE_SENSITIVITY

    def on_R3_x_changed(self, value):
        """Right stick X-axis: Turning movement."""
        # Convert raw value to a normalized float [-1.0, 1.0]
        normalized_value = value / 32767.0
        
        # Apply deadzone
        if abs(normalized_value) < DEADZONE_THRESHOLD:
            normalized_value = 0.0

        with self.state_lock:
            self.turn_speed = normalized_value * TURN_SENSITIVITY
    
    def on_circle_press(self):
        """Emergency stop button."""
        with self.state_lock:
            self.emergency_stop = True
        print("EMERGENCY STOP activated!")
    
    def on_circle_release(self):
        """Release emergency stop."""
        with self.state_lock:
            self.emergency_stop = False
        print("Emergency stop released")
    
    def get_drive_state(self) -> Tuple[float, float, bool, bool]:
        """
        Get current drive state in thread-safe manner.
        
        Returns:
            Tuple of (drive_speed, turn_speed, emergency_stop, connected)
        """
        with self.state_lock:
            return (
                self.drive_speed,
                self.turn_speed, 
                self.emergency_stop,
                self.controller_connected
            )


class TankDriveMixer:
    """
    Implements tank-style drive mixing algorithm.
    Converts drive/turn inputs to left/right motor speeds.
    """
    
    @staticmethod
    def mix_controls(drive_speed: float, turn_speed: float) -> Tuple[int, int]:
        """
        Convert drive and turn inputs to left/right motor speeds.
        
        Tank drive mixing algorithm:
        - Forward/backward motion affects both motors equally
        - Turning motion adds to one side, subtracts from other
        
        Args:
            drive_speed: Forward/backward speed (-1.0 to 1.0)
            turn_speed: Turn speed (-1.0 to 1.0, negative = left)
        
        Returns:
            Tuple of (left_motor_speed, right_motor_speed) as integers (-255 to 255)
        """
        # Basic tank drive mixing
        left_speed = drive_speed + turn_speed
        right_speed = drive_speed - turn_speed
        
        # Clamp values to [-1.0, 1.0] range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Convert to motor speed integers
        left_motor = int(left_speed * MOTOR_SPEED_MAX)
        right_motor = int(right_speed * MOTOR_SPEED_MAX)
        
        return left_motor, right_motor


class RobotController:
    """
    Main robot controller class.
    Manages serial communication, motor control, and system coordination.
    """
    
    def __init__(self):
        """Initialize the robot controller."""
        self.serial_connection: Optional[serial.Serial] = None
        self.controller: Optional[PS4ControllerHandler] = None
        self.running = False
        self.mixer = TankDriveMixer()
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
        print("Robot Controller initialized")
    
    def setup_serial_connection(self) -> bool:
        """
        Establish serial connection to Arduino.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_connection = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            
            # Wait for Arduino to reset and become ready
            time.sleep(2.0)
            
            # Send initial stop command
            self.send_motor_command(0, 0)
            
            print(f"Serial connection established on {SERIAL_PORT}")
            return True
            
        except serial.SerialException as e:
            print(f"ERROR: Failed to connect to Arduino on {SERIAL_PORT}")
            print(f"Serial error: {e}")
            print("Please check:")
            print("1. Arduino is connected via USB")
            print("2. Arduino is running the motor control sketch")
            print("3. Permissions for /dev/ttyACM0")
            return False
    
    def setup_controller(self) -> bool:
        """
        Initialize PS4 controller in background thread.
        
        Returns:
            True if setup initiated successfully, False otherwise
        """
        try:
            self.controller = PS4ControllerHandler()
            
            # Start controller in background thread
            controller_thread = threading.Thread(
                target=self._controller_thread_worker,
                daemon=True
            )
            controller_thread.start()
            
            print("PS4 Controller setup initiated")
            print("Please press the PS button to connect your controller")
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to setup PS4 controller: {e}")
            print("Please check:")
            print("1. Controller is paired via Bluetooth")
            print("2. pyPS4Controller is properly installed")
            return False
    
    def _controller_thread_worker(self):
        """Worker function for controller background thread."""
        try:
            self.controller.listen(timeout=60)
        except Exception as e:
            self.logger.error(f"Controller thread error: {e}")
    
    def send_motor_command(self, left_speed: int, right_speed: int) -> bool:
        """
        Send motor command to Arduino via serial.
        
        Args:
            left_speed: Left motor speed (-255 to 255)
            right_speed: Right motor speed (-255 to 255)
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.serial_connection:
            return False
        
        try:
            # Format: M,L,R\n
            command = f"M,{left_speed},{right_speed}\n"
            self.serial_connection.write(command.encode('utf-8'))
            self.serial_connection.flush()
            
            # Debug output
            print(f"Sending -> Left: {left_speed:4d}, Right: {right_speed:4d}")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"Serial communication error: {e}")
            return False
    
    def run_vision_processing(self):
        """
        Placeholder for future AI/computer vision processing.
        
        This function will be implemented later to handle:
        - Object detection and tracking
        - Path planning and navigation
        - Obstacle avoidance
        - AI-powered autonomous behaviors
        
        The AI Hat connected to the Pi's GPIO will accelerate
        these vision processing tasks.
        """
        # TODO: Implement vision processing pipeline
        # - Initialize camera feed
        # - Setup AI model inference
        # - Process frames for object detection
        # - Generate navigation commands
        # - Integrate with motor control system
        pass
    
    def main_control_loop(self):
        """
        Main control loop - runs at 20Hz sending motor commands.
        """
        loop_period = 1.0 / COMMAND_RATE_HZ  # 50ms per loop
        
        print(f"Starting main control loop at {COMMAND_RATE_HZ}Hz")
        print("Control mapping:")
        print("  Left stick Y-axis: Forward/Backward")
        print("  Right stick X-axis: Left/Right turn")
        print("  Circle button: Emergency stop")
        print("Press Ctrl+C to exit")
        print("-" * 50)
        
        try:
            while self.running:
                loop_start_time = time.time()
                
                # Get current controller state
                if self.controller:
                    drive_speed, turn_speed, emergency_stop, connected = self.controller.get_drive_state()
                    
                    if emergency_stop or not connected:
                        # Emergency stop or controller disconnected
                        left_motor, right_motor = 0, 0
                        if emergency_stop:
                            print("EMERGENCY STOP - Motors stopped")
                    else:
                        # Normal operation - mix controls
                        left_motor, right_motor = self.mixer.mix_controls(drive_speed, turn_speed)
                else:
                    # No controller available
                    left_motor, right_motor = 0, 0
                
                # Send command to Arduino
                success = self.send_motor_command(left_motor, right_motor)
                if not success:
                    self.logger.warning("Failed to send motor command")
                
                # Placeholder for vision processing
                # self.run_vision_processing()
                
                # Maintain loop timing
                elapsed_time = time.time() - loop_start_time
                sleep_time = max(0, loop_period - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received - shutting down")
        except Exception as e:
            self.logger.error(f"Unexpected error in main loop: {e}")
        finally:
            # Ensure motors are stopped
            self.send_motor_command(0, 0)
            print("Motors stopped")
    
    def shutdown(self):
        """Graceful shutdown procedure."""
        print("Shutting down robot controller...")
        
        self.running = False
        
        # Stop motors
        if self.serial_connection:
            try:
                self.send_motor_command(0, 0)
                time.sleep(0.1)  # Brief delay to ensure command is sent
                self.serial_connection.close()
                print("Serial connection closed")
            except Exception as e:
                self.logger.error(f"Error closing serial connection: {e}")
        
        print("Shutdown complete")
    
    def run(self):
        """
        Main entry point - initialize and run the robot controller.
        """
        print("=" * 60)
        print("Mobile Tracked Robot Control System")
        print("=" * 60)
        
        try:
            # Setup serial connection to Arduino
            if not self.setup_serial_connection():
                return False
            
            # Setup PS4 controller
            if not self.setup_controller():
                return False
            
            # Wait a moment for controller to connect
            print("Waiting for controller connection...")
            time.sleep(3.0)
            
            # Start main control system
            self.running = True
            self.main_control_loop()
            
        except Exception as e:
            self.logger.error(f"Critical error: {e}")
            return False
        finally:
            self.shutdown()
        
        return True


def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown."""
    print(f"\nReceived signal {signum} - initiating shutdown")
    sys.exit(0)


def main():
    """Main application entry point."""
    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run robot controller
    robot = RobotController()
    success = robot.run()
    
    if success:
        print("Robot controller exited normally")
        return 0
    else:
        print("Robot controller exited with errors")
        return 1


if __name__ == "__main__":
    sys.exit(main()) 