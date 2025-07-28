import pygame
import serial
import time

# --- Configuration ---
# Serial port settings
SERIAL_PORT = '/dev/ttyACM0'  # <-- IMPORTANT: Change this to your Arduino's serial port
BAUD_RATE = 115200

# Game controller settings
# These axis and button numbers might need to be adjusted for your specific controller.
# You can run a simple pygame joystick test script to find the correct numbers.
AXIS_LEFT_STICK_Y = 1  # Throttle
AXIS_RIGHT_STICK_X = 3 # Steering
BUTTON_SWITCH_MODE = 0 # 'A' button on an Xbox controller, for example
BUTTON_STOP = 8        # 'Start' button for emergency stop

# Deadzone for joystick axes to prevent drift
JOYSTICK_DEADZONE = 0.1

# --- Main Application Class ---

class TankController:
    def __init__(self):
        self.arduino = None
        self.joystick = None
        self.ai_mode_enabled = False
        self.running = True
        
        self._initialize_serial()
        self._initialize_pygame_and_joystick()

    def _initialize_serial(self):
        """Establish connection with the Arduino."""
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for the connection to establish
            print("Successfully connected to Arduino.")
        except serial.SerialException as e:
            print(f"Error connecting to Arduino on {SERIAL_PORT}: {e}")
            print("Please check the serial port and ensure the Arduino is connected.")
            self.running = False

    def _initialize_pygame_and_joystick(self):
        """Initialize Pygame and find the first available joystick."""
        if not self.running:
            return
            
        pygame.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No joystick detected. Please connect a controller.")
            self.running = False
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Initialized joystick: {self.joystick.get_name()}")

    def run(self):
        """The main application loop."""
        print("Tank controller is running. Press CTRL+C to exit.")
        while self.running:
            try:
                pygame.event.pump() # Process event queue
                
                # --- Handle button presses ---
                if self.joystick.get_button(BUTTON_STOP):
                    self.send_command("S") # Send stop command
                    print("Emergency Stop Pressed!")
                    time.sleep(0.5) # Prevent rapid-fire stops
                    continue

                if self.joystick.get_button(BUTTON_SWITCH_MODE):
                    self.toggle_ai_mode()
                    time.sleep(0.5) # Debounce the button

                # --- Handle control modes ---
                if self.ai_mode_enabled:
                    self.run_ai_mode()
                else:
                    self.run_manual_mode()
                
                time.sleep(0.05) # Loop at ~20Hz

            except KeyboardInterrupt:
                self.stop()
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                self.stop()
        
        self.cleanup()

    def run_manual_mode(self):
        """Read joystick and send drive commands."""
        # Get throttle value (invert because pygame's Y axis is often inverted)
        throttle = -self.joystick.get_axis(AXIS_LEFT_STICK_Y)
        
        # Get steering value
        steering = self.joystick.get_axis(AXIS_RIGHT_STICK_X)

        # Apply deadzone
        if abs(throttle) < JOYSTICK_DEADZONE:
            throttle = 0.0
        if abs(steering) < JOYSTICK_DEADZONE:
            steering = 0.0
            
        # Format and send the drive command
        drive_command = f"D,{throttle:.2f},{steering:.2f}\n"
        self.send_command(drive_command)

    def run_ai_mode(self):
        """Placeholder for AI-assisted targeting logic."""
        # In a real implementation, this function would:
        # 1. Capture a frame from the camera.
        # 2. Run the object detection/tracking AI.
        # 3. Calculate the required pan/tilt adjustments.
        # 4. Send a 'T,pan,tilt' command to the Arduino.
        print("AI mode is active. (Not implemented yet)")
        # For now, we'll just keep the tank still in AI mode.
        self.send_command("D,0.0,0.0\n")

    def toggle_ai_mode(self):
        """Flip the AI mode flag and provide feedback."""
        self.ai_mode_enabled = not self.ai_mode_enabled
        mode = "AI-Assisted" if self.ai_mode_enabled else "Manual"
        print(f"Switched to {mode} mode.")

    def send_command(self, command: str):
        """Send a command string to the Arduino."""
        if self.arduino and self.arduino.is_open:
            # print(f"Sending: {command.strip()}") # Uncomment for debugging
            self.arduino.write(command.encode('utf-8'))
        else:
            print("Cannot send command, Arduino is not connected.")
            self.running = False
            
    def stop(self):
        """Signal the loop to stop."""
        print("Stopping controller...")
        self.running = False

    def cleanup(self):
        """Clean up resources before exiting."""
        print("Cleaning up resources...")
        if self.arduino and self.arduino.is_open:
            self.send_command("S\n") # Send a final stop command
            self.arduino.close()
            print("Serial connection closed.")
        pygame.quit()
        print("Pygame quit.")

# --- Entry Point ---
if __name__ == "__main__":
    controller = TankController()
    if controller.running:
        controller.run()
    print("Application finished.") 