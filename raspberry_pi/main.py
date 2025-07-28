import pygame
import serial
import time

# ==============================================================================
# --- CONFIGURATION ---
# ==============================================================================

# -- Serial Communication --
SERIAL_PORT = '/dev/ttyACM0'  # <-- IMPORTANT: Change to your Arduino's serial port
BAUD_RATE = 115200

# -- Game Controller Mappings (Verified for a PS4-style controller) --
# Use a joystick test script to find these values if you use a different controller.
AXIS_LEFT_STICK_Y = 1   # Axis for the left track's forward/backward motion
AXIS_RIGHT_STICK_Y = 4  # Axis for the right track's forward/backward motion
BUTTON_SWITCH_MODE = 0  # A button to toggle between manual and AI modes
BUTTON_STOP = 8         # An emergency stop button (e.g., Start button)

# -- Control Tuning --
JOYSTICK_DEADZONE = 0.1 # Increase if your controller drifts when centered

# ==============================================================================
# --- Main Application Class ---
# ==============================================================================

class TankController:
    """ Manages joystick input, AI modes, and serial communication to the Arduino. """
    def __init__(self):
        self.arduino = None
        self.joystick = None
        self.ai_mode_enabled = False
        self.is_running = True
        
        self._initialize_serial()
        self._initialize_pygame_and_joystick()

    def _initialize_serial(self):
        """ Establishes the serial connection with the Arduino. """
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for the Arduino to reset and establish connection
            print(f"Successfully connected to Arduino on {SERIAL_PORT}")
        except serial.SerialException as e:
            print(f"FATAL: Could not connect to Arduino on {SERIAL_PORT}: {e}")
            self.is_running = False

    def _initialize_pygame_and_joystick(self):
        """ Initializes Pygame and finds the first available joystick. """
        if not self.is_running:
            return
            
        pygame.init()
        if pygame.joystick.get_count() == 0:
            print("FATAL: No joystick detected. Please connect a controller.")
            self.is_running = False
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Initialized joystick: {self.joystick.get_name()}")

    def run_main_loop(self):
        """ The main application loop. Handles events and control modes. """
        print("Tank controller is running. Press Ctrl+C to exit.")
        while self.is_running:
            try:
                pygame.event.pump()  # Must be called to process internal events
                
                # --- Handle button presses for system functions ---
                if self.joystick.get_button(BUTTON_STOP):
                    print("Emergency Stop button pressed!")
                    self.send_command("S\n") # Send immediate stop command
                    time.sleep(0.5) # Debounce to prevent rapid stops
                    continue

                if self.joystick.get_button(BUTTON_SWITCH_MODE):
                    self.toggle_ai_mode()
                    time.sleep(0.5) # Debounce to prevent rapid toggling

                # --- Execute the current control mode ---
                if self.ai_mode_enabled:
                    self.run_ai_mode()
                else:
                    self.run_manual_mode()
                
                time.sleep(0.05) # Loop at ~20Hz to prevent flooding the serial port

            except KeyboardInterrupt:
                self.stop()
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                self.stop()
        
        self.cleanup()

    def run_manual_mode(self):
        """ Reads joystick axes and sends direct left/right speed commands. """
        left_speed = -self.joystick.get_axis(AXIS_LEFT_STICK_Y)
        right_speed = -self.joystick.get_axis(AXIS_RIGHT_STICK_Y)

        if abs(left_speed) < JOYSTICK_DEADZONE:
            left_speed = 0.0
        if abs(right_speed) < JOYSTICK_DEADZONE:
            right_speed = 0.0
            
        drive_command = f"D,{left_speed:.2f},{right_speed:.2f}\n"
        self.send_command(drive_command)

    def run_ai_mode(self):
        """ Placeholder for AI-assisted targeting logic. """
        print("AI mode active (not implemented).")
        # For safety, ensure the robot is stopped when switching to this mode.
        self.send_command("D,0.0,0.0\n")

    def toggle_ai_mode(self):
        """ Flips the AI mode flag and provides user feedback. """
        self.ai_mode_enabled = not self.ai_mode_enabled
        mode = "AI-Assisted" if self.ai_mode_enabled else "Manual"
        print(f"Switched to {mode} mode.")

    def send_command(self, command: str):
        """ Sends a command string to the Arduino, encoded in UTF-8. """
        if self.arduino and self.arduino.is_open:
            self.arduino.write(command.encode('utf-8'))
        else:
            print("Warning: Cannot send command, Arduino is not connected.")
            self.is_running = False
            
    def stop(self):
        """ Signals the main loop to terminate. """
        self.is_running = False

    def cleanup(self):
        """ Cleans up resources (serial port, pygame) before exiting. """
        print("Stopping controller and cleaning up resources...")
        if self.arduino and self.arduino.is_open:
            self.send_command("S\n") # Send a final stop command for safety
            self.arduino.close()
            print("Serial connection closed.")
        pygame.quit()
        print("Pygame quit.")

# ==============================================================================
# --- Application Entry Point ---
# ==============================================================================
if __name__ == "__main__":
    controller = TankController()
    if controller.is_running:
        controller.run_main_loop()
    print("Application finished.") 