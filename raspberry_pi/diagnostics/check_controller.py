import pygame
import time

# A simple script to detect and print joystick events.
# Helps find the correct axis and button numbers for your specific controller.

def run_controller_check():
    """Initializes a joystick and prints its events to the console."""
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("Error: No joystick detected. Please connect a controller.")
        return

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print("--- Controller Diagnostic Tool ---")
        print(f"Joystick Name: {joystick.get_name()}")
        print(f"Number of Axes: {joystick.get_numaxes()}")
        print(f"Number of Buttons: {joystick.get_numbuttons()}")
        print("\nMove the sticks and press buttons to identify their numbers.")
        print("Press Ctrl+C to exit.")

        # Store last printed values to reduce spam
        last_axis_values = [0.0] * joystick.get_numaxes()
        last_button_states = [0] * joystick.get_numbuttons()

        while True:
            pygame.event.pump()

            # Check axes
            for i in range(joystick.get_numaxes()):
                axis_val = joystick.get_axis(i)
                # Print only if the value has changed significantly
                if abs(axis_val - last_axis_values[i]) > 0.1:
                    print(f"Axis {i} value: {axis_val:>6.3f}")
                    last_axis_values[i] = axis_val

            # Check buttons
            for i in range(joystick.get_numbuttons()):
                button_val = joystick.get_button(i)
                if button_val != last_button_states[i]:
                    state = "Pressed" if button_val else "Released"
                    print(f"Button {i}: {state}")
                    last_button_states[i] = button_val

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting diagnostic tool.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    run_controller_check() 