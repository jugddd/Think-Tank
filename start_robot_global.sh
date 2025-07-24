#!/bin/bash

# Mobile Tracked Robot Startup Script (Global Python Installation)
# This script starts the robot control system using system-wide Python packages

echo "=================================================="
echo "Mobile Tracked Robot Control System - Startup"
echo "=================================================="

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if required packages are installed globally
echo "Checking Python dependencies..."
python3 -c "import serial, pyPS4Controller" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ERROR: Required Python packages not installed!"
    echo "Please install with:"
    echo "  pip3 install pyserial pyPS4Controller"
    echo "Or use the venv version: ./start_robot.sh"
    exit 1
fi

# Check if Arduino is connected
echo "Checking Arduino connection..."
if [ ! -e "/dev/ttyACM0" ]; then
    echo "WARNING: Arduino not found at /dev/ttyACM0"
    echo "Please check:"
    echo "  1. Arduino is connected via USB"
    echo "  2. Arduino is running the motor control sketch"
    echo "  3. User permissions for serial port access"
    echo ""
    echo "Continuing anyway (you can connect Arduino later)..."
fi

# Check if user has serial port permissions
if ! groups $USER | grep -q dialout; then
    echo "WARNING: User not in 'dialout' group"
    echo "Add with: sudo usermod -a -G dialout $USER"
    echo "Then log out and back in"
fi

# Display system information
echo ""
echo "System Information:"
echo "  Python: $(python3 --version)"
echo "  Working Directory: $SCRIPT_DIR"
echo "  User: $USER"
echo "  Installation: Global (no venv)"
echo ""

# Start the robot control system
echo "Starting Robot Control System..."
echo "Press Ctrl+C to stop"
echo ""

# Run the main script
python3 main_robot.py

echo ""
echo "Robot Control System stopped" 