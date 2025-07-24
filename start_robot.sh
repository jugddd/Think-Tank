#!/bin/bash

# Mobile Tracked Robot Startup Script
# This script sets up the environment and starts the robot control system

echo "=================================================="
echo "Mobile Tracked Robot Control System - Startup"
echo "=================================================="

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "robot_venv" ]; then
    echo "ERROR: Virtual environment 'robot_venv' not found!"
    echo "Please run the following commands first:"
    echo "  python3 -m venv robot_venv"
    echo "  source robot_venv/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

# Activate virtual environment
echo "Activating Python virtual environment..."
source robot_venv/bin/activate

# Check if required packages are installed
echo "Checking Python dependencies..."
python3 -c "import serial, pyPS4Controller" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ERROR: Required Python packages not installed!"
    echo "Please install with: pip install -r requirements.txt"
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
echo "  Groups: $(groups)"
echo ""

# Start the robot control system
echo "Starting Robot Control System..."
echo "Press Ctrl+C to stop"
echo ""

# Run the main script with error handling
python3 main_robot.py

# Cleanup
echo ""
echo "Robot Control System stopped"
deactivate 