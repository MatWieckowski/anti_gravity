LK Motor Anti-Gravity Control via CAN Bus
This script provides a ready-to-use solution for implementing anti-gravity compensation on an LK Motor using Python and a CANable adapter. It actively counteracts gravitational forces, making a connected payload feel "weightless."

This README will guide you through the setup, configuration, and operation of the script.

Features
🦾 Physics-Based Control
The script implements a real-time control loop that uses the motor's angle to calculate and negate gravitational torque.

Calculates Torque: Uses the formula 
tau=m
cdotg
cdotl
cdot
sin(
theta) to determine the required counter-torque.

Sends Commands: Transmits torque commands directly to the motor using the Torque Closed-Loop Control mode (0xA1).

⚙️ CAN Bus Communication
All communication with the motor is handled via the CAN bus, using the python-can library.

High-Level Functions: Simple functions like motor_on(), motor_off(), and read_encoder_angle() abstract away the low-level CAN frame construction.

Robust I/O: The send_and_receive_command function ensures that commands are sent and corresponding acknowledgments are received from the motor.

🔧 Full Configuration
No hard-coded values. Easily adjust all key parameters at the top of the script to match your specific hardware setup. See the Configuration section for details.

How to use this script
Prerequisites
Hardware

An LK Motor with a CAN bus interface (e.g., MG5210).

A CANable USB to CAN adapter (or other slcan compatible device).

An appropriate power supply for your motor.

A computer to run the script.

Software & Libraries

Python 3.x

The python-can library.

Bash

pip install python-can
For Windows users, the Zadig driver may be needed to install WinUSB for the CANable.

🛠️ Configuration
Before running, you must edit the constants in the script to match your setup. The key parameters are located in the "Konfiguracja" (Configuration) and "Stałe fizyczne" (Physical constants) sections.

Python

#Konfiguracja
CANABLE_PORT = 'COM7'  # <-- Set your COM port (e.g., '/dev/ttyACMx' on Linux)
CAN_BITRATE = 1000000  # <-- Set your CAN bus bitrate
MOTOR_ID = 4           # <-- Set your motor's CAN ID

#Stałe fizyczne
MASS_KG = 0.08         # <-- Set the mass of your payload in kg
DISTANCE_TO_CENTER_OF_MASS_M = 0.05 # <-- Set the distance to the center of mass in meters
▶️ Execution
Connect your hardware securely (Motor, Power Supply, CANable).

Plug the CANable into your computer.

Run the script from your terminal:

Bash

python your_script_name.py
The script will initialize the CAN bus, turn the motor on, and begin the anti-gravity control loop. You will see the motor's angle printed to the console in real-time.

To stop the program, press Ctrl+C. This will safely turn the motor off and shut down the CAN bus connection.

⚠️ Disclaimer
Working with motors involves risk. Incorrect wiring, high currents, or unexpected motor movement can cause damage to your hardware or result in injury. Always secure your setup and operate it with caution. The author is not responsible for any damages. Use at your own risk.
