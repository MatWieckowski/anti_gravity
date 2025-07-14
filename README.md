# BLDC Motor Control with Encoder via CAN Bus

This project implements control of a Brushless DC (BLDC) motor equipped with an encoder, communicating via the CAN bus. The primary goal is to maintain the output arm's angle within a defined "hold zone" by compensating for gravitational torque and applying damping for stabilization.

---

## Table of Contents

-   [Introduction](#introduction)
-   [Features](#features)
-   [Requirements](#requirements)
-   [Installation and Setup](#installation-and-setup)
-   [Configuration](#configuration)
-   [Data Logging (CSV)](#data-logging-csv)
-   [Code Structure](#code-structure)
-   [Author](#author)
-   [License](#license)

---

## Introduction

This project demonstrates a simple control algorithm for an arm's position, utilizing CAN communication to interact with a BLDC motor. It focuses on keeping the arm within a specific angular range, with active compensation for gravitational torque and motion stabilization. Additionally, all key data is logged to a CSV file for later analysis.

---

## Features

* **Motor Torque Control:** Commands motor torque via CAN.
* **Encoder Angle Reading:** Retrieves the current motor position from the encoder.
* **Gravity Compensation:** Calculates and applies torque to offset the effect of gravity on the arm.
* **Motion Damping:** Dynamically dampens arm oscillations.
* **Hold Zone:** A special mode where the motor applies zero torque when the arm is within a specified angular range and its velocity is low.
* **CSV Data Logging:** Automatically saves key parameters (angle, torque, velocity) to a CSV file for analysis and debugging.

---

## Requirements

* **Hardware:**
    * BLDC motor with encoder (compatible with the communication protocol).
    * CAN interface (e.g., USB to CAN adapter, like slcan).
    * Appropriate power supply for the motor.
* **Software:**
    * Python 3.x
    * `python-can` library: `pip install python-can`
    * `pyserial` library (if using an slcan interface): `pip install pyserial`

---

## Installation and Setup

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YourGitHubUsername/YourRepositoryName.git](https://github.com/YourGitHubUsername/YourRepositoryName.git)
    cd YourRepositoryName
    ```
2.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
    (Create a `requirements.txt` file with the content: `python-can`, `pyserial`)
3.  **Connect CAN interface and motor:** Ensure your CAN interface is properly connected to your computer and the motor is connected to the interface.
4.  **Configure parameters:** Before running, adjust the constants in the `--- Konfiguracja ---` section (or `--- Configuration ---`) in your main script file (e.g., `main.py`) to match your hardware and requirements (e.g., `CAN_INTERFACE`, `MOTOR_ID`, `REDUCTION_RATIO`, `ANGLE_OFFSET_DEGREES`).
5.  **Run the script:**
    ```bash
    python your_main_script.py
    ```

---

## Configuration

In the code file, you'll find a `--- Konfiguracja ---` (or `--- Configuration ---`) section where you can adjust the following parameters:

* **`CAN_INTERFACE`**: The serial port your CAN adapter is connected to (e.g., `'COM7'` for Windows, `'/dev/ttyUSB0'` for Linux).
* **`CAN_BITRATE`**: The CAN bus bitrate (default `1000000`).
* **`MOTOR_ID`**: The motor's ID on the CAN bus.
* **`REDUCTION_RATIO`**: Gear reduction ratio, if the arm is not directly on the motor's axis.
* **`ANGLE_OFFSET_DEGREES`**: Encoder angle offset in degrees, used for calibration.
* **`SMOOTHING_FACTOR`**: Smoothing factor for the output torque.
* **`D_REG`**: Damping coefficient.
* **`HOLD_ZONE_MIN_OUTPUT_ANGLE`**, **`HOLD_ZONE_MAX_OUTPUT_ANGLE`**: Minimum and maximum output angles for the hold zone in degrees.
* **`SETTLE_VELOCITY_THRESHOLD_RPS`**: Velocity threshold (in radians per second) below which the arm is considered "settled" in the hold zone.
* **`MASS_KG`**: Mass of the arm in kilograms.
* **`DISTANCE_TO_CENTER_OF_MASS_M`**: Distance from the arm's pivot point to its center of mass in meters.
* **`TORQUE_LIMIT_NM`**: Maximum allowed output torque in Newton-meters.

---

## Data Logging (CSV)

The script automatically logs measurement and control data to a file named `motor_data.csv` in the same directory where the script is executed. The file contains the following columns:

* `Timestamp`: Unique time for each record.
* `Output Angle [deg]`: The arm's output angle in degrees.
* `Motor Angle [deg]`: The raw motor encoder angle reading in degrees.
* `Calibrated Motor Angle [deg]`: The motor angle after offset subtraction.
* `Angular Velocity [rad/s]`: The arm's angular velocity in radians per second.
* `Target Torque [Nm]`: The target torque calculated by the algorithm.
* `Current Torque [Nm]`: The currently commanded torque after smoothing.
* `Is Holding`: A boolean value (`True`/`False`) indicating whether the arm is in the "hold zone" mode.

---

## Code Structure

* **Configuration Constants:** Defined at the beginning of the script for easy modification.
* **`motor_on_off(state)`:** Function to turn the motor on or off.
* **`set_torque_and_get_angle(torque_nm)`:** Function that sends the commanded torque to the motor and receives the current encoder angle. It handles the conversion of torque to raw IQ values and vice versa for the angle.
* **`main_control_loop()`:** The main control loop. It is responsible for:
    * Cyclically reading the angle.
    * Calculating angular velocity.
    * Detecting the "hold zone."
    * Calculating gravity compensation and damping torques.
    * Smoothing the commanded torque.
    * Logging data to a CSV file.
    * Handling `KeyboardInterrupt` (`Ctrl+C`) and safely shutting down the motor.

---

## Author

[MatWieckowski]
---