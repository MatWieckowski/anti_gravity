import can
import time
import math
import struct
import csv
import os

CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
REDUCTION_RATIO = 6.0
ANGLE_OFFSET_DEGREES = 240.07

SMOOTHING_FACTOR = 0.8

ESTIMATED_COULOMB_FRICTION_IQ = 1.7821
ESTIMATED_VISCOUS_FRICTION_COEFFICIENT_IQ = 0.0002
FRICTION_COMP_VELOCITY_THRESHOLD_DPS = 0.05

TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_OFF = 0x80
COMMAND_MOTOR_ON = 0x88
COMMAND_TORQUE_CLOSED_LOOP = 0xA1
IQ_CONTROL_RANGE = 2048
MAX_MOTOR_TORQUE_NM = 1.2
ENCODER_MAX_VALUE = 65535.0
TORQUE_LIMIT_NM = 1.0

MAX_CURRENT_A_FOR_IQ_RANGE = 33.0
MOTOR_KT_NM_PER_A = 0.128

try:
    bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
except can.CanError as e:
    print(f"Błąd inicjalizacji magistrali CAN na porcie {CAN_INTERFACE}. Sprawdź podłączenie.")
    print(f"Błąd systemowy: {e}")
    exit()

CSV_FILENAME = 'motor_data_friction_comp_36_1.csv'

def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[cmd, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    bus.send(message)
    time.sleep(0.1)

def set_torque_and_get_angle(torque_nm):
    torque_nm = max(-TORQUE_LIMIT_NM, min(TORQUE_LIMIT_NM, torque_nm))
    iq_control_raw = int((torque_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    
    data_payload = [0xA1, 0x00, 0x00, 0x00, iq_bytes[0], iq_bytes[1], 0x00, 0x00]
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=data_payload, is_extended_id=False)
    
    try:
        bus.send(message)
        response = bus.recv(timeout=0.05)
        if response and response.arbitration_id == (TX_RX_BASE_ID + MOTOR_ID):
            raw_encoder_value = struct.unpack('<H', response.data[6:8])[0]
            motor_angle_degrees = (raw_encoder_value / ENCODER_MAX_VALUE) * (360.0 * REDUCTION_RATIO)
            return motor_angle_degrees
    except can.CanError as e:
        print(f"Błąd komunikacji CAN: {e}")
    return None

def convert_iq_to_nm(iq_value_lsb):
    current_A = (iq_value_lsb / IQ_CONTROL_RANGE) * MAX_CURRENT_A_FOR_IQ_RANGE
    torque_nm = current_A * MOTOR_KT_NM_PER_A
    return torque_nm

def calculate_friction_torque(angular_velocity_dps):
    coulomb_comp_iq = 0.0
    viscous_comp_iq = 0.0

    if abs(angular_velocity_dps) > FRICTION_COMP_VELOCITY_THRESHOLD_DPS:
        coulomb_comp_iq = ESTIMATED_COULOMB_FRICTION_IQ * math.copysign(1, angular_velocity_dps)

    viscous_comp_iq = ESTIMATED_VISCOUS_FRICTION_COEFFICIENT_IQ * angular_velocity_dps
    
    total_friction_comp_iq = coulomb_comp_iq + viscous_comp_iq
    total_friction_comp_nm = convert_iq_to_nm(total_friction_comp_iq)
    
    return total_friction_comp_nm

def main_control_loop():
    print(f"Uruchomiono pętlę sterującą z kompensacją tarcia {REDUCTION_RATIO}.")
    print(f"Docelowy silnik: ID={MOTOR_ID}, Interfejs={CAN_INTERFACE}")
    motor_on_off(True)

    current_torque_nm = 0.0
    motor_angle_degrees = set_torque_and_get_angle(0.0)
    if motor_angle_degrees is None:
        print("Błąd komunikacji z silnikiem. Sprawdź połączenie i zasilanie.")
        motor_on_off(False)
        bus.shutdown()
        return

    last_angle_degrees = (motor_angle_degrees - ANGLE_OFFSET_DEGREES) / REDUCTION_RATIO
    last_loop_time = time.time()

    csv_file = None
    csv_writer = None
    header = ['Timestamp', 'Output Angle [deg]', 'Angular Velocity [dps]', 
              'Friction Comp Torque [Nm]', 'Target Torque [Nm]']

    try:
        file_exists = os.path.isfile(CSV_FILENAME)
        csv_file = open(CSV_FILENAME, 'a', newline='')
        csv_writer = csv.writer(csv_file)
        if not file_exists:
            csv_writer.writerow(header)

        while True:
            loop_start_time = time.time()
            time_delta = loop_start_time - last_loop_time
            if time_delta == 0: continue
            last_loop_time = loop_start_time

            calibrated_motor_angle = motor_angle_degrees - ANGLE_OFFSET_DEGREES
            output_angle_degrees = calibrated_motor_angle / REDUCTION_RATIO
            
            angular_velocity_dps = (output_angle_degrees - last_angle_degrees) / time_delta
            
            friction_compensation_torque_nm = calculate_friction_torque(angular_velocity_dps)
            target_torque_nm = friction_compensation_torque_nm

            last_angle_degrees = output_angle_degrees

            current_torque_nm += SMOOTHING_FACTOR * (target_torque_nm - current_torque_nm)

            new_motor_angle = set_torque_and_get_angle(current_torque_nm)
            if new_motor_angle is not None:
                motor_angle_degrees = new_motor_angle

            row_data = [
                time.time(),
                output_angle_degrees,
                angular_velocity_dps,
                friction_compensation_torque_nm,
                current_torque_nm
            ]
            if csv_writer:
                csv_writer.writerow(row_data)
            
            print(f"Kąt: {output_angle_degrees:6.2f}° | Prędkość: {angular_velocity_dps:7.2f} dps | Moment kompensacji: {current_torque_nm:6.3f} Nm", end='\r')
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę sterującą przez użytkownika.")
    finally:
        print("\nWyłączanie silnika...")
        set_torque_and_get_angle(0)
        motor_on_off(False)
        bus.shutdown()
        if csv_file:
            csv_file.close()
        print("Silnik wyłączony, program zakończony.")

if __name__ == "__main__":
    main_control_loop()