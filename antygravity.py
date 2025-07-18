import can
import time
import math
import struct
import csv 
import os 

#Konfiguracja
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
REDUCTION_RATIO = 6.0
ANGLE_OFFSET_DEGREES = 240.07

#Stałe do strojenia
SMOOTHING_FACTOR = 0.7
D_REG = 0.05
HOLD_ZONE_MIN_OUTPUT_ANGLE = 175.0
HOLD_ZONE_MAX_OUTPUT_ANGLE = 185.0
SETTLE_VELOCITY_THRESHOLD_RPS = 0.8
COULOMB_FRICTION_FACTOR = 0.05

# Stałe fizyczne
MASS_KG = 1.25
DISTANCE_TO_CENTER_OF_MASS_M = 0.06
GRAVITY = 9.81

# Stałe protokołu CAN i silnika
TX_RX_BASE_ID = 0x140; COMMAND_MOTOR_OFF = 0x80; COMMAND_MOTOR_ON = 0x88;
COMMAND_TORQUE_CLOSED_LOOP = 0xA1; IQ_CONTROL_RANGE = 2048; MAX_MOTOR_TORQUE_NM = 1.2;
ENCODER_MAX_VALUE = 65535.0
TORQUE_LIMIT_NM = 0.8
bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

# Nazwa pliku CSV
CSV_FILENAME = 'motor_data.csv'

# Funkcje pomocnicze
def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[cmd,0,0,0,0,0,0,0], is_extended_id=False)
    bus.send(message)
    time.sleep(0.1)

def set_torque_and_get_angle(torque_nm):
    #Ograniczenie momentu
    torque_nm = max(-TORQUE_LIMIT_NM, min(TORQUE_LIMIT_NM, torque_nm))
    iq_control_raw = int((torque_nm/MAX_MOTOR_TORQUE_NM)*IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    data_payload = [0x00,0x00,0x00, iq_bytes[0],iq_bytes[1],0x00,0x00]
    message = can.Message(arbitration_id=TX_RX_BASE_ID+MOTOR_ID, data=[COMMAND_TORQUE_CLOSED_LOOP]+data_payload, is_extended_id=False)
    try:
        bus.send(message)
        response = bus.recv(timeout=0.05)
        if response and response.arbitration_id == (TX_RX_BASE_ID + MOTOR_ID):
            raw_encoder_value = struct.unpack('<H', response.data[6:8])[0]
            motor_angle = (raw_encoder_value/ENCODER_MAX_VALUE)*(360.0*REDUCTION_RATIO)
            return motor_angle
    except can.CanError as e:
        print(f"Błąd CAN :{e}")
    return None

def main_control_loop():
    print(f"Pętla z zadaną strefą spoczynku dla kąta wyjściowego: ({HOLD_ZONE_MIN_OUTPUT_ANGLE}° - {HOLD_ZONE_MAX_OUTPUT_ANGLE}°)")
    motor_on_off(True)

    current_torque_nm = 0.0
    motor_angle_degress = set_torque_and_get_angle(0.0)
    if motor_angle_degress is None:
        print ("Błąd komunikacji, sprawdź czy podłączono zasilacz do prądu "); motor_on_off(False); bus.shutdown(); return

    last_angle_rad = math.radians((motor_angle_degress - ANGLE_OFFSET_DEGREES) / REDUCTION_RATIO)
    last_loop_time = time.time()
    is_holding = False

    # Konfiguracja zapisu do CSV
    csv_file = None
    csv_writer = None
    header = ['Timestamp', 'Output Angle [deg]', 'Motor Angle [deg]', 'Calibrated Motor Angle [deg]', 'Angular Velocity [rad/s]', 'Target Torque [Nm]', 'Current Torque [Nm]', 'Is Holding']

    try:
        csv_file = open(CSV_FILENAME, 'a', newline='')
        csv_writer = csv.writer(csv_file)

        if os.stat(CSV_FILENAME).st_size == 0:
            csv_writer.writerow(header)

        while True:
            loop_start_time = time.time()
            time_delta = loop_start_time - last_loop_time
            last_loop_time = loop_start_time

            calibrated_motor_angle = motor_angle_degress - ANGLE_OFFSET_DEGREES
            output_angle_degress = calibrated_motor_angle / REDUCTION_RATIO
            output_angle_radians = math.radians(output_angle_degress)

            angular_velocity = 0.0
            if time_delta > 0:
                angular_velocity = (output_angle_radians - last_angle_rad) / time_delta

            is_in_hold_zone = (HOLD_ZONE_MIN_OUTPUT_ANGLE < output_angle_degress < HOLD_ZONE_MAX_OUTPUT_ANGLE)
            
            target_torque_nm = 0.0
            
            if is_holding:
                if not is_in_hold_zone:
                    is_holding = False
            else:
                if is_in_hold_zone and abs(angular_velocity) < SETTLE_VELOCITY_THRESHOLD_RPS:
                    is_holding = True
                else:
                    gravity_torque_nm = (MASS_KG * GRAVITY * DISTANCE_TO_CENTER_OF_MASS_M * math.sin(output_angle_radians))
                    damping_torque_nm = -D_REG * angular_velocity
                    friction_comp_torque = 0.0
                    if abs(angular_velocity) > 0.01:
                        friction_comp_torque = math.copysign(COULOMB_FRICTION_FACTOR, angular_velocity)
                    target_torque_nm = gravity_torque_nm + damping_torque_nm + friction_comp_torque

            last_angle_rad = output_angle_radians
            
            if not is_holding:
                current_torque_nm += SMOOTHING_FACTOR * (target_torque_nm - current_torque_nm)
            else:
                current_torque_nm = 0.0

            new_motor_angle = set_torque_and_get_angle(current_torque_nm)
            if new_motor_angle is not None:
                motor_angle_degress = new_motor_angle

            # Zapisz dane do pliku CSV
            row_data = [
                time.time(),
                output_angle_degress,
                motor_angle_degress,
                calibrated_motor_angle,
                angular_velocity,
                target_torque_nm,
                current_torque_nm,
                is_holding
            ]
            if csv_writer:
                csv_writer.writerow(row_data)

            print(f"Kąt:{output_angle_degress:6.2f}° |Moment zadany:{current_torque_nm:6.2f}|", end='\r')
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę")
    finally:
        print("\nWyłączam silnik")
        set_torque_and_get_angle(0)
        motor_on_off(False)
        bus.shutdown()
        if csv_file:
            csv_file.close() 

if __name__ == "__main__":
    main_control_loop()