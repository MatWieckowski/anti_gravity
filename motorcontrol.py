import can
import time
import math
import struct

# --- Konfiguracja ---
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
REDUCTION_RATIO = 6.0

# !!! WKLEJ TUTAJ SWÓJ ZMIERZONY OFFSET Z KROKU KALIBRACYJNEGO !!!
ANGLE_OFFSET_DEGREES = 240.07

# --- Stałe do strojenia regulatora ---
TARGET_ANGLE_DEGREES = 90.0  # Pozycja docelowa (90° = poziomo)
P_FACTOR = 0.04              # Wzmocnienie proporcjonalne
D_FACTOR = 0.004             # Wzmocnienie różniczkujące (tłumienie)

# Stałe fizyczne i silnika
MAX_MOTOR_TORQUE_NM = 1.2
IQ_CONTROL_RANGE = 2048
TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_ON = 0x88
COMMAND_MOTOR_OFF = 0x80
COMMAND_SINGLE_TURN_ANGLE = 0x94
COMMAND_TORQUE_CLOSED_LOOP = 0xA1

# Inicjalizacja magistrali
bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

# Funkcje pomocnicze
def send_can_command(command_byte, data_payload=None):
    if data_payload is None: data_payload = [0x00] * 7
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[command_byte] + data_payload, is_extended_id=False)
    try: bus.send(message)
    except can.CanError: pass

def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    send_can_command(cmd); time.sleep(0.1)

def read_motor_angle():
    send_can_command(COMMAND_SINGLE_TURN_ANGLE)
    try:
        response = bus.recv(timeout=0.1)
        if response and response.data[0] == COMMAND_SINGLE_TURN_ANGLE:
            return struct.unpack('<I', response.data[4:8])[0] / 100.0
    except can.CanError: pass
    return None

def set_torque_command(torque_nm):
    torque_nm = max(-0.8, min(0.8, torque_nm))
    iq_control_raw = int((torque_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    send_can_command(COMMAND_TORQUE_CLOSED_LOOP, [0,0,0,iq_bytes[0],iq_bytes[1],0,0])


def main_pd_controller():
    print(f"Uruchamiam ostateczny regulator pozycji. Cel: {TARGET_ANGLE_DEGREES}°")
    motor_on_off(True)
    
    last_error = 0
    # Używamy odczytanego kąta do inicjalizacji, aby uniknąć skoku na starcie
    initial_motor_angle = read_motor_angle()
    if initial_motor_angle is not None:
        calibrated_angle = initial_motor_angle - ANGLE_OFFSET_DEGREES
        last_error = TARGET_ANGLE_DEGREES - (calibrated_angle / REDUCTION_RATIO)

    last_loop_time = time.time()

    try:
        while True:
            loop_start_time = time.time()
            time_delta = loop_start_time - last_loop_time
            last_loop_time = loop_start_time

            motor_angle_degrees = read_motor_angle()

            if motor_angle_degrees is not None and time_delta > 0:
                calibrated_motor_angle = motor_angle_degrees - ANGLE_OFFSET_DEGREES
                current_angle_degrees = calibrated_motor_angle / REDUCTION_RATIO
                
                error = TARGET_ANGLE_DEGREES - current_angle_degrees
                error_derivative = (error - last_error) / time_delta
                
                torque_p = -P_FACTOR * error
                
                torque_d = -D_FACTOR * error_derivative
                
                target_torque_nm = torque_p + torque_d
                
                set_torque_command(target_torque_nm)
                
                last_error = error
                
                print(f"Cel: {TARGET_ANGLE_DEGREES:.1f} | Aktualnie: {current_angle_degrees:6.2f}° | Błąd: {error:6.2f}° | Moment: {target_torque_nm:.3f} Nm", end='\r')

            # Stały interwał pętli
            sleep_time = 0.02 - (time.time() - loop_start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nZatrzymano.")
    finally:
        print("\nWyłączam silnik...")
        set_torque_command(0); time.sleep(0.1)
        motor_on_off(False)
        bus.shutdown()

if __name__ == "__main__":
    main_pd_controller()