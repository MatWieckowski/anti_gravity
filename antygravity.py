import can
import time
import math
import struct

#Konfiguracja
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
SMOOTHING_FACTOR = 0.08
REDUCTION_RATIO = 6.0
SAFEGUARD_TORQUE_LIMIT_NM = 0.8

# Stałe fizyczne 
MASS_KG = 0.2
DISTANCE_TO_CENTER_OF_MASS_M = 0.06
GRAVITY = 9.81

# Stałe protokołu CAN 
TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_OFF = 0x80
COMMAND_MOTOR_ON = 0x88
COMMAND_SINGLE_TURN_ANGLE = 0x94
COMMAND_TORQUE_CLOSED_LOOP = 0xA1

# Stałe silnika
IQ_CONTROL_RANGE = 2048
MAX_MOTOR_TORQUE_NM = 1.2

# Inicjalizacja magistrali CAN
try:
    bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
    print(f"Pomyślnie połączono z CANable na {bus.channel_info}.")
except Exception as e:
    print(f"Błąd połączenia z CANable: {e}")
    exit()

def send_can_command(command_byte, data_payload=None):
    if data_payload is None:
        data_payload = [0x00] * 7
    full_data = [command_byte] + data_payload
    arb_id = TX_RX_BASE_ID + MOTOR_ID
    message = can.Message(arbitration_id=arb_id, data=full_data, is_extended_id=False)
    try:
        bus.send(message)
        return True
    except can.CanError as e:
        print(f"Błąd CAN podczas wysyłania: {e}")
        return False

def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    status_text = "ON" if state else "OFF"
    print(f"Wysyłam komendę Motor {status_text}...")
    send_can_command(cmd)
    time.sleep(0.1)

def read_motor_angle():
    send_can_command(COMMAND_SINGLE_TURN_ANGLE)
    try:
        response = bus.recv(timeout=0.1)
        if response and response.data[0] == COMMAND_SINGLE_TURN_ANGLE:
            angle_raw = struct.unpack('<I', response.data[4:8])[0]
            angle_degrees = angle_raw / 100.0
            return angle_degrees
    except can.CanError as e:
        print(f"Błąd odczytu kąta: {e}")
    return None

def set_torque_command(torque_nm):
    torque_nm = max(-SAFEGUARD_TORQUE_LIMIT_NM, min(SAFEGUARD_TORQUE_LIMIT_NM, torque_nm))
    
    iq_control_raw = int((torque_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    data_payload = [0x00, 0x00, 0x00, iq_bytes[0], iq_bytes[1], 0x00, 0x00]
    send_can_command(COMMAND_TORQUE_CLOSED_LOOP, data_payload)

def main_control_loop():
    loop_interval_ms = 20
    loop_interval_s = loop_interval_ms / 1000.0
    print(f"\nRozpoczynam pętlę kontroli antygrawitacyjnej (przełożenie 1:{REDUCTION_RATIO})...")
    
    motor_on_off(True)
    current_torque_nm = 0.0

    try:
        while True:
            start_loop_time = time.time()
            motor_angle_degrees = read_motor_angle()

            if motor_angle_degrees is not None:
                output_angle_degrees = motor_angle_degrees / REDUCTION_RATIO
                output_angle_radians = math.radians(output_angle_degrees)
                
                target_torque_nm = -(MASS_KG * GRAVITY * DISTANCE_TO_CENTER_OF_MASS_M * math.sin(output_angle_radians))
                
                current_torque_nm += SMOOTHING_FACTOR * (target_torque_nm - current_torque_nm)
                
                set_torque_command(current_torque_nm)

                print(f"Kąt wyjścia: {output_angle_degrees:6.2f}° | Moment zadany: {current_torque_nm:.3f} Nm", end='\r')

            elapsed_time = time.time() - start_loop_time
            sleep_time = loop_interval_s - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę przez użytkownika.")
    finally:
        print("\nWyłączam silnik i zamykam połączenie...")
        set_torque_command(0)
        time.sleep(0.1)
        motor_on_off(False)
        if 'bus' in locals() and bus:
            bus.shutdown()
            print("Połączenie z magistralą CAN zostało zamknięte.")

if __name__ == "__main__":
    main_control_loop()