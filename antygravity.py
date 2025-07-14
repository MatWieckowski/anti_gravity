import can
import time
import math
import struct

# --- Konfiguracja ---
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
SMOOTHING_FACTOR = 1.6
REDUCTION_RATIO = 6.0
ANGLE_OFFSET_DEGREES = 240.07

# Stałe fizyczne 
MASS_KG = 1.25
DISTANCE_TO_CENTER_OF_MASS_M = 0.06
GRAVITY = 9.81

# Stałe protokołu CAN i silnika
TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_OFF = 0x80
COMMAND_MOTOR_ON = 0x88
COMMAND_TORQUE_CLOSED_LOOP = 0xA1
IQ_CONTROL_RANGE = 2048
MAX_MOTOR_TORQUE_NM = 1.2
ENCODER_MAX_VALUE = 65535.0 

bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[cmd,0,0,0,0,0,0,0], is_extended_id=False)
    bus.send(message)
    time.sleep(0.1)

def set_torque_and_get_angle(torque_nm):

    torque_nm = max(-0.8, min(0.8, torque_nm))
    
    iq_control_raw = int((torque_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    data_payload = [0x00, 0x00, 0x00, iq_bytes[0], iq_bytes[1], 0x00, 0x00]
    
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[COMMAND_TORQUE_CLOSED_LOOP] + data_payload, is_extended_id=False)
    
    try:
        bus.send(message)
        response = bus.recv(timeout=0.05)
        
        if response and response.arbitration_id == (TX_RX_BASE_ID + MOTOR_ID):
            raw_encoder_value = struct.unpack('<H', response.data[6:8])[0]
            
            motor_angle = (raw_encoder_value / ENCODER_MAX_VALUE) * (360.0 * REDUCTION_RATIO)
            return motor_angle

    except can.CanError as e:
        print(f"Błąd CAN: {e}")
    
    return None

def main_control_loop():
    print(f"Uruchamiam zoptymalizowaną pętlę z kalibracją (offset: {ANGLE_OFFSET_DEGREES}°)...")
    motor_on_off(True)
    
    current_torque_nm = 0.0
    motor_angle_degrees = set_torque_and_get_angle(0.0)
    if motor_angle_degrees is None:
        print("Nie udało się nawiązać komunikacji z silnikiem. Zamykanie.")
        motor_on_off(False)
        bus.shutdown()
        return

    try:
        while True:
            calibrated_motor_angle = motor_angle_degrees - ANGLE_OFFSET_DEGREES
            output_angle_degrees = calibrated_motor_angle / REDUCTION_RATIO
            output_angle_radians = math.radians(output_angle_degrees)
            
            target_torque_nm = (MASS_KG * GRAVITY * DISTANCE_TO_CENTER_OF_MASS_M * math.sin(output_angle_radians))
            
            current_torque_nm += SMOOTHING_FACTOR * (target_torque_nm - current_torque_nm)
            
            new_motor_angle = set_torque_and_get_angle(current_torque_nm)
            
            if new_motor_angle is not None:
                motor_angle_degrees = new_motor_angle
            
            print(f"Kąt wyjścia: {output_angle_degrees:6.2f}° | Moment zadany: {current_torque_nm:.3f} Nm", end='\r')
            time.sleep(0.01) 

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę przez użytkownika.")
    finally:
        print("\nWyłączam silnik...")
        set_torque_and_get_angle(0) 
        motor_on_off(False)
        bus.shutdown()

if __name__ == "__main__":
    main_control_loop()