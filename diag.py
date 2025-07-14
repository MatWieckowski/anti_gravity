import can
import time
import struct

# --- Konfiguracja ---
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3

# --- Stałe ---
TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_ON = 0x88
COMMAND_MOTOR_OFF = 0x80
COMMAND_SINGLE_TURN_ANGLE = 0x94
COMMAND_TORQUE_CLOSED_LOOP = 0xA1
IQ_CONTROL_RANGE = 2048
MAX_MOTOR_TORQUE_NM = 1.2
# Moment testowy - niewielka dodatnia wartość
TEST_TORQUE_NM = 0.05 

# --- Inicjalizacja i funkcje pomocnicze (takie jak poprzednio) ---
bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

def send_can_command(command_byte, data_payload=None):
    if data_payload is None: data_payload = [0x00] * 7
    full_data = [command_byte] + data_payload
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=full_data, is_extended_id=False)
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
    iq_control_raw = int((torque_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_RANGE)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)
    send_can_command(COMMAND_TORQUE_CLOSED_LOOP, [0,0,0,iq_bytes[0],iq_bytes[1],0,0])

# --- Główna funkcja diagnostyczna ---
def run_diagnostic_test():
    try:
        print("Przygotowanie do testu diagnostycznego...")
        motor_on_off(True)
        
        # Odczyt kąta początkowego
        initial_angle = read_motor_angle()
        if initial_angle is None:
            print("Nie udało się odczytać kąta początkowego.")
            return

        print(f"Kąt początkowy: {initial_angle:.2f}°")
        print(f"Za chwilę na 3 sekundy zostanie podany moment testowy ({TEST_TORQUE_NM} Nm)...")
        print("OBSERWUJ RUCH RAMIENIA I ZMIANĘ KĄTA NA EKRANIE.")
        time.sleep(2)

        # Pętla testowa
        start_time = time.time()
        while time.time() - start_time < 3:
            set_torque_command(TEST_TORQUE_NM)
            current_angle = read_motor_angle()
            if current_angle is not None:
                print(f"Kąt: {current_angle:.2f}°", end='\r')
            time.sleep(0.02)
        
        print("\nTest zakończony.")
        
        # Odczyt kąta końcowego
        final_angle = read_motor_angle()
        if final_angle is not None:
            print(f"Kąt końcowy: {final_angle:.2f}°")
            if final_angle > initial_angle:
                print("Wynik: Wartość kąta ROSŁA.")
            else:
                print("Wynik: Wartość kąta MALAŁA.")
        
    finally:
        print("Wyłączam silnik.")
        # Wyzeruj moment i wyłącz silnik
        set_torque_command(0)
        time.sleep(0.1)
        motor_on_off(False)
        bus.shutdown()

if __name__ == "__main__":
    run_diagnostic_test()