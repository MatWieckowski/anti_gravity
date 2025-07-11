import can
import time
import math
import struct

#Konfiguracja
CANABLE_PORT = 'COM7'  
CAN_BITRATE = 1000000  
MOTOR_ID = 3           

#Stałe fizyczne
MASS_KG = 0.08
DISTANCE_TO_CENTER_OF_MASS_M = 0.05
GRAVITY = 9.81

#Stałe protokołu CAN
TX_RX_BASE_ID = 0x140  

# Komendy z dokumentacji
COMMAND_MOTOR_OFF = 0x80
COMMAND_MOTOR_ON = 0x88
COMMAND_READ_ENCODER_ANGLE = 0x92
COMMAND_TORQUE_CLOSED_LOOP_CONTROL = 0xA1

# Stałe specyficzne dla silnika
IQ_CONTROL_MAX = 2048
MAX_MOTOR_TORQUE_NM = 10.0

#Inicjalizacja magistrali CAN
try:
    bus = can.interface.Bus(bustype='slcan', channel=CANABLE_PORT, bitrate=CAN_BITRATE)
    print(f"Pomyślnie połączono z CANable na {bus.channel_info} z bitrate {CAN_BITRATE} bps.")
except Exception as e:
    print(f"Błąd połączenia z CANable: {e}")
    print("Upewnij się, że CANable jest podłączony i port COM jest prawidłowy.")
    print("Pamiętaj o wgraniu firmware slcan i instalacji sterownika WinUSB (Zadig) dla Windows.")
    exit()

#Funkcje komunikacyjne
def send_and_receive_command(cmd_byte, data_payload, timeout=0.1):
    """Wysyła ramkę CAN i oczekuje na odpowiedź."""
    if len(data_payload) != 7:
        print(f"Błąd: data_payload musi zawierać 7 bajtów. Otrzymano {len(data_payload)}.")
        return None

    full_data = [cmd_byte] + data_payload
    arb_id = TX_RX_BASE_ID + MOTOR_ID

    message = can.Message(
        arbitration_id=arb_id,
        data=full_data,
        is_extended_id=False
    )

    try:
        bus.send(message)
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = bus.recv(timeout=0.01)
            if response and response.arbitration_id == arb_id and response.data[0] == cmd_byte:
                return response
        return None
    except can.CanError as e:
        print(f"Błąd CAN podczas wysyłania/odbierania: {e}")
        return None

def motor_on():
    """Wysyła komendę włączenia silnika."""
    print("Wysyłam komendę 'Motor On'...")
    data_payload = [0x00] * 7
    response = send_and_receive_command(COMMAND_MOTOR_ON, data_payload)
    if response:
        print("'Motor On' - Potwierdzenie otrzymane.")
        return True
    else:
        print("'Motor On' - Brak potwierdzenia.")
        return False

def motor_off():
    """Wysyła komendę wyłączenia silnika."""
    print("Wysyłam komendę 'Motor Off'...")
    data_payload = [0x00] * 7
    response = send_and_receive_command(COMMAND_MOTOR_OFF, data_payload)
    if response:
        print("'Motor Off' - Potwierdzenie otrzymane.")
        return True
    else:
        print("'Motor Off' - Brak potwierdzenia.")
        return False

def read_encoder_angle():
    
    data_payload = [0x00] * 7
    response = send_and_receive_command(COMMAND_READ_ENCODER_ANGLE, data_payload)

    if response and len(response.data) == 8 and response.data[0] == COMMAND_READ_ENCODER_ANGLE:

        raw_bytes = bytes(response.data[1:8]) 

        if (raw_bytes[6] & 0x80):
            padding_byte = b'\xff'
        else:
            padding_byte = b'\x00'

        full_bytes = raw_bytes + padding_byte

        motor_angle_raw = struct.unpack('<q', full_bytes)[0]

        angle_degrees = motor_angle_raw * 0.01

        angle_radians = math.radians(angle_degrees)

        return angle_radians
    else:
        return None

def calculate_gravity_torque(angle_radians):
    """Oblicza moment grawitacyjny na podstawie kąta."""
    if angle_radians is None:
        return 0.0
    torque_gravity = MASS_KG * GRAVITY * DISTANCE_TO_CENTER_OF_MASS_M * math.sin(angle_radians)
    return torque_gravity

def send_torque_command(torque_value_nm):
    """Konwertuje moment w Nm na wartość iqControl i wysyła komendę."""
    iq_control_raw = int((torque_value_nm / MAX_MOTOR_TORQUE_NM) * IQ_CONTROL_MAX)
    iq_control = max(-IQ_CONTROL_MAX, min(IQ_CONTROL_MAX, iq_control_raw))
    iq_bytes = struct.pack('<h', iq_control)

    data_payload_torque = [0x00, 0x00, 0x00, iq_bytes[0], iq_bytes[1], 0x00, 0x00]
    send_and_receive_command(COMMAND_TORQUE_CLOSED_LOOP_CONTROL, data_payload_torque)

# Główna pętla programu 
def main_control_loop():
    loop_interval_ms = 15
    loop_interval_s = loop_interval_ms / 1000.0

    print(f"\nRozpoczynam pętlę kontroli antygrawitacyjnej (interwał: {loop_interval_ms} ms)...")
    print("Naciśnij Ctrl+C, aby zakończyć.")

    if not motor_on():
        print("Nie udało się włączyć silnika. Sprawdź połączenie i stan silnika.")
        return

    try:
        while True:
            start_loop_time = time.time()
            multi_turn_angle_radians = read_encoder_angle()

            if multi_turn_angle_radians is not None:
                single_turn_angle_radians = multi_turn_angle_radians % (2 * math.pi)

                print(f"Kąt (wiele obrotów): {math.degrees(multi_turn_angle_radians):.2f}° | Kąt (jeden obrót): {math.degrees(single_turn_angle_radians):.2f}°")

                required_torque_nm = calculate_gravity_torque(single_turn_angle_radians)
                send_torque_command(required_torque_nm)

            elapsed_time = time.time() - start_loop_time
            sleep_time = loop_interval_s - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę kontroli antygrawitacyjnej przez użytkownika.")
    except Exception as e:
        print(f"\nWystąpił nieoczekiwany błąd w pętli: {e}")
    finally:
        motor_off()
        if 'bus' in locals() and bus:
            bus.shutdown()
            print("Połączenie z magistralą CAN zostało zamknięte.")


if __name__ == "__main__":
    print("Przygotowanie do uruchomienia trybu antygrawitacyjnego dla silnika LK Motors.")
    main_control_loop()