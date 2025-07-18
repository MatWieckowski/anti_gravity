import can
import time
import struct
import csv
import numpy as np

CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
IQ_VALUES_TO_TEST = range(70, 351, 15)  
STABILIZATION_TIME_S = 2.5
SAMPLES_PER_SPEED = 25
OUTPUT_CSV_FILENAME = 'friction_data_torque_mode.csv'
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

def motor_on_off(state):
    """Wysyła komendę włączenia lub wyłączenia silnika."""
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[cmd, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    bus.send(message)
    time.sleep(0.1)

def set_torque_and_read_state(iq_value):
    """
    Wysyła komendę ustawienia prądu Iq i odczytuje odpowiedź ze stanem silnika.
    Zwraca zmierzoną prędkość (dps) i prąd Iq.
    """
    iq_control = int(iq_value)
    iq_control = max(-IQ_CONTROL_RANGE, min(IQ_CONTROL_RANGE, iq_control))
    iq_bytes = struct.pack('<h', iq_control)
    
    data_payload = [COMMAND_TORQUE_CLOSED_LOOP, 0, 0, 0, iq_bytes[0], iq_bytes[1], 0, 0]
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=data_payload, is_extended_id=False)
    
    try:
        bus.send(message)
        response = bus.recv(timeout=0.1)
        if response and response.arbitration_id == (TX_RX_BASE_ID + MOTOR_ID):
            actual_iq = struct.unpack('<h', response.data[2:4])[0]
            actual_speed = struct.unpack('<h', response.data[4:6])[0]
            return actual_speed, actual_iq
    except can.CanError as e:
        print(f"Błąd komunikacji CAN: {e}")
    
    return None, None

def run_friction_test_torque_mode():
    """Główna funkcja przeprowadzająca test pomiaru tarcia w trybie momentu."""
    
    print("Rozpoczynanie procedury pomiaru tarcia (tryb sterowania momentem).")
    print("!!! OSTRZEŻENIE: SILNIK ZACZNIE SIĘ OBRACAĆ !!!")
    time.sleep(3)
    
    motor_on_off(True)
    
    try:
        with open(OUTPUT_CSV_FILENAME, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['target_iq', 'avg_speed_dps', 'avg_actual_iq'])

            for target_iq in IQ_VALUES_TO_TEST:
                print(f"\nUstawianie prądu docelowego Iq: {target_iq}")
                set_torque_and_read_state(target_iq) 
                
                print(f"Stabilizacja przez {STABILIZATION_TIME_S}s...")
                time.sleep(STABILIZATION_TIME_S)

                print("Pobieranie próbek...")
                iq_samples = []
                speed_samples = []

                for i in range(SAMPLES_PER_SPEED):
                    actual_speed, actual_iq = set_torque_and_read_state(target_iq)
                    if actual_speed is not None and actual_iq is not None:
                        iq_samples.append(actual_iq)
                        speed_samples.append(actual_speed)
                    time.sleep(0.05)

                if iq_samples:
                    avg_iq = np.mean(iq_samples)
                    avg_speed = np.mean(speed_samples)
                    print(f"-> Wynik: Śr. prędkość: {avg_speed:.2f} dps | Śr. prąd Iq: {avg_iq:.2f}")
                    csv_writer.writerow([target_iq, avg_speed, avg_iq])
                else:
                    print("-> Błąd: Nie udało się zebrać próbek dla tego poziomu momentu.")

    except KeyboardInterrupt:
        print("\nTest przerwany przez użytkownika.")
    finally:
        print("\nZatrzymywanie silnika i kończenie pracy...")
        set_torque_and_read_state(0) 
        time.sleep(1)
        motor_on_off(False)
        bus.shutdown()
        print(f"Test zakończony. Dane zostały zapisane w pliku '{OUTPUT_CSV_FILENAME}'.")

if __name__ == "__main__":
    run_friction_test_torque_mode()