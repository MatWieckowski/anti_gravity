import can
import time
import math
import struct
import csv
import os
import matplotlib.pyplot as plt
import pandas as pd

CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3

REDUCTION_RATIO = 6.0
ANGLE_OFFSET_DEGREES = 240.07

TX_RX_BASE_ID = 0x140
COMMAND_MOTOR_OFF = 0x80
COMMAND_MOTOR_ON = 0x88
COMMAND_SPEED_CLOSED_LOOP = 0xA2
IQ_CONTROL_RANGE = 2048
ENCODER_MAX_VALUE = 65535.0

MAX_TARGET_SPEED_DPS = 500
SPEED_STEP_DPS = 10
MEASUREMENT_DELAY_S = 0.1

bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

FRICTION_CSV_FILENAME = 'friction_data_speed_control.csv'

def motor_on_off(state):
    cmd = COMMAND_MOTOR_ON if state else COMMAND_MOTOR_OFF
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[cmd,0,0,0,0,0,0,0], is_extended_id=False)
    bus.send(message)
    time.sleep(0.1)

def set_speed_and_get_motor_data(speed_dps):
    speed_control_lsb = int(speed_dps / 0.01)
    
    speed_bytes = struct.pack('<i', speed_control_lsb)
    
    data_payload = [0x00,0x00,0x00, speed_bytes[0],speed_bytes[1],speed_bytes[2],speed_bytes[3]]
    message = can.Message(arbitration_id=TX_RX_BASE_ID+MOTOR_ID, data=[COMMAND_SPEED_CLOSED_LOOP]+data_payload, is_extended_id=False)
    
    try:
        bus.send(message)
        
        for _ in range(3):
            response = bus.recv(timeout=0.1)
            
            if response:
                if response.arbitration_id == (TX_RX_BASE_ID + MOTOR_ID):
                    if response.data[0] == COMMAND_SPEED_CLOSED_LOOP:
                        temperature = struct.unpack('<b', response.data[1:2])[0]
                        iq = struct.unpack('<h', response.data[2:4])[0]
                        actual_speed_dps = struct.unpack('<h', response.data[4:6])[0]
                        encoder = struct.unpack('<H', response.data[6:8])[0]

                        actual_speed_rad_s = math.radians(actual_speed_dps)
                        
                        return temperature, iq, actual_speed_rad_s, encoder
                    elif response.data[0] == COMMAND_MOTOR_ON:
                        continue
                    else:
                        continue 
                else:
                    continue
            else:
                break 

    except can.CanError as e:
        print(f"Błąd CAN: {e}")
    except Exception as e:
        print(f"Ogólny błąd w set_speed_and_get_motor_data: {e}")
    return None, None, None, None

def plot_friction_data(filename):
    try:
        df = pd.read_csv(filename)

        plt.figure(figsize=(10, 6))

        plt.plot(df['Motor Speed [rad/s]'], df['IQ Value'], 'o-', markersize=4)
        plt.title('IQ Value vs. Prędkość silnika')
        plt.xlabel('Prędkość silnika [rad/s]')
        plt.ylabel('IQ Value')
        plt.grid(True)

        plt.tight_layout()
        plt.show()

    except FileNotFoundError:
        print(f"Błąd: Plik '{filename}' nie został znaleziony. Upewnij się, że pomiar tarcia został wykonany.")
    except Exception as e:
        print(f"Wystąpił błąd podczas generowania wykresów: {e}")

def measure_friction_profile():
    print("\n--- Rozpoczynanie pomiaru tarcia (sterowanie prędkością) ---")
    motor_on_off(True)
    time.sleep(0.5)

    friction_header = ['Timestamp', 'Target Speed [dps]', 'Output Angle [deg]', 'Motor Speed [rad/s]', 'IQ Value', 'Encoder Raw']
    friction_data_points = []
    
    _, _, _, initial_encoder_raw = set_speed_and_get_motor_data(0.0) 
    if initial_encoder_raw is None:
        print("Nie udało się pobrać początkowych danych silnika. Anulowanie pomiaru tarcia.")
        motor_on_off(False)
        bus.shutdown()
        return

    initial_output_angle_deg = ( (initial_encoder_raw / ENCODER_MAX_VALUE) * (360.0 * REDUCTION_RATIO) - ANGLE_OFFSET_DEGREES * REDUCTION_RATIO) / REDUCTION_RATIO
    print(f"Początkowy kąt wyjściowy: {initial_output_angle_deg:.2f}°")

    print(f"Zwiększanie prędkości od 0 dps do {MAX_TARGET_SPEED_DPS} dps w krokach po {SPEED_STEP_DPS} dps...")

    FILTER_ALPHA = 0.2 
    filtered_speed_rad_s = 0.0

    try:
        target_speed_dps = 0.0
        while target_speed_dps <= MAX_TARGET_SPEED_DPS:
            temp, iq_val, actual_speed_rad_s_raw, encoder_raw = set_speed_and_get_motor_data(target_speed_dps)
            
            if temp is None:
                print("Błąd komunikacji podczas pomiaru tarcia. Przerywanie.")
                break

            if target_speed_dps == 0.0:
                filtered_speed_rad_s = actual_speed_rad_s_raw
            else:
                filtered_speed_rad_s = (FILTER_ALPHA * actual_speed_rad_s_raw) + ((1 - FILTER_ALPHA) * filtered_speed_rad_s)

            current_motor_angle_degrees = (encoder_raw / ENCODER_MAX_VALUE) * (360.0 * REDUCTION_RATIO)
            calibrated_motor_angle = current_motor_angle_degrees - (ANGLE_OFFSET_DEGREES * REDUCTION_RATIO)
            output_angle_deg = calibrated_motor_angle / REDUCTION_RATIO


            friction_data_points.append([
                time.time(),
                target_speed_dps,
                output_angle_deg,
                filtered_speed_rad_s,
                iq_val,
                encoder_raw
            ])
            
            print(f"Prędkość zadana: {target_speed_dps:6.2f} dps | Prędkość akt. (filtrowana): {filtered_speed_rad_s:6.2f} rad/s | IQ: {iq_val:6d}", end='\r')

            target_speed_dps += SPEED_STEP_DPS
            time.sleep(MEASUREMENT_DELAY_S)
        
        print("\n--- Pomiar w kierunku dodatnim zakończony ---")
        time.sleep(1)

        filtered_speed_rad_s = 0.0 

        target_speed_dps = -SPEED_STEP_DPS
        while target_speed_dps >= -MAX_TARGET_SPEED_DPS:
            temp, iq_val, actual_speed_rad_s_raw, encoder_raw = set_speed_and_get_motor_data(target_speed_dps)
            
            if temp is None:
                print("Błąd komunikacji podczas pomiaru tarcia w kierunku ujemnym. Przerywanie.")
                break

            if target_speed_dps == -SPEED_STEP_DPS:
                 filtered_speed_rad_s = actual_speed_rad_s_raw
            else:
                 filtered_speed_rad_s = (FILTER_ALPHA * actual_speed_rad_s_raw) + ((1 - FILTER_ALPHA) * filtered_speed_rad_s)

            current_motor_angle_degrees = (encoder_raw / ENCODER_MAX_VALUE) * (360.0 * REDUCTION_RATIO)
            calibrated_motor_angle = current_motor_angle_degrees - (ANGLE_OFFSET_DEGREES * REDUCTION_RATIO)
            output_angle_deg = calibrated_motor_angle / REDUCTION_RATIO

            friction_data_points.append([
                time.time(),
                target_speed_dps,
                output_angle_deg,
                filtered_speed_rad_s,
                iq_val,
                encoder_raw
            ])
            
            print(f"Prędkość zadana: {target_speed_dps:6.2f} dps | Prędkość akt. (filtrowana): {filtered_speed_rad_s:6.2f} rad/s | IQ: {iq_val:6d}", end='\r')

            target_speed_dps -= SPEED_STEP_DPS
            time.sleep(MEASUREMENT_DELAY_S)
        
        print("\n--- Pomiar w kierunku ujemnym zakończony ---")


    except KeyboardInterrupt:
        print("\nPomiar tarcia przerwany przez użytkownika.")
    finally:
        print("\n--- Pomiar tarcia zakończony ---")
        set_speed_and_get_motor_data(0.0)
        motor_on_off(False)
        bus.shutdown()

        with open(FRICTION_CSV_FILENAME, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(friction_header)
            csv_writer.writerows(friction_data_points)
        print(f"Dane tarcia zapisane do {FRICTION_CSV_FILENAME}")
        
        plot_friction_data(FRICTION_CSV_FILENAME)

if __name__ == "__main__":
    measure_friction_profile()