import can
import time
import struct

CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
REDUCTION_RATIO = 6.0
TX_RX_BASE_ID = 0x140
COMMAND_READ_SINGLE_TURN_ANGLE = 0x94

try:
    bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
    print(f"Pomyślnie połączono z interfejsem CAN na porcie: {bus.channel_info}.")
except Exception as e:
    print(f"Błąd połączenia z interfejsem CAN: {e}")
    exit()

def read_motor_angle():
    """Odczytuje kąt wewnętrznego wału silnika (przed przekładnią)."""
    message = can.Message(
        arbitration_id=TX_RX_BASE_ID + MOTOR_ID,
        data=[COMMAND_READ_SINGLE_TURN_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False
    )
    try:
        bus.send(message)
        response = bus.recv(timeout=0.1)
        
        if response and response.data[0] == COMMAND_READ_SINGLE_TURN_ANGLE:
            angle_raw = struct.unpack('<I', response.data[4:8])[0]
            angle_degrees = angle_raw / 100.0
            return angle_degrees
            
    except can.CanError as e:
        print(f"Błąd komunikacji CAN: {e}")
    return None

def main():
    """Główna pętla do odczytywania i przeliczania kąta."""
    print("\nRozpoczynam odczyt kąta z silnika...")
    print(f"Zastosowane przełożenie: 1:{REDUCTION_RATIO}")
    print("Naciśnij Ctrl+C, aby zakończyć.")
    
    try:
        while True:
            motor_angle = read_motor_angle()
            
            if motor_angle is not None:
                output_angle = motor_angle / REDUCTION_RATIO
                
                print(f"Kąt na wyjściu (skorygowany): {output_angle:6.2f}°", end='\r')
            else:
                print("Nie otrzymano odpowiedzi od silnika...", end='\r')
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nZakończono odczyt przez użytkownika.")
    finally:
        if 'bus' in locals() and bus:
            bus.shutdown()
            print("Połączenie z magistralą CAN zostało zamknięte.")

if __name__ == "__main__":
    main()