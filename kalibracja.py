import can
import time
import struct
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3
TX_RX_BASE_ID = 0x140
COMMAND_READ_SINGLE_TURN_ANGLE = 0x94

def find_offset():
    try:
        bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
        print("Połączono z CAN.")
        
        message = can.Message(
            arbitration_id=TX_RX_BASE_ID + MOTOR_ID,
            data=[COMMAND_READ_SINGLE_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0],
            is_extended_id=False
        )
        
        print("\nPozwól ramieniu swobodnie zwisać w najniższym punkcie.")
        print("Naciśnij Enter, aby odczytać kąt offsetu...")
        input() 

        bus.send(message)
        response = bus.recv(timeout=0.2)
        
        if response and response.data[0] == COMMAND_READ_SINGLE_TURN_ANGLE:
            angle_raw = struct.unpack('<I', response.data[4:8])[0]
            offset_degrees = angle_raw / 100.0
            print("-" * 30)
            print(f"ODCZYTANY KĄT OFFSETU: {offset_degrees:.4f}°")
            print("Zapisz tę wartość i wklej ją do głównego skryptu.")
            print("-" * 30)
        else:
            print("Nie udało się odczytać kąta. Spróbuj ponownie.")

        bus.shutdown()
        
    except Exception as e:
        print(f"Wystąpił błąd: {e}")

if __name__ == "__main__":
    find_offset()