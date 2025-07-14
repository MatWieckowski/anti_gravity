import can
import time
import struct

# --- Konfiguracja ---
CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000
MOTOR_ID = 3

# Stałe protokołu CAN
TX_RX_BASE_ID = 0x140
COMMAND_READ_PID = 0x30
COMMAND_WRITE_PID_TO_RAM = 0x31

# Inicjalizacja magistrali
bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

def read_pids():
    """Wysyła komendę 0x30 i odczytuje parametry PID z silnika."""
    print("Odczytuję aktualne parametry PID z silnika...")
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=[COMMAND_READ_PID,0,0,0,0,0,0,0], is_extended_id=False)
    try:
        bus.send(message)
        response = bus.recv(timeout=0.2)
        if response and response.data[0] == COMMAND_READ_PID:
            angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki = response.data[2:8]
            print("\n--- Aktualne ustawienia PID ---")
            print(f"Pętla Kąta    (Angle):    Kp = {angle_kp}, Ki = {angle_ki}")
            print(f"Pętla Prędkości (Speed):  Kp = {speed_kp}, Ki = {speed_ki}")
            print(f"Pętla Prądu     (Current):  Kp = {iq_kp}, Ki = {iq_ki}")
            print("---------------------------------")
            return True
    except can.CanError as e:
        print(f"Błąd CAN: {e}")
    
    print("Nie udało się odczytać parametrów PID.")
    return False

def set_pids_to_ram(angle_p, angle_i, speed_p, speed_i, current_p, current_i):
    """Wysyła komendę 0x31, aby zapisać nowe parametry PID do pamięci RAM silnika."""
    print("\nWysyłam nowe parametry PID do pamięci RAM silnika...")
    data = [
        COMMAND_WRITE_PID_TO_RAM,
        0x00,
        int(angle_p), int(angle_i),
        int(speed_p), int(speed_i),
        int(current_p), int(current_i)
    ]
    message = can.Message(arbitration_id=TX_RX_BASE_ID + MOTOR_ID, data=data, is_extended_id=False)
    try:
        bus.send(message)
        # Czekamy na odpowiedź, która jest echem wysłanej komendy
        response = bus.recv(timeout=0.2)
        if response and response.data == bytearray(data):
            print("Sukces! Nowe parametry PID zostały zapisane w RAM.")
            print("Będą aktywne do momentu odłączenia zasilania.")
            return True
    except can.CanError as e:
        print(f"Błąd CAN: {e}")
        
    print("Nie udało się zapisać nowych parametrów PID.")
    return False

def main():
    try:
        # Krok 1: Zawsze najpierw odczytujemy, żeby wiedzieć do czego wracać
        if not read_pids():
            return

        # Krok 2: Propozycja bardzo "łagodnych" ustawień
        print("\nSugerowane 'bezpieczne' wartości PID do testów:")
        print("Angle P/I = 1, Speed P/I = 1, Current P/I = 20")
        
        choice = input("Czy chcesz wgrać nowe wartości PID do RAM? (tak/nie): ").lower()
        
        if choice == 'tak':
            # Krok 3: Wgrywamy nowe, niskie wartości
            # Drastycznie obniżamy wzmocnienia pętli kąta i prędkości
            # Pętlę prądu zostawiamy na rozsądnym, ale nieagresywnym poziomie
            success = set_pids_to_ram(
                angle_p=1, angle_i=1,
                speed_p=1, speed_i=1,
                current_p=20, current_i=20
            )
            if success:
                print("\nMożesz teraz zatrzymać ten skrypt (Ctrl+C) i uruchomić swój główny kod sterujący.")
                # Dajemy użytkownikowi czas na reakcję
                while True: time.sleep(1)
        else:
            print("Nie wprowadzono zmian.")

    finally:
        bus.shutdown()
        print("Połączenie CAN zamknięte.")

if __name__ == "__main__":
    main()
    