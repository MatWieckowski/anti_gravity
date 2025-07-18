import can
import time
import struct
import json

def get_motor_angle(bus, motor_id, base_id):
    """Wysyła pustą komendę i nasłuchuje odpowiedzi, aby odczytać kąt."""
    # Używamy polecenia odczytu ID, które zwykle zwraca status, w tym pozycję.
    # Jeśli to nie zadziała, można użyć komendy 'set_torque' z momentem 0.
    read_id_command = 0x30
    message = can.Message(arbitration_id=base_id + motor_id, data=[read_id_command, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    
    try:
        bus.send(message)
        # Dajemy silnikowi chwilę na odpowiedź
        response = bus.recv(timeout=0.2)
        if response and response.arbitration_id == (base_id + motor_id):
            # Zakładamy, że pozycja enkodera jest w ostatnich dwóch bajtach
            raw_encoder_value = struct.unpack('<H', response.data[6:8])[0]
            return raw_encoder_value
    except can.CanError as e:
        print(f"Błąd komunikacji CAN: {e}")
    return None

def main():
    """Główna funkcja do znajdowania offsetu kąta."""
    try:
        with open('config.json', 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        print("Błąd: Nie znaleziono pliku 'config.json'. Uruchom ten skrypt w tym samym folderze.")
        return

    # Wczytanie konfiguracji
    can_config = config['can']
    
    # --- Wybór silnika ---
    if len(config['motors']) > 1:
        print("Dostępne silniki:")
        for i, motor_conf in enumerate(config['motors']):
            print(f"  {i+1}. {motor_conf['name']} (ID: {motor_conf['id']})")
        
        while True:
            try:
                choice = int(input(f"Wybierz silnik (1-{len(config['motors'])}): "))
                if 1 <= choice <= len(config['motors']):
                    motor_config = config['motors'][choice - 1]
                    break
                else:
                    print("Nieprawidłowy wybór.")
            except ValueError:
                print("Proszę podać liczbę.")
    else:
        motor_config = config['motors'][0]

    motor_id = motor_config['id']
    reduction_ratio = motor_config['reduction_ratio']
    encoder_max_value = config['global_params']['encoder_max_value']
    base_id = int(config['commands']['tx_rx_base_id'], 16)
    
    print("\n--- Program do kalibracji offsetu kątowego ---")
    print(f"Wybrany silnik: {motor_config['name']} (ID: {motor_id})")

    bus = None
    try:
        bus = can.interface.Bus(bustype='slcan', channel=can_config['interface'], bitrate=can_config['bitrate'])
        print(f"Połączono z magistralą CAN na porcie {can_config['interface']}.")

        # Wyłączenie silnika, aby można było nim swobodnie obracać
        motor_off_cmd = int(config['commands']['motor_off'], 16)
        off_message = can.Message(arbitration_id=base_id + motor_id, data=[motor_off_cmd, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
        bus.send(off_message)
        print("Moment obrotowy silnika został wyłączony.")
        
        print("\nKROK 1: Ręcznie ustaw ramię silnika w pozycji referencyjnej (np. idealnie w poziomie, co odpowiada 0 stopni).")
        input("KROK 2: Po ustawieniu ramienia naciśnij Enter, aby odczytać pozycję...")

        raw_value = get_motor_angle(bus, motor_id, base_id)
        
        if raw_value is not None:
            # Obliczenie "surowego" kąta, jaki widzi program bez offsetu
            current_motor_angle = (raw_value / encoder_max_value) * (360.0 * reduction_ratio)
            
            print(f"\nOdczytano surową wartość enkodera: {raw_value}")
            print(f"Obliczony 'surowy' kąt silnika: {current_motor_angle:.4f}°")
            
            print("\n-------------------------------------------------------------")
            print("WYNIK:")
            print(f"  Obliczona wartość 'angle_offset_degrees' to: {current_motor_angle:.4f}")
            print("-------------------------------------------------------------")
            print(f"\nSkopiuj tę wartość i wklej ją do pliku 'config.json' dla silnika o ID {motor_id}.")

        else:
            print("Nie udało się odczytać wartości z silnika. Sprawdź połączenie.")

    except can.CanError as e:
        print(f"Krytyczny błąd CAN: {e}")
    except Exception as e:
        print(f"Wystąpił nieoczekiwany błąd: {e}")
    finally:
        if bus:
            bus.shutdown()
            print("Połączenie CAN zostało zamknięte.")

if __name__ == "__main__":
    main()