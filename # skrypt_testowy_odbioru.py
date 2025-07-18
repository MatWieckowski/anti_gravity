# skrypt_testowy_odbioru.py
import can

CAN_INTERFACE = 'COM7'
CAN_BITRATE = 1000000

print(f"Nasłuchiwanie na interfejsie {CAN_INTERFACE} z bitrate {CAN_BITRATE}...")
print("Ruszaj silnikiem używając programu producenta lub poruszaj nim ręcznie.")
print("Naciśnij Ctrl+C, aby zakończyć.")

try:
    bus = can.interface.Bus(bustype='slcan', channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
    while True:
        message = bus.recv(timeout=1.0) # Czekaj 1 sekundę na ramkę
        if message:
            print(f"Odebrano ramkę: {message}")
except can.CanError as e:
    print(f"Błąd CAN: {e}")
except KeyboardInterrupt:
    print("\nZakończono nasłuchiwanie.")
finally:
    if 'bus' in locals() and bus:
        bus.shutdown()