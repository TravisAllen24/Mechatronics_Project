# main.py
import time
import bluetooth
from ble_uart import BLEUART
import command_handler
import pump_controller as pump

def on_ble_rx(msg: str):
    """
    Called whenever a BLE central writes text to the RX characteristic.
    """
    print("on_ble_rx got:", msg)
    command_handler.handle_command(msg)

def main():

    pump.all_off()
    ble = bluetooth.BLE()
    uart = BLEUART(ble, on_rx=on_ble_rx)

    print("Pump controller ready. Commands over BLE:")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()

