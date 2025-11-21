# main.py
import time
import bluetooth
from ble_uart import BLEUART
from pump_controller import Pumps
from machine import I2C, Pin
import time
import utime
from internal_sensor import LPS35HW

def init_sensor():
    # I2C0 on Raspberry Pi Pico: SDA=GP0, SCL=GP1
    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

    # I2C address:
    #   SA0/SDO tied to 3V3 → 0x5D
    #   SA0/SDO tied to GND → 0x5C
    sensor = LPS35HW(i2c, address=0x5D)

    return sensor

def init_log_file():
    log_time = utime.time()

    log = open(f"pressure_log_{log_time}.txt", 'w')

    return log

def pressure_reading(log, sensor):
    try:
        current_time = utime.time()

        p = sensor.pressure_hpa()
        t = sensor.temperature_c()

        print(f"Pressure: {p} hPa | Temp: {t} °C | Time: {current_time}")
        log.write(f"Pressure: {p} hPa | Temp: {t} °C | Time: {current_time} \n")

    except Exception as e:
        print("Sensor read error:", e)

def main():
    # Turn pumps off
    with Pumps() as pump:

        sensor = init_sensor()
        log = init_log_file()

        ble = bluetooth.BLE()
        uart = BLEUART(ble, on_rx=pump.handle_command)

        print("Pump controller ready. Commands over BLE:")
        while True:
            pressure_reading(log, sensor)
            time.sleep(1)

if __name__ == "__main__":
    main()

