# ble_uart.py
import bluetooth
from micropython import const
from machine import Pin
from ble_advertising import advertising_payload

# Nordic UART-style UUIDs
_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_RX_CHAR_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # write from central
_TX_CHAR_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # notify to central

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_led = Pin("LED", Pin.OUT)

class BLEUART:
    def __init__(self, ble, on_rx=None, name=b"Pico2W-BLE"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        self._on_rx = on_rx          # callback: on_rx(str_message)

        rx_char = (
            _RX_CHAR_UUID,
            bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE,
        )
        tx_char = (
            _TX_CHAR_UUID,
            bluetooth.FLAG_NOTIFY,
        )
        uart_service = (
            _SERVICE_UUID,
            (tx_char, rx_char),
        )

        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services(
            (uart_service,)
        )

        self._connections = set()
        self._payload = advertising_payload(
            name=name,
            services=[_SERVICE_UUID],
        )
        self._advertise()

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, addr_type, addr = data
            print("Central connected:", conn_handle)
            self._connections.add(conn_handle)
            _led.value(1)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, addr_type, addr = data
            print("Central disconnected:", conn_handle)
            self._connections.discard(conn_handle)
            _led.value(0)
            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle == self._rx_handle:
                raw = self._ble.gatts_read(self._rx_handle)
                try:
                    msg = raw.decode("utf-8")
                except UnicodeError:
                    msg = "".join(chr(b) for b in raw)
                print("BLE RX message:", repr(msg))
                if self._on_rx:
                    self._on_rx(msg)

    def send(self, data: bytes):
        for conn in self._connections:
            self._ble.gatts_notify(conn, self._tx_handle, data)

    def _advertise(self, interval_us=500000):
        print("Starting advertising...")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

