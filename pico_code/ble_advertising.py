# ble_advertising.py
from micropython import const
import struct
import bluetooth

_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x03)
_ADV_TYPE_UUID32_COMPLETE = const(0x05)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_TYPE_UUID16_MORE = const(0x02)
_ADV_TYPE_UUID32_MORE = const(0x04)
_ADV_TYPE_UUID128_MORE = const(0x06)
_ADV_TYPE_APPEARANCE = const(0x19)

def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None, appearance=0):
    payload = bytearray()

    def _append(adv_type, value):
        nonlocal payload
        payload += struct.pack("BB", len(value) + 1, adv_type) + value

    # Flags
    _append(
        _ADV_TYPE_FLAGS,
        struct.pack(
            "B",
            (0x01 if limited_disc else 0x02) + (0x18 if br_edr else 0x04),
        ),
    )

    if name:
        _append(_ADV_TYPE_NAME, name)

    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                _append(_ADV_TYPE_UUID16_COMPLETE, b)
            elif len(b) == 4:
                _append(_ADV_TYPE_UUID32_COMPLETE, b)
            elif len(b) == 16:
                _append(_ADV_TYPE_UUID128_COMPLETE, b)

    if appearance:
        _append(_ADV_TYPE_APPEARANCE, struct.pack("<h", appearance))

    return payload


