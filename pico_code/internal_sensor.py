from machine import I2C, Pin
import time
import utime

class LPS35HW:
    # Register addresses
    _REG_WHO_AM_I      = 0x0F
    _REG_CTRL_REG1     = 0x10
    _REG_PRESS_OUT_XL  = 0x28
    _REG_TEMP_OUT_L    = 0x2B

    # Expected WHO_AM_I value
    _WHO_AM_I_VALUE = 0xB1

    def __init__(self, i2c: I2C, address: int = 0x5D):
        self.i2c = i2c
        self.addr = address

        # Verify sensor
        who = self.i2c.readfrom_mem(self.addr, self._REG_WHO_AM_I, 1)[0]
        if who != self._WHO_AM_I_VALUE:
            raise RuntimeError("LPS35HW not found (WHO_AM_I=0x{:02X})".format(who))

        # CTRL_REG1:
        #   ODR = 1 Hz, BDU enabled (Block Data Update)
        #   0x12 = 0b00010010
        self.i2c.writeto_mem(self.addr, self._REG_CTRL_REG1, bytes([0x12]))

    def _read_u24(self, reg: int) -> int:
        """Read 24-bit signed pressure value."""
        data = self.i2c.readfrom_mem(self.addr, reg, 3)
        raw = (data[2] << 16) | (data[1] << 8) | data[0]
        if raw & 0x800000:
            raw -= 1 << 24
        return raw

    def _read_i16(self, reg: int) -> int:
        """Read 16-bit signed temperature value."""
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        raw = (data[1] << 8) | data[0]
        if raw & 0x8000:
            raw -= 1 << 16
        return raw

    def pressure_hpa(self) -> float:
        """Return pressure in hPa (4096 LSB/hPa)."""
        raw = self._read_u24(self._REG_PRESS_OUT_XL)
        return raw / 4096.0

    def temperature_c(self) -> float:
        """Return temperature in °C (100 LSB/°C)."""
        raw = self._read_i16(self._REG_TEMP_OUT_L)
        return raw / 100.0
