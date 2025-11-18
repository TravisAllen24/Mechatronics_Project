from machine import Pin, I2C
import time

# ---------------- I2C SETUP ----------------
# <<< IMPORTANT: set these to whatever worked in your scan test >>>
# For side pads labeled SCL/SDA:
SCL_PIN = 1    # change if needed
SDA_PIN = 0    # change if needed

# If you're using the STEMMA QT connector instead, use:
# SCL_PIN = 40
# SDA_PIN = 41

i2c = I2C(
    0,
    scl=Pin(SCL_PIN),
    sda=Pin(SDA_PIN),
    freq=400_000,
)

# ---------------- NPI-19 CONSTANTS ----------------
NPI19_ADDR = 0x28
READ_COMMAND = 0xAC   # "full measurement" command

# Counts range (10–90% of full-scale output from datasheet)
OUT_MIN = 1638
OUT_MAX = 14746

# Your part is 30 psi absolute
P_MIN = 0.0
P_MAX = 30.0


# ---------------- LOW-LEVEL READ ----------------
def read_npi19_raw():
    """
    Trigger a measurement and read raw pressure + temperature.

    Returns:
        (raw_pressure_counts, raw_temperature_bits)
    """
    # Start a measurement
    i2c.writeto(NPI19_ADDR, bytes([READ_COMMAND]))

    # Small delay for conversion
    time.sleep_ms(5)

    # Read 4 bytes back
    data = i2c.readfrom(NPI19_ADDR, 4)
    if len(data) != 4:
        raise RuntimeError("NPI-19: expected 4 bytes, got %d" % len(data))

    b0, b1, b2, b3 = data

    # 14-bit pressure:
    #   raw_p = (top 6 bits of b0) << 8 | b1
    raw_p = ((b0 & 0x3F) << 8) | b1

    # 11-bit temperature:
    #   raw_t = (b2 << 3) | (top 3 bits of b3)
    raw_t = (b2 << 3) | ((b3 & 0xE0) >> 5)

    return raw_p, raw_t


# ---------------- CONVERSIONS ----------------
def counts_to_pressure_psi(counts):
    """
    Convert raw pressure counts to psi using linear mapping.

    Adjust P_MIN / P_MAX / OUT_MIN / OUT_MAX if you calibrate.
    """
    # Clamp to rated range
    if counts < OUT_MIN:
        counts = OUT_MIN
    elif counts > OUT_MAX:
        counts = OUT_MAX

    return (counts - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN) + P_MIN


def temp_bits_to_c(raw_temp_bits):
    """
    Convert raw temperature bits to °C.

    From app note:
      temp_bits spans 0..2047 -> -50..150 °C (200°C span)
    """
    return (raw_temp_bits * 50.0 / 2048.0)


# ---------------- MAIN LOOP ----------------
def main():
    # One-time scan just to show the device is there
    devices = i2c.scan()
    print("I2C devices found:", [hex(d) for d in devices])
    if NPI19_ADDR not in devices:
        print("WARNING: NPI-19 (0x28) not found, check wiring/pins!")
    print()

    while True:
        try:
            raw_p, raw_t = read_npi19_raw()
            psi = counts_to_pressure_psi(raw_p)
            temp_c = temp_bits_to_c(raw_t)

            print(
                "raw_p = {:5d} -> {:6.2f} psi | "
                "raw_t = {:4d} -> {:5.1f} °C".format(
                    raw_p, psi, raw_t, temp_c
                )
            )
        except OSError as e:
            # I2C error (e.g. ETIMEDOUT)
            print("I2C error:", e)
        except Exception as e:
            print("Error:", e)

        time.sleep(0.5)


if __name__ == "__main__":
    main()



