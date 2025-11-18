# pump_controller.py
from machine import Pin
import time

# Configure pins
pin16 = Pin(16, Pin.OUT)
pin17 = Pin(17, Pin.OUT)

def inflate():
    pin17.value(0)   # ensure safety: turn off other pin first
    pin16.value(1)
    print("Inflating: GPIO16 ON, GPIO17 OFF")

def deflate():
    pin16.value(0)   # ensure safety
    pin17.value(1)
    print("Deflating: GPIO16 OFF, GPIO17 ON")

def all_off():
    pin16.value(0)
    pin17.value(0)
    print("Both OFF")

def run_action(action, duration):
    """Run inflate/deflate for `duration` seconds, then turn everything off."""
    if action == "inflate":
        inflate()
    elif action == "deflate":
        deflate()
    else:
        print("Unknown action:", action)
        return

    print(f"Holding {action} for {duration} seconds...")
    time.sleep(duration)
    all_off()
    print("Action complete.")

