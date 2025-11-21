# pump_controller.py
from machine import Pin
import time

class Pumps:

    def __init__(self):
        # Configure pins
        self.pin16 = Pin(16, Pin.OUT)
        self.pin17 = Pin(17, Pin.OUT)

        self.all_off()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.all_off()

    def inflate(self):
        self.pin17.value(0)   # ensure safety: turn off other pin first
        self.pin16.value(1)
        print("Inflating: GPIO16 ON, GPIO17 OFF")

    def deflate(self):
        self.pin16.value(0)   # ensure safety
        self.pin17.value(1)
        print("Deflating: GPIO16 OFF, GPIO17 ON")

    def all_off(self):
        self.pin16.value(0)
        self.pin17.value(0)
        print("Both OFF")

    def run_action(action, duration):
        """Run inflate/deflate for `duration` seconds, then turn everything off."""
        if action == "inflate":
            self.inflate()
        elif action == "deflate":
            self.deflate()
        else:
            print("Unknown action:", action)
            return

        print(f"Holding {action} for {duration} seconds...")
        time.sleep(duration)
        all_off()
        print("Action complete.")

