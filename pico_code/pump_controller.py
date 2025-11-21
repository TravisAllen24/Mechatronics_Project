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

    def run_action(self, action, duration):
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
        self.all_off()
        print("Action complete.")

    def handle_command(self, raw: str):
        raw = raw.replace("\x00", "").strip()
        if not raw:
            return

        print("Raw command:", repr(raw))

        # Simple commands: "inflate", "deflate", "off", "ping"
        lower = raw.lower()
        if lower in ("inflate", "deflate", "off", "ping") and "," not in lower:
            if lower == "inflate":
                self.inflate()
            elif lower == "deflate":
                self.deflate()
            elif lower == "off":
                self.all_off()
            elif lower == "ping":
                print("Pico command handler alive (ping).")
            return

        # Extended commands: "action,duration,delay"
        # Examples:
        #   "inflate,10,5"
        #   "inflate,10"
        #   "inflate,,5"
        #   "inflate,"        (immediate + no stop)
        parts = [p.strip().lower() for p in raw.split(",")]
        if len(parts) < 1:
            print("Invalid command format.")
            return

        action = parts[0]
        duration = None
        delay = 0

        # Parse duration (optional)
        if len(parts) >= 2 and parts[1] != "":
            try:
                duration = int(parts[1])
            except ValueError:
                print("Invalid duration:", parts[1])
                duration = None  # ignore bad value

        # Parse delay (optional)
        if len(parts) >= 3 and parts[2] != "":
            try:
                delay = int(parts[2])
            except ValueError:
                print("Invalid delay:", parts[2])
                delay = 0

        print("Parsed command:", {"action": action, "duration": duration, "delay": delay})

        if action not in ("inflate", "deflate", "off"):
            print("Invalid action:", action)
            return

        # Handle OFF command (delay allowed)
        if action == "off":
            if delay > 0:
                print(f"Delaying {delay} seconds before off...")
                time.sleep(delay)
            self.all_off()
            return

        # Delay if required
        if delay > 0:
            print(f"Delaying {delay} seconds before {action}...")
            time.sleep(delay)

        # No duration case → run indefinitely until next command
        if duration is None:
            if action == "inflate":
                self.inflate()
            elif action == "deflate":
                self.deflate()
            return

        # Duration given → run and then turn off
        if duration > 0:
            self.run_action(action, duration)
        else:
            print("Duration was 0 or invalid, ignoring.")

