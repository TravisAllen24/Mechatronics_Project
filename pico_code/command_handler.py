# command_handler.py
import time
import pump_controller as pump

def handle_command(raw: str):
    raw = raw.replace("\x00", "").strip()
    if not raw:
        return

    print("Raw command:", repr(raw))

    # Simple commands: "inflate", "deflate", "off", "ping"
    lower = raw.lower()
    if lower in ("inflate", "deflate", "off", "ping") and "," not in lower:
        if lower == "inflate":
            pump.inflate()
        elif lower == "deflate":
            pump.deflate()
        elif lower == "off":
            pump.all_off()
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
        pump.all_off()
        return

    # Delay if required
    if delay > 0:
        print(f"Delaying {delay} seconds before {action}...")
        time.sleep(delay)

    # No duration case → run indefinitely until next command
    if duration is None:
        if action == "inflate":
            pump.inflate()
        elif action == "deflate":
            pump.deflate()
        return

    # Duration given → run and then turn off
    if duration > 0:
        pump.run_action(action, duration)
    else:
        print("Duration was 0 or invalid, ignoring.")

