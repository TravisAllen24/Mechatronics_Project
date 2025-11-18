# send_ble_command.py
import asyncio
from bleak import BleakClient

PICO_ADDRESS = "2C:CF:67:EF:A7:AE"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

def get_command_from_user():
    # Ask for action
    while True:
        action = input("Action (inflate / deflate / off): ").strip().lower()
        if action in ("inflate", "deflate", "off"):
            break
        print("Invalid action. Please enter 'inflate', 'deflate', or 'off'.")

    # Ask for duration
    dur_raw = input("Duration in seconds (e.g. 10, blank = 0): ").strip()
    if dur_raw == "":
        duration = 0
    else:
        try:
            duration = int(dur_raw)
        except ValueError:
            print("Invalid duration, defaulting to 0.")
            duration = 0

    # Ask for delay
    delay_raw = input("Delay in seconds before action (e.g. 5, blank = 0): ").strip()
    if delay_raw == "":
        delay = 0
    else:
        try:
            delay = int(delay_raw)
        except ValueError:
            print("Invalid delay, defaulting to 0.")
            delay = 0

    # Compose short command string: "inflate,10,5"
    cmd_str = f"{action},{duration},{delay}"
    print("Command to send:", cmd_str)
    return cmd_str

async def send_command(cmd_str: str):
    print(f"\nConnecting to {PICO_ADDRESS} ...")
    async with BleakClient(PICO_ADDRESS) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return

        print("Connected. Writing command...")
        # small message, fits in a single BLE write
        await client.write_gatt_char(RX_CHAR_UUID, cmd_str.encode("utf-8"), response=True)
        print("Message sent.")

    print("Disconnected.")

def main():
    while True:
        cmd_str = get_command_from_user()
        asyncio.run(send_command(cmd_str))

        again = input("\nSend another command? (y/N): ").strip().lower()
        if again != "y":
            break

if __name__ == "__main__":
    main()
