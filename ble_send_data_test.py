import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "PicoBLE"
CHAR_UUID   = "12345678-1234-5678-1234-56789ABCDEF1"  # must match Pico

async def main():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)

    if not device:
        print("Device not found. Make sure the Pico is powered and advertising.")
        return

    print("Found:", device)
    async with BleakClient(device) as client:
        print("Connected to Pico.")
        while True:
            msg = input("Type a message to send: ")
            await client.write_gatt_char(CHAR_UUID, msg.encode())
            print("Message sent.")

            if msg.lower() == "exit":
                print("Exiting.")
                break

if __name__ == "__main__":
    asyncio.run(main())
