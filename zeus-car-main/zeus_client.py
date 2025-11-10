import asyncio, websockets, time

ESP_IP = "192.168.4.1"
PORT = 8765

def make_packet(kx=0, ky=0, drift=0, qx=0, qy=0):
    fields = [""] * 26
    fields[9]  = str(drift)
    fields[10] = f"{kx},{ky}"
    fields[16] = f"{qx},{qy}"
    payload = "WS+" + ";".join(fields) + ";\r\n"
    return payload

async def main():
    uri = f"ws://{ESP_IP}:{PORT}"
    async with websockets.connect(uri) as ws:
        print(f"Connected to {uri}")
        while True:
            msg = ";;;;;;;;;;;;;;;;0,0;;;;;;;;;\r\n"
            # show human-readable and byte-level versions
            print("Text:", repr(msg))
            print("Bytes:", list(msg.encode()))
            await ws.send(msg)
            await asyncio.sleep(0.5)

asyncio.run(main())
