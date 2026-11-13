#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["smpclient[ble]==7.4.0"]
# ///
"""Update the board over Bluetooth from a computer: upload, test boot, confirm.

    uv run firmware/dfu.py [firmware/build/firmware/zephyr/zephyr.signed.bin]
"""
import asyncio
import sys
from pathlib import Path

from smpclient import SMPClient
from smpclient.generics import error
from smpclient.mcuboot import IMAGE_TLV, ImageInfo
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smpclient.requests.os_management import ResetWrite
from smpclient.transport.ble import SMPBLETransport

NAME = "OpenRZ67"
DEFAULT = Path(__file__).parent / "build/firmware/zephyr/zephyr.signed.bin"


def die(msg: str) -> None:
    sys.exit(f"dfu: {msg}")


async def request(client: SMPClient, req):
    r = await client.request(req)
    if error(r):
        die(f"{type(req).__name__} failed: {r}")
    return r


async def active_hash(client: SMPClient) -> bytes:
    r = await request(client, ImageStatesRead())
    return next(i.hash for i in r.images if i.active)


async def connect() -> SMPClient:
    for _ in range(10):
        client = SMPClient(SMPBLETransport(), NAME, timeout_s=10)
        try:
            await client.connect()
            return client
        except Exception:
            await asyncio.sleep(2)
    die(f"{NAME} not found. Press the button and close the phone app.")


async def main(path: Path) -> None:
    # The device reports the hash TLV MCUboot uses (SHA512 with ed25519), so read it from the file
    info = ImageInfo.load_file(str(path))
    new = info.get_tlv(IMAGE_TLV.SHA512).value

    client = await connect()
    if await active_hash(client) == new:
        print("already running this image")
        return await client.disconnect()

    data = path.read_bytes()
    async for offset in client.upload(data):
        print(f"\rupload {offset * 100 // len(data)} %", end="", flush=True)
    print()
    await request(client, ImageStatesWrite(hash=new))
    await request(client, ResetWrite())
    await client.disconnect()

    print("rebooting into the new image")
    await asyncio.sleep(3)
    client = await connect()
    if await active_hash(client) != new:
        die("the board did not boot the new image, still on the old one")
    await request(client, ImageStatesWrite(confirm=True))
    await client.disconnect()
    print("done, new image confirmed")


asyncio.run(main(Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT))
