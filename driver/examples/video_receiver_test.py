#!/usr/bin/env python3
"""
Simple video frame receiver for testing iMuJoCo video streaming.

Usage:
    python video_receiver_test.py [--host HOST] [--port PORT]

This script:
1. Sends periodic "hello" packets to the simulation's video port
2. Receives fragmented UDP video frames
3. Reassembles them and prints frame descriptors
4. Optionally saves the first frame as a raw RGBA file

Requires no dependencies beyond Python stdlib.
"""

import argparse
import socket
import struct
import time
import zlib
from dataclasses import dataclass
from typing import Optional

# Constants matching the fragment protocol
FRAGMENT_MAGIC = 0x4D4A4647  # "MJFG"
FRAGMENT_HEADER_SIZE = 16
MAX_UDP_PAYLOAD = 1472
VIDEO_FRAME_DESC_SIZE = 40

# Video formats
VIDEO_FORMATS = {0: "RGBA8", 1: "RGB8", 2: "DEPTH32F", 3: "JPEG"}


@dataclass
class FragmentHeader:
    magic: int
    message_id: int
    fragment_index: int
    fragment_count: int
    total_size: int
    payload_size: int
    checksum: int


@dataclass
class VideoFrameDesc:
    width: int
    height: int
    stride: int
    format: int
    camera_index: int
    simulation_time: float
    frame_number: int
    data_size: int
    checksum: int


def parse_fragment_header(data: bytes) -> Optional[FragmentHeader]:
    if len(data) < FRAGMENT_HEADER_SIZE:
        return None
    magic, msg_id, frag_idx, frag_count, total_size, payload_size, cksum = struct.unpack(
        "<IHBBIHH", data[:FRAGMENT_HEADER_SIZE]
    )
    if magic != FRAGMENT_MAGIC:
        return None
    return FragmentHeader(magic, msg_id, frag_idx, frag_count, total_size, payload_size, cksum)


def parse_video_frame_desc(data: bytes) -> Optional[VideoFrameDesc]:
    if len(data) < VIDEO_FRAME_DESC_SIZE:
        return None
    width, height, stride, fmt, cam_idx, _, _, sim_time, frame_num, data_size, checksum = struct.unpack(
        "<IIIBBBBdQII", data[:VIDEO_FRAME_DESC_SIZE]
    )
    return VideoFrameDesc(width, height, stride, fmt, cam_idx, sim_time, frame_num, data_size, checksum)


class ReassemblySlot:
    def __init__(self, message_id: int, fragment_count: int, total_size: int):
        self.message_id = message_id
        self.fragment_count = fragment_count
        self.total_size = total_size
        self.buffer = bytearray(total_size)
        self.received = set()
        self.last_activity = time.monotonic()

    def add_fragment(self, index: int, offset: int, payload: bytes):
        if index not in self.received:
            self.buffer[offset : offset + len(payload)] = payload
            self.received.add(index)
            self.last_activity = time.monotonic()

    @property
    def complete(self) -> bool:
        return len(self.received) == self.fragment_count


def main():
    parser = argparse.ArgumentParser(description="iMuJoCo video frame receiver")
    parser.add_argument("--host", default="127.0.0.1", help="Simulation host")
    parser.add_argument("--port", type=int, default=9100, help="Video port")
    parser.add_argument("--save-first", action="store_true", help="Save first frame as raw file")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.1)
    sock.bind(("", 0))

    remote = (args.host, args.port)
    slots: dict[int, ReassemblySlot] = {}
    frames_received = 0
    last_hello = 0.0

    print(f"Receiving video from {args.host}:{args.port}")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            # Send hello periodically
            now = time.monotonic()
            if now - last_hello > 1.0:
                sock.sendto(b"\x01", remote)
                last_hello = now

            try:
                data, addr = sock.recvfrom(MAX_UDP_PAYLOAD)
            except socket.timeout:
                continue

            # Parse fragment header
            frag = parse_fragment_header(data)
            if frag is None:
                # Non-fragmented — try as complete frame
                desc = parse_video_frame_desc(data)
                if desc:
                    pixel_data = data[VIDEO_FRAME_DESC_SIZE:]
                    frames_received += 1
                    fmt_name = VIDEO_FORMATS.get(desc.format, f"?{desc.format}")
                    print(
                        f"Frame #{desc.frame_number}: {desc.width}x{desc.height} "
                        f"{fmt_name} cam={desc.camera_index} "
                        f"t={desc.simulation_time:.3f}s "
                        f"size={desc.data_size} bytes"
                    )
                continue

            # Fragment reassembly
            msg_id = frag.message_id
            if msg_id not in slots:
                slots[msg_id] = ReassemblySlot(msg_id, frag.fragment_count, frag.total_size)

            slot = slots[msg_id]
            payload = data[FRAGMENT_HEADER_SIZE : FRAGMENT_HEADER_SIZE + frag.payload_size]

            # Calculate offset: fragment_index * max_fragment_payload
            max_frag_payload = MAX_UDP_PAYLOAD - FRAGMENT_HEADER_SIZE
            offset = frag.fragment_index * max_frag_payload
            slot.add_fragment(frag.fragment_index, offset, payload)

            if slot.complete:
                # Complete message
                msg = bytes(slot.buffer[: frag.total_size])
                del slots[msg_id]

                desc = parse_video_frame_desc(msg)
                if desc:
                    pixel_data = msg[VIDEO_FRAME_DESC_SIZE:]
                    frames_received += 1
                    fmt_name = VIDEO_FORMATS.get(desc.format, f"?{desc.format}")

                    # Verify CRC
                    computed_crc = zlib.crc32(pixel_data) & 0xFFFFFFFF
                    crc_ok = "OK" if computed_crc == desc.checksum else "MISMATCH"

                    print(
                        f"Frame #{desc.frame_number}: {desc.width}x{desc.height} "
                        f"{fmt_name} cam={desc.camera_index} "
                        f"t={desc.simulation_time:.3f}s "
                        f"size={desc.data_size} bytes "
                        f"CRC={crc_ok}"
                    )

                    if args.save_first and frames_received == 1:
                        fname = f"frame_{desc.frame_number}_{desc.width}x{desc.height}.rgba"
                        with open(fname, "wb") as f:
                            f.write(pixel_data)
                        print(f"  -> Saved to {fname}")

            # Cleanup stale slots
            stale = [k for k, v in slots.items() if now - v.last_activity > 1.0]
            for k in stale:
                del slots[k]

    except KeyboardInterrupt:
        print(f"\nReceived {frames_received} frames total")

    sock.close()


if __name__ == "__main__":
    main()
