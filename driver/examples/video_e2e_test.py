#!/usr/bin/env python3
"""
End-to-end video transport test.

Verifies the full pipeline: create test image → fragment → UDP → reassemble → save PNG.
Acts as both sender (simulating iMuJoCo sim side) and receiver (simulating driver side).

Usage:
    python video_e2e_test.py

Outputs:
    video_e2e_test_sent.ppm    -- The original test image
    video_e2e_test_received.ppm -- The image after UDP transport round-trip

Both files should be pixel-identical. Open them in any image viewer to verify.
No dependencies beyond Python stdlib.
"""

import socket
import struct
import threading
import time
import zlib

# ── Fragment protocol constants (must match mjc_fragment.h) ──────────────────

FRAGMENT_MAGIC = 0x4D4A4647  # "MJFG"
FRAGMENT_HEADER_SIZE = 16
MAX_UDP_PAYLOAD = 1472
MAX_FRAGMENT_PAYLOAD = MAX_UDP_PAYLOAD - FRAGMENT_HEADER_SIZE  # 1456
VIDEO_FRAME_DESC_SIZE = 40

# CRC-16-CCITT lookup table (polynomial 0x1021, init 0xFFFF)
_CRC16_TABLE = []
for _i in range(256):
    _crc = _i << 8
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x1021) if (_crc & 0x8000) else (_crc << 1)
    _CRC16_TABLE.append(_crc & 0xFFFF)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def make_fragment_header(message_id, fragment_index, fragment_count,
                         total_size, payload_size) -> bytes:
    """Build a 16-byte fragment header with valid CRC-16 checksum."""
    # Pack first 14 bytes (without checksum)
    partial = struct.pack(
        "<IHBBIH",
        FRAGMENT_MAGIC, message_id, fragment_index, fragment_count,
        total_size, payload_size,
    )
    checksum = crc16(partial)
    return partial + struct.pack("<H", checksum)


def fragment_message(data: bytes, message_id: int) -> list[bytes]:
    """Fragment a message into UDP-safe packets (mirrors FragmentedSender)."""
    size = len(data)
    frag_count = (size + MAX_FRAGMENT_PAYLOAD - 1) // MAX_FRAGMENT_PAYLOAD
    fragments = []
    offset = 0
    for i in range(frag_count):
        payload_size = min(MAX_FRAGMENT_PAYLOAD, size - offset)
        header = make_fragment_header(
            message_id, i, frag_count, size, payload_size
        )
        fragments.append(header + data[offset:offset + payload_size])
        offset += payload_size
    return fragments


def make_video_frame_desc(width, height, fmt, camera_index,
                          sim_time, frame_number, pixel_data) -> bytes:
    """Build the 40-byte MJVideoFrameDesc wire format."""
    stride = width * 4  # RGBA8
    data_size = len(pixel_data)
    checksum = zlib.crc32(pixel_data) & 0xFFFFFFFF
    return struct.pack(
        "<IIIBBBBdQII",
        width, height, stride,
        fmt, camera_index,
        0, 0,  # reserved[2]
        sim_time,
        frame_number,
        data_size,
        checksum,
    )


def make_color_bars(width: int, height: int) -> bytes:
    """Generate an 8-bar SMPTE-style color bar pattern (RGBA8)."""
    colors = [
        (255, 255, 255, 255),  # White
        (255, 255, 0, 255),    # Yellow
        (0, 255, 255, 255),    # Cyan
        (0, 255, 0, 255),      # Green
        (255, 0, 255, 255),    # Magenta
        (255, 0, 0, 255),      # Red
        (0, 0, 255, 255),      # Blue
        (0, 0, 0, 255),        # Black
    ]
    pixels = bytearray(width * height * 4)
    for y in range(height):
        for x in range(width):
            bar = (x * 8) // width
            idx = (y * width + x) * 4
            pixels[idx:idx + 4] = bytes(colors[bar])
    return bytes(pixels)


def save_ppm(filename: str, width: int, height: int, rgba_data: bytes):
    """Save RGBA pixel data as a PPM file (RGB, no deps needed)."""
    with open(filename, "wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode())
        for i in range(0, len(rgba_data), 4):
            f.write(rgba_data[i:i + 3])  # RGB, skip A


def parse_fragment_header(data):
    if len(data) < FRAGMENT_HEADER_SIZE:
        return None
    magic, msg_id, frag_idx, frag_count, total_size, payload_size, cksum = \
        struct.unpack("<IHBBIHH", data[:FRAGMENT_HEADER_SIZE])
    if magic != FRAGMENT_MAGIC:
        return None
    return (msg_id, frag_idx, frag_count, total_size, payload_size)


# ── Main test ────────────────────────────────────────────────────────────────

def main():
    WIDTH, HEIGHT = 128, 128
    PORT = 19200  # High port to avoid conflicts

    print(f"=== Video Transport E2E Test ===")
    print(f"Image: {WIDTH}x{HEIGHT} RGBA8 ({WIDTH * HEIGHT * 4} bytes)")
    print(f"Port: {PORT}")
    print()

    # Generate test image
    pixels = make_color_bars(WIDTH, HEIGHT)
    print(f"[1/5] Generated {len(pixels)}-byte color bar test pattern")

    # Save original for comparison
    save_ppm("video_e2e_test_sent.ppm", WIDTH, HEIGHT, pixels)
    print(f"[2/5] Saved original: video_e2e_test_sent.ppm")

    # Build wire message: [MJVideoFrameDesc (40 bytes)][pixel data]
    desc = make_video_frame_desc(WIDTH, HEIGHT, 0, 0, 1.234, 1, pixels)
    wire_msg = desc + pixels
    fragments = fragment_message(wire_msg, message_id=42)
    print(f"[3/5] Fragmented into {len(fragments)} UDP packets "
          f"(message size: {len(wire_msg)} bytes)")

    # Create sender and receiver sockets
    sender_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sender_sock.bind(("127.0.0.1", PORT))

    receiver_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_sock.settimeout(3.0)
    receiver_sock.bind(("127.0.0.1", 0))
    receiver_port = receiver_sock.getsockname()[1]

    # Receiver sends hello to register
    receiver_sock.sendto(b"\x01", ("127.0.0.1", PORT))

    # Sender reads hello to get receiver address
    hello_data, receiver_addr = sender_sock.recvfrom(64)
    print(f"[4/5] Hello received from {receiver_addr}, sending {len(fragments)} fragments...")

    # Send all fragments
    for frag in fragments:
        sender_sock.sendto(frag, receiver_addr)

    # Receive and reassemble
    reassembly_buffer = bytearray(len(wire_msg))
    received_set = set()
    expected_count = len(fragments)

    deadline = time.monotonic() + 3.0
    while len(received_set) < expected_count and time.monotonic() < deadline:
        try:
            data, _ = receiver_sock.recvfrom(MAX_UDP_PAYLOAD)
        except socket.timeout:
            break

        parsed = parse_fragment_header(data)
        if parsed is None:
            continue
        msg_id, frag_idx, frag_count, total_size, payload_size = parsed

        payload = data[FRAGMENT_HEADER_SIZE:FRAGMENT_HEADER_SIZE + payload_size]
        offset = frag_idx * MAX_FRAGMENT_PAYLOAD
        reassembly_buffer[offset:offset + len(payload)] = payload
        received_set.add(frag_idx)

    sender_sock.close()
    receiver_sock.close()

    # Verify
    reassembled = bytes(reassembly_buffer)
    rx_desc_bytes = reassembled[:VIDEO_FRAME_DESC_SIZE]
    rx_pixels = reassembled[VIDEO_FRAME_DESC_SIZE:]

    # Parse received descriptor
    w, h, stride, fmt, cam_idx, _, _, sim_time, frame_num, data_size, checksum = \
        struct.unpack("<IIIBBBBdQII", rx_desc_bytes)

    # CRC check
    computed_crc = zlib.crc32(rx_pixels) & 0xFFFFFFFF
    crc_ok = computed_crc == checksum

    # Pixel comparison
    pixels_match = (rx_pixels == pixels)

    print()
    print(f"[5/5] Results:")
    print(f"  Fragments: {len(received_set)}/{expected_count} received")
    print(f"  Descriptor: {w}x{h} stride={stride} format={fmt} cam={cam_idx}")
    print(f"  Sim time: {sim_time:.3f}s, frame #{frame_num}")
    print(f"  Data size: {data_size} bytes")
    print(f"  CRC-32: {'PASS' if crc_ok else 'FAIL'} "
          f"(expected 0x{checksum:08X}, got 0x{computed_crc:08X})")
    print(f"  Pixel match: {'PASS' if pixels_match else 'FAIL'}")

    if pixels_match and crc_ok:
        save_ppm("video_e2e_test_received.ppm", w, h, rx_pixels)
        print(f"\n  Saved received image: video_e2e_test_received.ppm")
        print(f"\n=== ALL CHECKS PASSED ===")
        print(f"\nOpen both .ppm files in an image viewer to visually verify:")
        print(f"  open video_e2e_test_sent.ppm video_e2e_test_received.ppm")
    else:
        print(f"\n=== TEST FAILED ===")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
