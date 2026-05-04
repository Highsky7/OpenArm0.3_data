#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FMVLA dummy ZeroMQ client.

Run from the OpenArm0.3_data root:
    cd /home/highsky/OpenArm0.3_data
    python3 src/vla_server_inference/fmvla_dummy_client.py --port 5555 --rate 30
"""

import argparse
import sys
import time
from typing import Dict

import numpy as np

try:
    import msgpack
    import zmq
except ImportError as exc:
    missing = exc.name or str(exc)
    print(
        f"Missing dependency: {missing}. Install server communication deps with "
        "`pip install pyzmq msgpack numpy` or run this script in the OpenArm VLA Python environment.",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


CAMERA_KEYS = ("top", "wrist_left", "wrist_right")


def make_dummy_images(image_size: int, step: int) -> Dict[str, bytes]:
    """Create deterministic RGB images matching server payload shape."""
    coords = np.linspace(0, 255, image_size, dtype=np.uint8)
    x_grid = np.tile(coords[None, :], (image_size, 1))
    y_grid = np.tile(coords[:, None], (1, image_size))

    images = {}
    for idx, key in enumerate(CAMERA_KEYS):
        offset = np.uint8((step * 3 + idx * 53) % 256)
        channel_r = (x_grid + offset).astype(np.uint8)
        channel_g = (y_grid + np.uint8(idx * 37)).astype(np.uint8)
        blue_offset = np.uint8((step + idx * 19) % 256)
        channel_b = ((x_grid // 2 + y_grid // 2) + blue_offset).astype(np.uint8)
        rgb = np.stack([channel_r, channel_g, channel_b], axis=-1)
        images[f"observation.images.{key}"] = np.ascontiguousarray(rgb).tobytes()

    return images


def make_dummy_state(step: int) -> list[float]:
    """Create a stable 16-dim proprioceptive state."""
    base = np.zeros(16, dtype=np.float32)
    base[0] = np.sin(step * 0.01) * 0.01
    base[8] = np.cos(step * 0.01) * 0.01
    return base.tolist()


def make_request(image_size: int, task: str, step: int) -> dict:
    request = {
        "observation.state": make_dummy_state(step),
        "task": task,
    }
    request.update(make_dummy_images(image_size, step))
    return request


def connect_socket(context: zmq.Context, port: int, timeout_ms: int) -> zmq.Socket:
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.connect(f"tcp://localhost:{port}")
    return socket


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send dummy FMVLA inputs through the existing SSH tunnel.")
    parser.add_argument("--port", type=int, default=5555, help="Local ZeroMQ port from SSH tunnel.")
    parser.add_argument("--rate", type=float, default=30.0, help="Request rate in Hz.")
    parser.add_argument("--image-size", type=int, default=256, help="Square RGB image size.")
    parser.add_argument("--timeout-ms", type=int, default=30000, help="ZeroMQ send/receive timeout.")
    parser.add_argument("--duration-sec", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--max-requests", type=int, default=0, help="0 means no request-count limit.")
    parser.add_argument(
        "--task",
        type=str,
        default="Put the umbrellas into the basket",
        help="Task text sent to FMVLA.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be > 0")
    if args.image_size <= 0:
        raise ValueError("--image-size must be > 0")

    context = zmq.Context()
    socket = connect_socket(context, args.port, args.timeout_ms)
    period = 1.0 / args.rate
    start_time = time.time()
    request_count = 0
    success_count = 0
    total_rtt_ms = 0.0

    print("[FMVLA_DUMMY_CLIENT]")
    print(f"server=tcp://localhost:{args.port}")
    print(f"rate_hz={args.rate}")
    print(f"image_size={args.image_size}")
    print(f"task={args.task}")
    print("payload=3 RGB images + 16-dim observation.state + task")
    print("stop=Ctrl+C")

    try:
        while True:
            loop_start = time.time()
            elapsed = loop_start - start_time
            if args.duration_sec > 0 and elapsed >= args.duration_sec:
                break
            if args.max_requests > 0 and request_count >= args.max_requests:
                break

            request = make_request(args.image_size, args.task, request_count)
            packed = msgpack.packb(request, use_bin_type=True)

            try:
                send_start = time.time()
                socket.send(packed)
                response = msgpack.unpackb(socket.recv(), raw=False)
                rtt_ms = (time.time() - send_start) * 1000.0
            except zmq.Again:
                print("[FMVLA_DUMMY_TIMEOUT] reconnecting ZeroMQ socket")
                socket.close(linger=0)
                socket = connect_socket(context, args.port, args.timeout_ms)
                continue

            request_count += 1
            total_rtt_ms += rtt_ms

            if response.get("status") == "ok":
                success_count += 1
                action = response.get("action", [])
                server_ms = response.get("inference_time_ms", 0.0)
                print(
                    "[FMVLA_DUMMY_RESPONSE] "
                    f"request={request_count} status=ok action_dim={len(action)} "
                    f"rtt_ms={rtt_ms:.1f} server_ms={server_ms:.1f} payload_kb={len(packed) / 1024:.1f}"
                )
            else:
                print(
                    "[FMVLA_DUMMY_RESPONSE] "
                    f"request={request_count} status={response.get('status')} "
                    f"message={response.get('message', 'N/A')}"
                )

            sleep_sec = period - (time.time() - loop_start)
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    except KeyboardInterrupt:
        print("\n[FMVLA_DUMMY_CLIENT_STOP] Ctrl+C")
    finally:
        avg_rtt = total_rtt_ms / request_count if request_count else 0.0
        print("[FMVLA_DUMMY_SUMMARY]")
        print(f"requests={request_count}")
        print(f"success={success_count}")
        print(f"avg_rtt_ms={avg_rtt:.1f}")
        socket.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
