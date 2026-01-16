#!/usr/bin/env python3
"""
Web visual test for ICM-20948 orientation output.

- Serves icm/viz/index.html over HTTP
- Streams quaternion samples over WebSocket as JSON

Data source can be:
  1) Live sensor: spawn icm/icm20948_log.py --format jsonl --out -
  2) Replay JSONL file: --file path.jsonl
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import os
import pathlib
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import AsyncIterator, Dict, Optional, Set


@dataclass
class SharedStreamState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    seq: int = 0
    last_json: str = ""

    def update(self, msg: Dict[str, object]) -> None:
        data = json.dumps(msg, separators=(",", ":"))
        with self.lock:
            self.seq += 1
            self.last_json = data

    def snapshot(self) -> tuple[int, str]:
        with self.lock:
            return self.seq, self.last_json


def _serve_http(root: pathlib.Path, host: str, port: int, state: SharedStreamState) -> ThreadingHTTPServer:
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(root), **kwargs)

        def do_GET(self) -> None:
            if self.path.split("?", 1)[0] == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()

                last_seq = -1
                try:
                    while True:
                        seq, payload = state.snapshot()
                        if seq != last_seq and payload:
                            self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_seq = seq
                        time.sleep(0.05)
                except Exception:
                    return

            return super().do_GET()

        def log_message(self, fmt: str, *args) -> None:
            # quieter
            sys.stderr.write("[http] " + (fmt % args) + "\n")

    httpd = ThreadingHTTPServer((host, port), Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd


async def _ws_broadcast_loop(
    clients: Set[object],
    source: AsyncIterator[Dict[str, object]],
    state: SharedStreamState,
) -> None:
    async for msg in source:
        state.update(msg)
        if not clients:
            continue
        data = json.dumps(msg, separators=(",", ":"))
        dead = []
        for c in clients:
            try:
                await c.send(data)
            except Exception:
                dead.append(c)
        for c in dead:
            clients.discard(c)


async def _source_replay_file(path: str, playback_hz: float | None) -> AsyncIterator[Dict[str, object]]:
    delay = (1.0 / playback_hz) if playback_hz and playback_hz > 0 else 0.0
    last_ts: Optional[float] = None
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            # Optional real-time-ish playback using timestamps
            if delay <= 0.0 and "t_s" in obj:
                ts = float(obj["t_s"])
                if last_ts is not None:
                    dt = ts - last_ts
                    if dt > 0:
                        await asyncio.sleep(min(dt, 0.05))
                last_ts = ts
            elif delay > 0.0:
                await asyncio.sleep(delay)
            yield obj


async def _source_live_subprocess(cmd: list[str]) -> AsyncIterator[Dict[str, object]]:
    # Ensure unbuffered Python child so we get timely lines.
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        env=env,
    )

    assert proc.stdout is not None
    assert proc.stderr is not None

    def _drain_stderr() -> None:
        for l in proc.stderr:
            sys.stderr.write("[imu] " + l)

    threading.Thread(target=_drain_stderr, daemon=True).start()

    try:
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            yield obj
            await asyncio.sleep(0)  # let event loop breathe
    finally:
        with contextlib.suppress(Exception):
            proc.terminate()


async def main_async() -> int:
    ap = argparse.ArgumentParser(description="ICM-20948 cube visualizer server (HTTP + WebSocket).")
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--ws-host", default="0.0.0.0")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--file", default=None, help="Replay an existing JSONL file instead of reading the sensor.")
    ap.add_argument("--playback-hz", type=float, default=None, help="Replay speed (Hz) when using --file.")
    ap.add_argument("--use-mag", action="store_true", help="Use magnetometer in fusion when running live.")
    ap.add_argument("--include-mag", action="store_true", help="Include magnetometer fields in streamed data when running live.")
    ap.add_argument("--rate", type=float, default=100.0, help="IMU sample rate for live mode.")
    ap.add_argument("--calibrate-seconds", type=float, default=2.0, help="Calibration duration for live mode.")
    args = ap.parse_args()

    # Local imports (dependency)
    try:
        import websockets
    except Exception as e:
        sys.stderr.write("Missing dependency: pip3 install websockets\n")
        sys.stderr.write(f"Import error: {e}\n")
        return 2

    root = pathlib.Path(__file__).resolve().parent
    state = SharedStreamState()
    httpd = _serve_http(root, host=args.http_host, port=args.http_port, state=state)
    sys.stderr.write(f"[http] Serving {root}/index.html on http://{args.http_host}:{args.http_port}\n")
    sys.stderr.write(f"[ws]   WebSocket on ws://{args.ws_host}:{args.ws_port}\n")
    sys.stderr.write(f"[http] SSE  on http://{args.http_host}:{args.http_port}/stream\n")

    clients: Set[websockets.WebSocketServerProtocol] = set()

    async def ws_handler(websocket):
        clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        finally:
            clients.discard(websocket)

    if args.file:
        source = _source_replay_file(args.file, playback_hz=args.playback_hz)
    else:
        # Live mode: spawn the IMU logger, streaming JSONL on stdout
        cmd = [
            sys.executable,
            "-u",
            str((root.parent / "icm20948_log.py").resolve()),
            "--rate",
            str(args.rate),
            "--calibrate-seconds",
            str(args.calibrate_seconds),
            "--format",
            "jsonl",
            "--out",
            "-",
        ]
        if args.use_mag:
            cmd.append("--use-mag")
        if args.include_mag:
            cmd.append("--include-mag")
        source = _source_live_subprocess(cmd)

    ws_server = await websockets.serve(ws_handler, args.ws_host, args.ws_port)
    try:
        await _ws_broadcast_loop(clients, source, state=state)
    finally:
        ws_server.close()
        await ws_server.wait_closed()
        httpd.shutdown()
    return 0


def main() -> int:
    try:
        return asyncio.run(main_async())
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())


