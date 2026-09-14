"""Inspect/backup a DVRIP camera's encoding settings (no flight-control access).

Wire-format reference: https://github.com/OpenIPC/python-dvr
No camera writes occur in the default inspect operation.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import socket
import struct


class Camera:
    def __init__(self, host: str, user: str, password: str):
        self.sock = socket.create_connection((host, 34567), timeout=5)
        self.session = 0
        self.sequence = 0
        digest = hashlib.md5(password.encode()).digest()
        alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        password_hash = "".join(alphabet[(digest[n] + digest[n + 1]) % 62] for n in range(0, 16, 2))
        try:
            reply = self.request(1000, {"UserName": user, "PassWord": password_hash,
                                        "EncryptType": "MD5", "LoginType": "DVRIP-Web"})
            if reply.get("Ret") not in (100, 515):
                raise RuntimeError(f"Camera login rejected: {reply.get('Ret')}")
        except BaseException:
            self.close()
            raise

    def close(self):
        self.sock.close()

    def _receive(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Camera closed connection")
            data.extend(chunk)
        return data

    def request(self, code, data):
        payload = json.dumps(data, separators=(",", ":")).encode() + b"\n\0"
        header = struct.pack("<BB2xII2xHI", 255, 0, self.session, self.sequence, code, len(payload))
        self.sock.sendall(header + payload)
        marker, version, self.session, sequence, reply_code, length = struct.unpack(
            "<BB2xII2xHI", self._receive(20))
        if marker != 255 or length > 2_000_000:
            raise ValueError("Invalid camera reply")
        self.sequence += 1
        return json.loads(self._receive(length).rstrip(b"\0\n\r "))

    def get(self, name, code=1042):
        return self.request(code, {"Name": name, "SessionID": f"0x{self.session:08X}"})

    def set_encoding(self, value):
        return self.request(1040, {"Name": "Simplify.Encode",
                                  "SessionID": f"0x{self.session:08X}",
                                  "Simplify.Encode": value})


def low_latency_substream(encoding, sync_main_codec=False):
    """Keep main stream, audio, resolution and unrelated fields unchanged."""
    updated = copy.deepcopy(encoding)
    video = updated[0]["ExtraFormat"]["Video"]
    video.update(Compression="H.264", FPS=15, GOP=1, BitRate=512, BitRateControl="CBR")
    if sync_main_codec:
        updated[0]["MainFormat"]["Video"]["Compression"] = "H.264"
    return updated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", default="admin")
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--apply-h264-substream", action="store_true",
                        help="Explicitly apply H.264, 15 FPS, GOP=1 second, CBR 512 kb/s; backup first")
    parser.add_argument("--sync-main-codec", action="store_true",
                        help="Also change main stream codec to H.264 if camera requires shared codec")
    args = parser.parse_args()
    camera = Camera(args.host, args.user, os.environ.get("CAMERA_PASSWORD", ""))
    try:
        result = {}
        for name, code in (("Simplify.Encode", 1042), ("Camera", 1042), ("AVEnc", 1042),
                           ("NetWork.NetCommon", 1042),
                           ("EncodeCapability", 1360), ("SystemFunction", 1360)):
            result[name] = camera.get(name, code)
        # Exclusive creation protects an earlier known-good backup.
        with args.backup.open("x", encoding="utf-8") as target:
            json.dump(result, target, indent=2, ensure_ascii=False)
        if args.apply_h264_substream:
            original = result["Simplify.Encode"]["Simplify.Encode"]
            requested = low_latency_substream(original, args.sync_main_codec)
            reply = camera.set_encoding(requested)
            print("Write result:", json.dumps(reply))
            actual = camera.get("Simplify.Encode")
            report = {"write": reply, "readback": actual}
            with args.backup.with_suffix(".after.json").open("x", encoding="utf-8") as target:
                json.dump(report, target, indent=2, ensure_ascii=False)
            print(json.dumps(actual, indent=2, ensure_ascii=False))
            if reply.get("Ret") not in (100, 150, 602, 603):
                raise RuntimeError("Camera rejected encoding update; inspect backup and readback")
            if actual.get("Simplify.Encode") != requested:
                raise RuntimeError("Camera readback differs from requested settings; inspect .after.json")
            if reply.get("Ret") in (150, 602, 603):
                print("Settings saved; camera restart required. This tool does NOT restart the device.")
        else:
            print(json.dumps(result, indent=2, ensure_ascii=False))
    finally:
        camera.close()


if __name__ == "__main__":
    main()
