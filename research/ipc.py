"""Bounded JSON + bytes frames over a private inherited Unix socketpair.

No pickle, network listener, polling directory, or third-party transport.
"""
import json
import struct

MAX_JSON = 1024 * 1024
MAX_BINARY = 8 * 1024 * 1024


def _exact(sock, count):
    chunks = bytearray()
    while len(chunks) < count:
        chunk = sock.recv(count-len(chunks))
        if not chunk:
            raise EOFError("Inference socket closed")
        chunks.extend(chunk)
    return bytes(chunks)


def send(sock, metadata, binary=b""):
    header = json.dumps(metadata, allow_nan=False).encode()
    if len(header) > MAX_JSON or len(binary) > MAX_BINARY:
        raise ValueError("IPC frame exceeds bounds")
    sock.sendall(struct.pack("!II", len(header), len(binary)) + header + binary)


def receive(sock):
    json_len, binary_len = struct.unpack("!II", _exact(sock, 8))
    if json_len > MAX_JSON or binary_len > MAX_BINARY:
        raise ValueError("IPC frame exceeds bounds")
    metadata = json.loads(_exact(sock, json_len))
    return metadata, _exact(sock, binary_len)
