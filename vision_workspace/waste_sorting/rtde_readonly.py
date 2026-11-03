"""Bounded RTDE output-only snapshot. NEVER registers inputs or sends scripts.

Protocol: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html
"""
import math
import socket
import struct
import time

from .zones import timestamp

OUTPUT_NAMES = b"timestamp,actual_q,actual_TCP_pose,tcp_offset"
OUTPUT_TYPES = b"DOUBLE,VECTOR6D,VECTOR6D,VECTOR6D"
ALLOWED_MESSAGES = frozenset((86, 118, 79, 83))


def encode_packet(kind, payload=b""):
    if kind not in ALLOWED_MESSAGES:
        raise ValueError(f"RTDE control/input message {kind} is forbidden")
    if len(payload) > 65532:
        raise ValueError("RTDE packet too large")
    return struct.pack("!HB", len(payload)+3, kind) + payload


def _read_exact(connection, size, deadline):
    data = bytearray()
    while len(data) < size:
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise TimeoutError("RTDE reply timed out")
        connection.settimeout(remaining)
        chunk = connection.recv(size-len(data))
        if not chunk:
            raise ConnectionError("RTDE connection closed before complete reply")
        data.extend(chunk)
    return bytes(data)


def _reply(connection, expected, deadline):
    for _ in range(32):
        size, kind = struct.unpack("!HB", _read_exact(connection, 3, deadline))
        if not 3 <= size <= 8192:
            raise ValueError(f"Invalid RTDE frame length {size}")
        payload = _read_exact(connection, size-3, deadline)
        if kind == expected:
            return payload
        if kind not in (77, 85):
            raise ValueError(f"Unexpected RTDE reply {kind}; expected {expected}")
    raise ValueError("Too many unrelated RTDE messages")


def decode_state(payload, recipe_id):
    if len(payload) != 153 or payload[0] != recipe_id:
        raise ValueError("RTDE output recipe or payload length mismatch")
    values = struct.unpack("!19d", payload[1:])
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite RTDE state")
    return {
        "controller_timestamp_s": values[0],
        "actual_q_rad": list(values[1:7]),
        "actual_tcp_pose_base_m_rotvec_rad": list(values[7:13]),
        "active_tcp_offset_flange_m_rotvec_rad": list(values[13:19]),
    }


def read_robot_state(host="147.250.35.40", port=30004, timeout=3.0):
    return _read_snapshot(host,port,timeout,OUTPUT_NAMES,OUTPUT_TYPES,decode_state)


TOOL_OUTPUT_NAMES = b'timestamp,actual_digital_output_bits,actual_q,actual_qd,safety_mode,robot_status_bits,runtime_state,tool_output_voltage'
TOOL_OUTPUT_TYPES = b'DOUBLE,UINT64,VECTOR6D,VECTOR6D,INT32,UINT32,UINT32,INT32'


def decode_tool_io_state(payload,recipe_id):
    layout = '!dQ12diIIi'
    if len(payload) != 1+struct.calcsize(layout) or payload[0] != recipe_id:
        raise ValueError('Tool I/O output recipe or payload length mismatch')
    v = struct.unpack(layout,payload[1:])
    if not all(math.isfinite(x) for x in v):
        raise ValueError('Non-finite tool I/O state')
    return {'controller_timestamp_s':v[0],'actual_digital_output_bits':v[1],
            'tool_output_0':bool(v[1] & (1<<16)), 'tool_output_1':bool(v[1] & (1<<17)),
            'actual_q_rad':list(v[2:8]),'actual_qd_rad_s':list(v[8:14]),
            'safety_mode':v[14],'robot_status_bits':v[15],
            'runtime_state':v[16],'tool_output_voltage_v':v[17]}


def read_tool_io_state(host='147.250.35.40',port=30004,timeout=3.0):
    """OUTPUT-only snapshot, including Tool DO0 = digital output bit 16."""
    return _read_snapshot(host,port,timeout,TOOL_OUTPUT_NAMES,TOOL_OUTPUT_TYPES,decode_tool_io_state)


def _read_snapshot(host,port,timeout,names,types,decoder):
    """Subscribe on one connection, get one state, disconnect. Whole-call deadline."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive and finite")
    started = time.monotonic()
    deadline = started + timeout
    with socket.create_connection((host, port), timeout=timeout) as connection:
        def request(kind, payload=b""):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("RTDE snapshot deadline expired")
            connection.settimeout(remaining)
            connection.sendall(encode_packet(kind, payload))
            return _reply(connection, kind, deadline)

        if request(86, struct.pack("!H", 2)) != b"\x01":
            raise ValueError("Controller rejected RTDE protocol v2")
        version = request(118)
        if len(version) != 16:
            raise ValueError("Invalid controller version reply")
        version = ".".join(map(str, struct.unpack("!4I", version)))
        recipe = request(79, struct.pack("!d", 10.0)+names)
        if len(recipe) < 2 or recipe[0] == 0 or recipe[1:] != types:
            raise ValueError(f"RTDE output fields rejected: {recipe!r}")
        if request(83) != b"\x01":
            raise ValueError("Controller refused RTDE output streaming")
        state = decoder(_reply(connection, 85, deadline), recipe[0])
        state.update({
            "robot_ip": host, "controller_version": version,
            "received_at_utc": timestamp(),
            "host_sample_monotonic_s": time.monotonic(),
            "request_duration_s": time.monotonic()-started,
            "robot_execution_enabled": False,
        })
        return state
