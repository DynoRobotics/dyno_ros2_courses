#!/usr/bin/env python3
#
# ROS2 cmd_vel subscriber that listens to velocity commands
#

import argparse
import struct
import time
import zenoh
import json
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import int8, int32, uint32, float64


@dataclass
class Vector3(IdlStruct, typename="Vector3"):
    x: float64
    y: float64
    z: float64


@dataclass
class Twist(IdlStruct, typename="Twist"):
    linear: Vector3
    angular: Vector3


@dataclass
class Time(IdlStruct, typename="Time"):
    sec: int32
    nanosec: uint32


@dataclass
class Log(IdlStruct, typename="Log"):
    stamp: Time
    level: int8
    name: str
    msg: str
    file: str
    function: str
    line: uint32


def parse_attachment(attach_bytes, version: int | None = None):
    """Parse rmw_zenoh_cpp attachment.

    v1 layout:
      - int64_t sequence_number (LE)
      - int64_t source_timestamp (LE)
      - uint8_t[16] source_gid

    v2 layout:
      - uint8_t version (=2)
      - v1 payload
    v3 layout:
      - uint8_t version (=3)
      - v1 payload
    """
    if not attach_bytes:
        return None, None, None, None

    buf = bytes(attach_bytes)
    total_len = len(buf)

    # Try explicit version if provided
    if version == 3 and total_len >= 1 + 8 + 8 + 16 and buf[0] == 3:
        try:
            seq, ts_ns, gid = struct.unpack("<qq16s", buf[1:1 + 8 + 8 + 16])
            return seq, ts_ns, gid.hex(), total_len
        except Exception:
            pass
    if version == 2 and total_len >= 1 + 8 + 8 + 16 and buf[0] == 2:
        try:
            seq, ts_ns, gid = struct.unpack("<qq16s", buf[1:1 + 8 + 8 + 16])
            return seq, ts_ns, gid.hex(), total_len
        except Exception:
            pass

    # Try v1
    if total_len >= 8 + 8 + 16:
        try:
            seq, ts_ns, gid = struct.unpack("<qq16s", buf[: 8 + 8 + 16])
            return seq, ts_ns, gid.hex(), total_len
        except Exception:
            pass

    # Try auto-detect tagged versions
    if total_len >= 1 + 8 + 8 + 16 and buf[0] in (2, 3):
        try:
            seq, ts_ns, gid = struct.unpack("<qq16s", buf[1:1 + 8 + 8 + 16])
            return seq, ts_ns, gid.hex(), total_len
        except Exception:
            pass

    return None, None, None, total_len


def main():
    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="ros2-sub-cmd-vel",
        description="zenoh ros2 cmd_vel subscriber example",
    )
    parser.add_argument(
        "--mode",
        "-m",
        dest="mode",
        choices=["peer", "client"],
        type=str,
        help="The zenoh session mode.",
    )
    parser.add_argument(
        "--connect",
        "-e",
        dest="connect",
        metavar="ENDPOINT",
        action="append",
        type=str,
        help="zenoh endpoints to connect to.",
        default=["tcp/127.0.0.1:7447"],
    )
    parser.add_argument(
        "--listen",
        "-l",
        dest="listen",
        metavar="ENDPOINT",
        action="append",
        type=str,
        help="zenoh endpoints to listen on.",
    )
    parser.add_argument(
        "--config",
        "-c",
        dest="config",
        metavar="FILE",
        type=str,
        help="A configuration file.",
    )
    parser.add_argument(
        "--cmd_vel",
        dest="cmd_vel",
        default="0/turtle1/safe_cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
        type=str,
        help='The "cmd_vel" ROS2 topic.',
    )
    parser.add_argument(
        "--rosout",
        dest="rosout",
        default="rt/rosout",
        type=str,
        help='The "rosout" ROS2 topic.',
    )
    parser.add_argument(
        "--show-metadata",
        action="store_true",
        help="Show message metadata (sequence, timestamp, GID).",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="Compact output format (one line per message).",
    )
    parser.add_argument(
        "--attachment-version",
        dest="attachment_version",
        choices=[1, 2],
        default=1,
        type=int,
        help="Attachment parse layout: 1 (qq16s, default) or 2 (u8+qq16s)",
    )
    parser.add_argument(
        "--dump-attachment",
        action="store_true",
        help="Hex-dump first bytes of attachment for cross-checking",
    )
    parser.add_argument(
        "--dump-payload",
        action="store_true",
        help="Hex-dump first bytes of payload to inspect CDR encapsulation",
    )

    args = parser.parse_args()
    conf = (
        zenoh.Config.from_file(args.config)
        if args.config is not None
        else zenoh.Config()
    )
    if args.mode is not None:
        conf.insert_json5("mode", json.dumps(args.mode))
    if args.connect is not None:
        conf.insert_json5("connect/endpoints", json.dumps(args.connect))
    if args.listen is not None:
        conf.insert_json5("listen/endpoints", json.dumps(args.listen))

    cmd_vel = args.cmd_vel
    rosout = args.rosout
    show_metadata = args.show_metadata
    compact = args.compact
    attachment_version = args.attachment_version
    dump_attachment = args.dump_attachment
    dump_payload = args.dump_payload

    # zenoh-net code  --- --- --- --- --- --- --- --- --- --- ---

    # initiate logging
    zenoh.init_log_from_env_or("error")

    print("Opening session...")
    session = zenoh.open(conf)

    print("Subscriber on '{}'...".format(rosout))

    def rosout_callback(sample):
        try:
            # Convert ZBytes to bytes for pycdr2 deserialization
            payload_bytes = bytes(sample.payload)
            log = Log.deserialize(payload_bytes)
            print(
                "[{}.{}] [{}]: {}".format(
                    log.stamp.sec, log.stamp.nanosec, log.name, log.msg
                )
            )
        except Exception as e:
            pass  # Silently ignore rosout deserialization errors

    rosout_sub = session.declare_subscriber(rosout, rosout_callback)

    # Message counter
    msg_count = [0]  # Use list to allow modification in nested function

    def cmd_vel_callback(sample):
        try:
            # Convert ZBytes to bytes for pycdr2 deserialization
            payload_bytes = bytes(sample.payload)

            # Optionally dump first bytes to inspect encapsulation
            if dump_payload:
                head = payload_bytes[:16].hex()
                print(f"Payload head (16B max): {head}")

            # Deserialize the Twist message (payload may include encapsulation)
            twist = Twist.deserialize(payload_bytes)
            msg_count[0] += 1

            # Parse attachment if available
            seq, ts_ns, gid, attach_len = None, None, None, None
            if hasattr(sample, "attachment") and sample.attachment:
                # Convert ZBytes attachment to bytes
                attach_bytes = bytes(sample.attachment)
                seq, ts_ns, gid, attach_len = parse_attachment(
                    attach_bytes, version=attachment_version
                )

            # Display the message
            if compact:
                # Compact format: one line
                print(
                    f"[{msg_count[0]:4d}] linear: [{twist.linear.x:6.2f}, {twist.linear.y:6.2f}, {twist.linear.z:6.2f}] "
                    f"angular: [{twist.angular.x:6.2f}, {twist.angular.y:6.2f}, {twist.angular.z:6.2f}]"
                )
            else:
                # Detailed format
                print(f"\n=== Message #{msg_count[0]} ===")
                print(f"Linear velocity:")
                print(f"  x: {twist.linear.x:8.3f} m/s")
                print(f"  y: {twist.linear.y:8.3f} m/s")
                print(f"  z: {twist.linear.z:8.3f} m/s")
                print(f"Angular velocity:")
                print(f"  x: {twist.angular.x:8.3f} rad/s")
                print(f"  y: {twist.angular.y:8.3f} rad/s")
                print(f"  z: {twist.angular.z:8.3f} rad/s")

                if show_metadata and (seq is not None or attach_len is not None):
                    print(f"Metadata:")
                    if seq is not None:
                        print(f"  Sequence: {seq}")
                    if ts_ns is not None:
                        ts_sec = ts_ns / 1e9
                        print(f"  Timestamp: {ts_sec:.6f} s ({ts_ns} ns)")
                    if gid is not None:
                        print(f"  Publisher GID (hex, len={len(gid)//2}): {gid}")
                    if attach_len is not None:
                        print(f"  Attachment length: {attach_len} bytes")
                    if dump_attachment and hasattr(sample, "attachment") and sample.attachment:
                        ab = bytes(sample.attachment)
                        head = ab[:64].hex()
                        print(f"  Attachment head (64B max): {head}")

        except Exception as e:
            print(f"Error processing message: {e}")

    print(f"Subscribing to '{cmd_vel}'...")
    print("Waiting for cmd_vel messages (press Ctrl+C to stop)...")
    if compact:
        print("Using compact output format")
    print()

    cmd_vel_sub = session.declare_subscriber(cmd_vel, cmd_vel_callback)

    # Keep the program running
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n\nReceived {msg_count[0]} messages. Exiting...")

    # Cleanup
    cmd_vel_sub.undeclare()
    rosout_sub.undeclare()
    session.close()
    print("Session closed")


if __name__ == "__main__":
    main()
