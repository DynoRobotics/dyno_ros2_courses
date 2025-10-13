#!/usr/bin/env python3
#
# ROS2 cmd_vel publisher that publishes at regular intervals
#

import argparse
import os
import struct
import time
import zenoh
import json
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import int8, int32, uint32, float64

# Import ROS 2 message definitions
try:
    from geometry_msgs.msg import Twist as ROS2Twist, Vector3 as ROS2Vector3
    from builtin_interfaces.msg import Time as ROS2Time
    from rcl_interfaces.msg import Log as ROS2Log
    ROS2_MSGS_AVAILABLE = True
except ImportError:
    ROS2_MSGS_AVAILABLE = False
    print("Warning: ROS 2 message definitions not available. Using manual definitions.")

# Global publisher GID (stable per process)
# Align with current rmw_zenoh_cpp build expecting 16-byte GID
PUB_GID = os.urandom(16)
seq = 0

# ROS 2 metadata constants
ADMIN_SPACE = "@ros2_lv"
DOMAIN_ID = 0
ENTITY_PUBLISHER = "MP"
KEYEXPR_DELIMITER = "/"
SLASH_REPLACEMENT = "%"

def mangle_name(name: str) -> str:
    """Replace "/" instances with "%" for Zenoh keyexpr compatibility."""
    return name.replace("/", SLASH_REPLACEMENT)

def qos_to_keyexpr(reliability: int = 1, durability: int = 2, history: int = 1, depth: int = 10) -> str:
    """Convert QoS settings to keyexpr format.
    
    Format: <reliability>:<durability>:<history>,<depth>:<deadline_sec>,<deadline_nsec>:<lifespan_sec>,<lifespan_nsec>:<liveliness>,<liveliness_sec>,<liveliness_nsec>
    Default values match ROS 2 defaults.
    """
    return f"{reliability}:{durability}:{history},{depth}:0,0:0,0:1,0,0"

def create_liveliness_keyexpr(
    zid: str,
    nid: str,
    entity_id: str,
    node_namespace: str = "",
    node_name: str = "zenoh_publisher",
    topic_name: str = "turtle1/safe_cmd_vel",
    topic_type: str = "geometry_msgs::msg::dds_::Twist_",
    topic_type_hash: str = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
    qos: str = None
) -> str:
    """Create ROS 2 liveliness token keyexpr for publisher metadata."""
    if qos is None:
        qos = qos_to_keyexpr()
    
    # Build keyexpr parts - must include ALL parts for non-node entities
    parts = [
        ADMIN_SPACE,                    # 0: AdminSpace
        str(DOMAIN_ID),                 # 1: DomainId
        zid,                           # 2: Zid
        nid,                           # 3: Nid
        entity_id,                     # 4: Id
        ENTITY_PUBLISHER,              # 5: EntityStr
        "_",                           # 6: Enclave (placeholder for empty)
        "_",                           # 7: Namespace (placeholder for empty)
        mangle_name(node_name),        # 8: NodeName
        mangle_name(topic_name),       # 9: TopicName
        mangle_name(topic_type),       # 10: TopicType
        mangle_name(topic_type_hash),  # 11: TopicTypeHash
        qos                            # 12: TopicQoS
    ]
    
    # Join all parts
    return KEYEXPR_DELIMITER.join(parts)


def build_attachment(sequence: int, version: int = 3) -> bytes:
    """Build rmw_zenoh_cpp-compatible attachment.
    
    rmw_zenoh_cpp uses zenoh::ext::Serializer which serializes:
    - int64_t sequence_number (8 bytes LE)
    - int64_t source_timestamp (8 bytes LE)
    - std::array<uint8_t, 16> source_gid (VarInt(16) + 16 bytes = 17 bytes)
    
    Total: 8 + 8 + 17 = 33 bytes
    
    But backtrace shows len=24, so try different layouts.
    """
    ts_ns = int(time.time_ns())
    
    if version == 3:
        # Try Zenoh serialization format: seq + ts + VarInt(16) + gid
        # This should be 8 + 8 + 1 + 16 = 33 bytes
        leb128_len = b'\x10'  # VarInt(16) = 0x10
        return struct.pack("<qq", sequence, ts_ns) + leb128_len + PUB_GID
    elif version == 2:
        # Try seq + VarInt(16) + gid = 8 + 1 + 16 = 25 bytes (close to 24)
        leb128_len = b'\x10'  # VarInt(16) = 0x10
        return struct.pack("<q", sequence) + leb128_len + PUB_GID
    elif version == 1:
        # Try ts + VarInt(16) + gid = 8 + 1 + 16 = 25 bytes (close to 24)
        leb128_len = b'\x10'  # VarInt(16) = 0x10
        return struct.pack("<q", ts_ns) + leb128_len + PUB_GID
    else:
        # Fallback to v3
        leb128_len = b'\x10'  # VarInt(16) = 0x10
        return struct.pack("<qq", sequence, ts_ns) + leb128_len + PUB_GID


def maybe_encapsulate(body: bytes, encapsulate: bool, kind: str) -> bytes:
    """Prepend CDR encapsulation header when requested.

    kind:
    - "xcdr1": 0x0001 0x0000 → b"\x00\x01\x00\x00"
    - "xcdr2": 0x0003 0x0000 → b"\x00\x03\x00\x00" (PL_CDR_LE)
    """
    if not encapsulate:
        return body
    if kind == "xcdr2":
        return b"\x00\x03\x00\x00" + body
    return b"\x00\x01\x00\x00" + body


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


# Define pycdr2 classes - either manually or derived from ROS 2 messages
if ROS2_MSGS_AVAILABLE:
    # Method 1: Use ROS 2 messages directly with pycdr2 serialization
    # This is the cleanest approach - use ROS 2 messages and serialize them with pycdr2
    def create_twist_from_ros2(linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                              angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Create a Twist message using ROS 2 message classes."""
        ros2_twist = ROS2Twist()
        ros2_twist.linear.x = linear_x
        ros2_twist.linear.y = linear_y
        ros2_twist.linear.z = linear_z
        ros2_twist.angular.x = angular_x
        ros2_twist.angular.y = angular_y
        ros2_twist.angular.z = angular_z
        return ros2_twist
    
    def serialize_ros2_message(message):
        """Serialize a ROS 2 message using pycdr2."""
        # Convert ROS 2 message to pycdr2-compatible format
        if isinstance(message, ROS2Twist):
            twist = Twist(
                linear=Vector3(x=message.linear.x, y=message.linear.y, z=message.linear.z),
                angular=Vector3(x=message.angular.x, y=message.angular.y, z=message.angular.z)
            )
            return twist.serialize()
        else:
            raise ValueError(f"Unsupported message type: {type(message)}")
    
    print("Using ROS 2 message classes with pycdr2 serialization")
else:
    # Fallback: Manual pycdr2 class definitions
    def create_twist_from_ros2(linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                              angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Create a Twist message using manual pycdr2 classes."""
        return Twist(
            linear=Vector3(x=linear_x, y=linear_y, z=linear_z),
            angular=Vector3(x=angular_x, y=angular_y, z=angular_z)
        )
    
    def serialize_ros2_message(message):
        """Serialize a pycdr2 message."""
        return message.serialize()
    
    print("Using manual pycdr2 class definitions")


def main():
    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="ros2-pub-cmd-vel", description="zenoh ros2 cmd_vel publisher example"
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
        default=["tcp/172.18.0.2:7447"],  # 👈 Default endpoint here
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
        "--linear",
        dest="linear",
        default="0.5",
        type=float,
        help="The linear velocity (m/s).",
    )
    parser.add_argument(
        "--angular",
        dest="angular",
        default="0.0",
        type=float,
        help="The angular velocity (rad/s).",
    )
    parser.add_argument(
        "--rate",
        "-r",
        dest="rate",
        default="2.0",
        type=float,
        help="Publishing rate in Hz.",
    )
    parser.add_argument(
        "--duration",
        "-d",
        dest="duration",
        default="0",
        type=float,
        help="Duration to publish in seconds (0 = infinite).",
    )
    parser.add_argument(
        "--no-encap",
        dest="no_encap",
        action="store_true",
        help="Disable XCDR1 LE encapsulation header (for debugging)",
    )
    parser.add_argument(
        "--attachment-version",
        dest="attachment_version",
        choices=[1, 2, 3],
        default=3,
        type=int,
        help="Attachment layout version: 1 (qq16s) or 2/3 (u8+qq16s, default=3)",
    )
    parser.add_argument(
        "--encap",
        dest="encap",
        choices=["xcdr1", "xcdr2"],
        default="xcdr1",
        type=str,
        help="CDR encapsulation kind to prepend (default: xcdr1)",
    )
    parser.add_argument(
        "--zenoh-encoding",
        dest="zenoh_encoding",
        default="application/x-cdr",
        type=str,
        help="Zenoh encoding to set on the payload (default: application/x-cdr)",
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
    linear = args.linear
    angular = args.angular
    rate = args.rate
    duration = args.duration
    no_encap = args.no_encap
    attachment_version = args.attachment_version
    encap_kind = args.encap
    zenoh_encoding = args.zenoh_encoding

    # zenoh-net code  --- --- --- --- --- --- --- --- --- --- ---

    # initiate logging
    zenoh.init_log_from_env_or("error")

    print("Opening session...")
    session = zenoh.open(conf)

    # Create ROS 2 liveliness token for metadata publishing
    print("Creating ROS 2 liveliness token...")
    zid = str(session.info.zid())  # Get Zenoh session ID
    nid = "1"  # Node ID (arbitrary, but consistent)
    entity_id = "1"  # Entity ID (arbitrary, but unique)
    
    # Extract topic name from cmd_vel (which is the full DDS interop key)
    # cmd_vel format: "0/turtle1/safe_cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_..."
    # We need the full ROS 2 topic name: "/turtle1/safe_cmd_vel"
    topic_name = "/turtle1/safe_cmd_vel"  # Use the correct ROS 2 topic name
    
    liveliness_keyexpr = create_liveliness_keyexpr(
        zid=zid,
        nid=nid,
        entity_id=entity_id,
        node_namespace="",
        node_name="zenoh_publisher",
        topic_name=topic_name,
        topic_type="geometry_msgs::msg::dds_::Twist_",
        topic_type_hash="RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
    )
    
    print(f"Liveliness keyexpr: {liveliness_keyexpr}")
    
    # Declare liveliness token
    token = session.liveliness().declare_token(zenoh.KeyExpr(liveliness_keyexpr))
    print("ROS 2 metadata liveliness token declared")

    print("Subscriber on '{}'...".format(rosout))

    def rosout_callback(sample):
        log = Log.deserialize(sample.payload)
        print(
            "[{}.{}] [{}]: {}".format(
                log.stamp.sec, log.stamp.nanosec, log.name, log.msg
            )
        )

    sub = session.declare_subscriber(rosout, rosout_callback)

    def pub_twist(linear_vel, angular_vel):
        global seq
        # Use the new approach that works with both ROS 2 messages and manual definitions
        if ROS2_MSGS_AVAILABLE:
            # Create ROS 2 message and serialize it
            ros2_twist = create_twist_from_ros2(
                linear_x=float(linear_vel), linear_y=0.0, linear_z=0.0,
                angular_x=0.0, angular_y=0.0, angular_z=float(angular_vel)
            )
            body = serialize_ros2_message(ros2_twist)
        else:
            # Use manual pycdr2 classes
            t = create_twist_from_ros2(
                linear_x=float(linear_vel), linear_y=0.0, linear_z=0.0,
                angular_x=0.0, angular_y=0.0, angular_z=float(angular_vel)
            )
            body = serialize_ros2_message(t)
        
        # pycdr2 produces raw CDR data, don't add encapsulation headers
        # rmw_zenoh_cpp will handle the encapsulation internally
        payload = body
        attach = build_attachment(seq, version=attachment_version)
        seq += 1
        # zenoh-python ≥1.0 takes attachment as bytes

        session.put(
            cmd_vel,
            payload,
            encoding=zenoh.Encoding(zenoh_encoding),
            attachment=attach,
        )

    print(f"Publishing on '{cmd_vel}' at {rate} Hz")
    print(f"Linear velocity: {linear} m/s, Angular velocity: {angular} rad/s")
    if duration > 0:
        print(f"Duration: {duration} seconds")
    else:
        print("Duration: infinite (press Ctrl+C to stop)")

    # Calculate sleep time based on rate
    sleep_time = 1.0 / rate if rate > 0 else 0.1

    start_time = time.time()
    count = 0

    try:
        while True:
            # Check duration
            if duration > 0 and (time.time() - start_time) >= duration:
                print(f"\nPublished {count} messages in {duration} seconds")
                break

            # Publish twist message
            pub_twist(linear, angular)
            count += 1

            if count % int(rate) == 0:  # Print status every second
                print(
                    f"Published {count} messages (linear: {linear}, angular: {angular})"
                )

            # Sleep to maintain rate
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\nStopped. Published {count} messages")

    finally:
        # Send stop command before closing
        print("Sending stop command...")
        pub_twist(0.0, 0.0)

        # Clean up resources
        sub.undeclare()
        token.undeclare()  # Clean up liveliness token
        session.close()
        print("Session closed")


if __name__ == "__main__":
    main()
