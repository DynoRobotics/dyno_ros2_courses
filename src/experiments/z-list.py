#!/usr/bin/env python3
"""
zenoh_list_keys.py — minimal 'list keys' helper for Zenoh 1.5.x

How it works:
  - Subscribes to one or more key expressions (wildcards allowed)
  - Collects unique keys that appear during a time window
  - Prints the deduplicated, sorted list at the end

Examples:
  # See all keys for 5 seconds
  python zenoh_list_keys.py

  # Focus on ROS 2 via rmw_zenoh (topics + services) for 10s
  python zenoh_list_keys.py -e /rt/** /rq/** /rr/** -d 10

  # Keep streaming keys live (no final summary)
  python zenoh_list_keys.py -l

Requires:
  pip install zenoh   (use a version matching your environment; 1.0+ API)
"""

import argparse
import signal
import sys
import threading
import time
from typing import List, Set
import json

import zenoh


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="List observed Zenoh keys by subscribing for a short time window.")
    p.add_argument(
        "-e", "--expr",
        nargs="+",
        default=["**"],
        help="Key expressions to subscribe to (default: **). Example: rt/** rq/** rr/**"
    )
    p.add_argument(
        "-d", "--duration",
        type=float,
        default=5.0,
        help="How long to listen (seconds). Ignored with --live."
    )
    p.add_argument(
        "-l", "--live",
        action="store_true",
        help="Live mode: print keys as they arrive and do not exit automatically."
    )
    p.add_argument(
        "--show-values",
        action="store_true",
        help="Also print a short preview of the payload value (may be noisy)."
    )
    p.add_argument(
        "--no-dedup",
        action="store_true",
        help="Do not deduplicate keys (prints every occurrence)."
    )
    p.add_argument(
        "--scout",
        action="store_true",
        help="Also print discovered peers/routers via zenoh.scout()."
    )
    p.add_argument(
        "--connect",
        "-c",
        dest="connect",
        metavar="ENDPOINT",
        action="append",
        type=str,
        default=["tcp/172.18.0.2:7447"],   # 👈 Default endpoint here
        help="Endpoints to connect to (default: tcp/0.0.0.0:7447). "
             "Use multiple --connect to add more."
    )
    return p.parse_args()


def main():
    args = parse_args()

    # Open Zenoh session with default config (inherits env vars if set)
    cfg = zenoh.Config()
    cfg.insert_json5("connect/endpoints", json.dumps(args.connect))
    session = zenoh.open(cfg)

    # Optional: show some environment info
    info = session.info
    print(f"# Session ZID: {session.zid()}")
    print(f"# Known routers: {list(info.routers_zid())}")
    print(f"# Known peers:   {list(info.peers_zid())}")

    if args.scout:
        print("# Scouting (peer|router) for ~1s...")
        scout = zenoh.scout(what="peer|router")
        # Stop the scout automatically after 1s
        timer = threading.Timer(1.0, scout.stop)
        timer.start()
        for hello in scout:
            print(f"#  SCOUT: {hello}")
        timer.cancel()

    # Thread-safe set of keys we’ve seen
    seen: Set[str] = set()
    lock = threading.Lock()

    def on_sample(sample: zenoh.Sample):
        key = str(sample.key_expr)
        if args.no_dedup:
            if args.show_values:
                print(f"{key}  |  {sample.value.payload[:80]!r}")
            else:
                print(key)
            return

        with lock:
            if key not in seen:
                seen.add(key)
                if args.live:
                    # In live mode, print as we discover new keys
                    if args.show_values:
                        print(f"{key}  |  {sample.value.payload[:80]!r}")
                    else:
                        print(key)

    # Declare subscribers for all requested expressions
    subs = []
    try:
        for expr in args.expr:
            sub = session.declare_subscriber(expr, on_sample)
            subs.append(sub)
    except Exception as e:
        print(f"Failed to declare subscriber(s): {e}", file=sys.stderr)
        session.close()
        sys.exit(2)

    # Handle Ctrl-C cleanly
    stop_event = threading.Event()

    def _sigint(_sig, _frm):
        stop_event.set()
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    # Wait for duration or forever if live
    if args.live:
        print("# Live mode: press Ctrl-C to stop.")
        while not stop_event.is_set():
            time.sleep(0.1)
    else:
        time.sleep(max(0.0, args.duration))

    # Cleanup subscribers
    for s in subs:
        try:
            s.undeclare()
        except Exception:
            pass

    # Final summary (non-live & dedup only)
    if not args.live and not args.no_dedup:
        print("\n# === Unique keys observed ===")
        for k in sorted(seen):
            print(k)

    session.close()


if __name__ == "__main__":
    main()
