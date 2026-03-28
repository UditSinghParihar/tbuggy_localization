#!/usr/bin/env python3
"""Monitor ROS2 bag playback progress as a percentage.

Usage:
    python3 bag_progress.py          # prompted to choose
    python3 bag_progress.py 1        # log_01_ros2
    python3 bag_progress.py 2        # log_02_ros2
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
import time

BAGS = {
    '1': {
        'name':  'log_01_ros2',
        'start': 1764838901.156430853,
        'end':   1764839450.993756635,
    },
    '2': {
        'name':  'log_02_ros2',
        'start': 1764839832.440683819,
        'end':   1764840169.182940440,
    },
}


def select_bag() -> dict:
    # If bag id passed as CLI arg
    for arg in sys.argv[1:]:
        if arg in BAGS:
            return BAGS[arg]

    # Interactive prompt
    print("Select bag to monitor:")
    for key, bag in BAGS.items():
        duration = bag['end'] - bag['start']
        print(f"  [{key}] {bag['name']}  ({format_duration(duration)})")
    choice = input("Enter 1 or 2: ").strip()
    if choice not in BAGS:
        print(f"Invalid choice '{choice}'. Exiting.")
        sys.exit(1)
    return BAGS[choice]


class BagProgressMonitor(Node):
    def __init__(self, bag: dict):
        super().__init__('bag_progress_monitor')
        self.bag_start    = bag['start']
        self.bag_duration = bag['end'] - bag['start']
        self.last_print   = 0.0
        self.sub = self.create_subscription(
            Odometry,
            '/tbuggy/odom',
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        stamp   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        elapsed = stamp - self.bag_start
        pct     = max(0.0, min(100.0, elapsed / self.bag_duration * 100.0))

        now = time.monotonic()
        if now - self.last_print >= 1.0:
            self.last_print = now
            bar = make_bar(pct)
            print(
                f"\r{bar} {pct:5.1f}%  "
                f"[{format_duration(elapsed)} / {format_duration(self.bag_duration)}]",
                end='', flush=True,
            )

        if pct >= 100.0:
            print("\nBag playback complete.")
            rclpy.shutdown()


def format_duration(secs: float) -> str:
    secs = max(0.0, secs)
    m = int(secs) // 60
    s = secs % 60
    return f"{m:02d}:{s:05.2f}"


def make_bar(pct: float, width: int = 40) -> str:
    filled = int(width * pct / 100)
    return '[' + '#' * filled + '-' * (width - filled) + ']'


def main():
    bag = select_bag()
    print(f"\nMonitoring: {bag['name']}  ({format_duration(bag['end'] - bag['start'])})")
    print("Waiting for /tbuggy/odom messages... (start ros2 bag play)\n")

    rclpy.init(args=[sys.argv[0]])  # pass only script name, not bag arg
    node = BagProgressMonitor(bag)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
