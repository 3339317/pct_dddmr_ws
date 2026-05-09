#!/usr/bin/env python3

import argparse
import math
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header


class TimeSyncPublisher(Node):
    def __init__(self, args):
        super().__init__("time_sync_probe_pub")
        self.args = args
        self.seq = 0
        self.pub = self.create_publisher(Header, args.topic, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_stamp)
        self.get_logger().info(
            f"Publishing stamped probe on {args.topic} at {args.rate:.1f} Hz"
        )

    def publish_stamp(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.seq)
        self.seq += 1
        self.pub.publish(msg)


class TimeSyncSubscriber(Node):
    def __init__(self, args):
        super().__init__("time_sync_probe_sub")
        self.args = args
        self.samples = []
        self.last_report_time = time.monotonic()
        self.negative_count = 0
        self.create_subscription(Header, args.topic, self.callback, 10)
        self.get_logger().info(f"Listening stamped probe on {args.topic}")

    def callback(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.stamp)
        delay_ms = (now - stamp).nanoseconds / 1e6
        if delay_ms < 0:
            self.negative_count += 1
        if math.isfinite(delay_ms):
            self.samples.append(delay_ms)
        if len(self.samples) > self.args.window:
            self.samples = self.samples[-self.args.window:]

        current_time = time.monotonic()
        if current_time - self.last_report_time >= self.args.report_period:
            self.last_report_time = current_time
            self.report()

    def report(self):
        if not self.samples:
            return
        avg = statistics.fmean(self.samples)
        min_delay = min(self.samples)
        max_delay = max(self.samples)
        std = statistics.pstdev(self.samples) if len(self.samples) > 1 else 0.0
        verdict = self.make_verdict(avg, max_delay, std)
        self.get_logger().info(
            "delay_ms avg={:.2f} min={:.2f} max={:.2f} std={:.2f} "
            "negative={} window={} => {}".format(
                avg,
                min_delay,
                max_delay,
                std,
                self.negative_count,
                len(self.samples),
                verdict,
            )
        )

    @staticmethod
    def make_verdict(avg, max_delay, std):
        if avg < 5.0 and max_delay < 20.0 and std < 5.0:
            return "GOOD for cmd_vel control"
        if avg < 20.0 and max_delay < 80.0 and std < 20.0:
            return "OK, but watch wireless jitter"
        return "BAD/UNSTABLE, fix time sync or network"


def parse_args():
    parser = argparse.ArgumentParser(
        description="ROS2 two-machine time sync and small-message delay probe."
    )
    parser.add_argument("mode", choices=["pub", "sub"], help="pub on PC, sub on S100")
    parser.add_argument("--topic", default="/time_sync_probe", help="Probe topic")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz")
    parser.add_argument("--window", type=int, default=200, help="Statistics window")
    parser.add_argument("--report-period", type=float, default=1.0, help="Report period in seconds")
    return parser.parse_args()


def main(args=None):
    parsed = parse_args()
    parsed.rate = max(0.1, float(parsed.rate))
    parsed.window = max(2, int(parsed.window))
    parsed.report_period = max(0.2, float(parsed.report_period))

    rclpy.init(args=args)
    node = TimeSyncPublisher(parsed) if parsed.mode == "pub" else TimeSyncSubscriber(parsed)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
