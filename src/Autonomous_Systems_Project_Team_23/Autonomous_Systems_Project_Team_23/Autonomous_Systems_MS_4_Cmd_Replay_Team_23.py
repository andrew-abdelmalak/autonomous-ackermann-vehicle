#!/usr/bin/env python3
"""Replay Twist commands from CSV."""

import csv
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdReplayer(Node):
    def __init__(self):
        super().__init__('ms4_cmd_replay_team_23')

        self.declare_parameter('output_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter('input_path', '/tmp/ms4_cmd_record.csv')
        self.declare_parameter('rate_hz', 50.0)

        self._topic = self.get_parameter('output_topic').value
        self._input_path = self.get_parameter('input_path').value
        self._rate_hz = float(self.get_parameter('rate_hz').value)

        self._publisher = self.create_publisher(Twist, self._topic, 10)
        self._sequence = self._load_sequence(self._input_path)
        # _start_time is set on the first _tick() call so that wall-clock drift
        # during node startup (e.g., the 5 s TimerAction delay in the launch
        # file) does not cause a burst of old commands to be fired all at once.
        self._start_time = None
        self._index = 0
        self._done = False

        if not self._sequence:
            self.get_logger().error(f'No data loaded from {self._input_path}')
        else:
            duration = self._sequence[-1][0]
            self.get_logger().info(
                f'Replaying {len(self._sequence)} commands over {duration:.2f}s'
            )

        period = 1.0 / max(self._rate_hz, 1.0)
        self.create_timer(period, self._tick)

    def _load_sequence(self, path):
        """Load and normalise the CSV so the first entry always starts at t=0.

        The recorder saves ``time.monotonic()`` offsets that begin when the
        recorder node was constructed (often several seconds into the launch).
        Subtracting the first timestamp means the replayer starts playing
        immediately rather than trying to catch up to an arbitrary offset.
        """
        sequence = []
        try:
            with open(path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        t = float(row['t'])
                        v = float(row['linear_x'])
                        w = float(row['angular_z'])
                        sequence.append((t, v, w))
                    except Exception:
                        continue
        except Exception as exc:
            self.get_logger().error(f'Failed to read {path}: {exc}')
            return []

        if not sequence:
            return []

        sequence.sort(key=lambda item: item[0])

        # Normalise timestamps: shift so the first command fires at t=0.
        t_offset = sequence[0][0]
        sequence = [(t - t_offset, v, w) for (t, v, w) in sequence]
        return sequence

    def _tick(self):
        if not self._sequence or self._done:
            return

        # Initialise the wall-clock reference on the very first tick so that
        # any startup delay (launch TimerAction, DDS discovery, etc.) does not
        # manifest as a command burst.
        if self._start_time is None:
            self._start_time = time.monotonic()

        elapsed = time.monotonic() - self._start_time

        while self._index < len(self._sequence) and self._sequence[self._index][0] <= elapsed:
            _, linear_x, angular_z = self._sequence[self._index]
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self._publisher.publish(msg)
            self._index += 1

        if self._index >= len(self._sequence):
            self._done = True
            self.get_logger().info('Replay finished — sending stop command.')
            # Publish a zero-velocity command so the vehicle comes to a halt
            # instead of coasting indefinitely.
            stop = Twist()
            self._publisher.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = CmdReplayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
