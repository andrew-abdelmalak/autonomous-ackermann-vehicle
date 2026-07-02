#!/usr/bin/env python3
"""
MS3 Gamepad Teleop – Team 23
2B GP076 wireless gamepad → serial → Arduino

Reads joystick directly via Linux /dev/input/js0 (no ros-jazzy-joy needed).
Runs as a standalone ROS 2 node — no extra packages required.

DEFAULT MAPPING (2B GP076 / PS2-style pad in PC mode):
  Left  stick  Y  (axis 1)  →  speed     (push up = forward)
  Left  stick  X  (axis 0)  →  steering  (push right = steer right)
  Cross / A button (btn 0)  →  EMERGENCY STOP (toggle)
  L1 (btn 4)                →  SLOW mode  (hold: 0.25 m/s max)
  R1 (btn 5)                →  FAST mode  (hold: 1.0  m/s max)
  Select / Back (btn 8)     →  QUIT

RUN:
  ros2 run Autonomous_Systems_Project_Team_23 joy_teleop_team_23

  # To identify your axes first (prints raw values):
  ros2 run Autonomous_Systems_Project_Team_23 joy_teleop_team_23 \\
      --ros-args -p debug_axes:=true -p serial_forwarding_enabled:=false

  # Full run on car:
  ros2 run Autonomous_Systems_Project_Team_23 joy_teleop_team_23 \\
      --ros-args -p serial_port:=/dev/ttyUSB0
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import serial as pyserial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

try:
    import struct
    STRUCT_OK = True
except ImportError:
    STRUCT_OK = False


# ── Linux joystick event reader ──────────────────────────────────────────
# Format: 4-byte timestamp, 2-byte value, 1-byte type, 1-byte number
# type: 0x01 = button, 0x02 = axis
JS_EVENT_FMT  = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_TYPE_BUTTON = 0x01
JS_TYPE_AXIS   = 0x02
JS_TYPE_INIT   = 0x80  # initial state events (OR'd)


class JoystickReader:
    """Reads raw events from /dev/input/jsX in a background thread."""

    def __init__(self, device='/dev/input/js0'):
        self.device  = device
        self.axes    = [0.0] * 16
        self.buttons = [0] * 32
        self._alive  = True
        self._fd     = None
        self._lock   = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        try:
            self._fd = open(self.device, 'rb')
            while self._alive:
                data = self._fd.read(JS_EVENT_SIZE)
                if not data or len(data) < JS_EVENT_SIZE:
                    time.sleep(0.01)
                    continue
                _, value, etype, number = struct.unpack(JS_EVENT_FMT, data)
                etype &= ~JS_TYPE_INIT  # strip init flag
                with self._lock:
                    if etype == JS_TYPE_AXIS and number < len(self.axes):
                        # Normalize: raw is -32767..32767
                        self.axes[number] = value / 32767.0
                    elif etype == JS_TYPE_BUTTON and number < len(self.buttons):
                        self.buttons[number] = value
        except Exception:
            pass

    def get(self):
        with self._lock:
            return list(self.axes), list(self.buttons)

    def close(self):
        self._alive = False
        if self._fd:
            try:
                self._fd.close()
            except Exception:
                pass


# ── ROS 2 Node ──────────────────────────────────────────────────────────
class JoyTeleopTeam23(Node):

    def __init__(self):
        super().__init__('joy_teleop_team_23')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('joystick_device', '/dev/input/js0')
        self.declare_parameter('speed_axis', 1)          # Left stick Y
        self.declare_parameter('steer_axis', 0)          # Left stick X
        self.declare_parameter('speed_axis_invert', True)  # fwd=negative on most pads
        self.declare_parameter('steer_axis_invert', False)
        self.declare_parameter('estop_button', 0)        # Cross/A
        self.declare_parameter('slow_button', 4)         # L1
        self.declare_parameter('fast_button', 5)         # R1
        self.declare_parameter('quit_button', 8)         # Select/Back
        self.declare_parameter('max_speed_normal', 0.6)  # m/s default
        self.declare_parameter('max_speed_slow', 0.25)   # m/s with L1
        self.declare_parameter('max_speed_fast', 1.0)    # m/s with R1
        self.declare_parameter('max_turn_rate', 0.5)     # rad
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('speed_smoothing', 0.20)  # 0=instant
        self.declare_parameter('serial_forwarding_enabled', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('cmd_vel_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter('debug_axes', False)      # print raw values

        # Load params
        joy_dev        = self.get_parameter('joystick_device').value
        self.spd_axis  = self.get_parameter('speed_axis').value
        self.str_axis  = self.get_parameter('steer_axis').value
        self.spd_inv   = self.get_parameter('speed_axis_invert').value
        self.str_inv   = self.get_parameter('steer_axis_invert').value
        self.estop_btn = self.get_parameter('estop_button').value
        self.slow_btn  = self.get_parameter('slow_button').value
        self.fast_btn  = self.get_parameter('fast_button').value
        self.quit_btn  = self.get_parameter('quit_button').value
        self.spd_norm  = self.get_parameter('max_speed_normal').value
        self.spd_slow  = self.get_parameter('max_speed_slow').value
        self.spd_fast  = self.get_parameter('max_speed_fast').value
        self.str_max   = self.get_parameter('max_turn_rate').value
        self.deadzone  = self.get_parameter('deadzone').value
        self.smooth    = self.get_parameter('speed_smoothing').value
        serial_en      = self.get_parameter('serial_forwarding_enabled').value
        serial_port    = self.get_parameter('serial_port').value
        serial_baud    = self.get_parameter('serial_baudrate').value
        publish_cmd    = self.get_parameter('publish_cmd_vel').value
        cmd_vel_topic  = self.get_parameter('cmd_vel_topic').value
        self.debug     = self.get_parameter('debug_axes').value

        # ── State ─────────────────────────────────────────────────────
        self._estopped   = False
        self._smooth_spd = 0.0
        self._running    = True

        # Track button edges (only trigger on press, not hold)
        self._prev_btns  = [0] * 32

        # ── Joystick ──────────────────────────────────────────────────
        self._js = None
        try:
            self._js = JoystickReader(joy_dev)
            self.get_logger().info(f'Joystick opened: {joy_dev}')
        except Exception as e:
            self.get_logger().error(f'Joystick error: {e}')

        # ── Serial ────────────────────────────────────────────────────
        self._serial = None
        if serial_en and SERIAL_OK:
            try:
                self._serial = pyserial.Serial(serial_port, serial_baud, timeout=1)
                time.sleep(2)
                # background reader
                threading.Thread(target=self._reader, daemon=True).start()
                self.get_logger().info(f'Serial: {serial_port} @ {serial_baud}')
            except Exception as e:
                self.get_logger().error(f'Serial: {e}')
        elif not serial_en:
            self.get_logger().info('Serial forwarding disabled (debug mode)')

        # ── Cmd_vel publisher ─────────────────────────────────────────
        self._cmd_pub = None
        if publish_cmd:
            self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().info(f'Publishing cmd_vel on {cmd_vel_topic}')

        # ── 20 Hz control timer ────────────────────────────────────────
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'Joy Teleop ready | speed_axis={self.spd_axis} '
            f'steer_axis={self.str_axis} | '
            f'normal={self.spd_norm} slow={self.spd_slow} fast={self.spd_fast} m/s')
        self.get_logger().info(
            'Cross/A=ESTOP  L1=slow  R1=fast  Select=quit')
        if self.debug:
            self.get_logger().warn(
                'DEBUG MODE: raw axes will print. Identify your axes then remove -p debug_axes:=true')

    # ── 20 Hz tick ───────────────────────────────────────────────────
    def _tick(self):
        if not self._js:
            return

        axes, buttons = self._js.get()

        # ── Debug dump ────────────────────────────────────────────────
        if self.debug:
            a_str = '  '.join(f'A{i}={v:+.2f}' for i, v in enumerate(axes[:8]))
            b_str = '  '.join(f'B{i}' for i, v in enumerate(buttons[:12]) if v)
            self.get_logger().info(f'{a_str}   pressed:[{b_str}]')

        # ── Button edge detection ─────────────────────────────────────
        def pressed(idx):
            return (idx is not None and idx >= 0 and idx < len(buttons) and buttons[idx]
                    and not self._prev_btns[idx])

        # Quit
        if pressed(self.quit_btn):
            self.get_logger().info('Quit pressed — stopping')
            self._stop()
            rclpy.shutdown()
            return

        # E-stop toggle on press edge
        if pressed(self.estop_btn):
            self._estopped = not self._estopped
            if self._estopped:
                self.get_logger().warn('ESTOP ACTIVE — press Cross/A again to release')
            else:
                self.get_logger().info('ESTOP released')
            self._stop()

        self._prev_btns = list(buttons)

        if self._estopped:
            self._stop()
            return

        # ── Speed limit ───────────────────────────────────────────────
        if self.slow_btn >= 0 and self.slow_btn < len(buttons) and buttons[self.slow_btn]:
            max_spd = self.spd_slow
        elif self.fast_btn >= 0 and self.fast_btn < len(buttons) and buttons[self.fast_btn]:
            max_spd = self.spd_fast
        else:
            max_spd = self.spd_norm

        # ── Read axes ─────────────────────────────────────────────────
        raw_spd = axes[self.spd_axis] if self.spd_axis >= 0 and self.spd_axis < len(axes) else 0.0
        raw_str = axes[self.str_axis] if self.str_axis >= 0 and self.str_axis < len(axes) else 0.0

        # Deadzone
        if abs(raw_spd) < self.deadzone:
            raw_spd = 0.0
        if abs(raw_str) < self.deadzone:
            raw_str = 0.0

        # Invert
        if self.spd_inv:
            raw_spd = -raw_spd
        if self.str_inv:
            raw_str = -raw_str

        # Scale
        target_spd = raw_spd * max_spd
        cmd_str    = raw_str * self.str_max

        # Smooth speed to avoid motor jerks
        self._smooth_spd = (self.smooth * self._smooth_spd
                            + (1.0 - self.smooth) * target_spd)

        # ── Send serial ───────────────────────────────────────────────
        cmd = f'SPD:{self._smooth_spd:.3f},STR:{cmd_str:.3f}\n'
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'Serial write: {e}')

        if self._cmd_pub:
            msg = Twist()
            msg.linear.x = float(self._smooth_spd)
            msg.angular.z = float(cmd_str)
            self._cmd_pub.publish(msg)

    def _stop(self):
        self._smooth_spd = 0.0
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(b'SPD:0.000,STR:0.000\n')
            except Exception:
                pass
        if self._cmd_pub:
            msg = Twist()
            self._cmd_pub.publish(msg)

    def _reader(self):
        """Print Arduino ACK/MEAS lines to logger."""
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    line = self._serial.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'Arduino: {line}')
            except Exception:
                pass
            time.sleep(0.02)

    def destroy_node(self):
        self._running = False
        self._stop()
        if self._serial:
            self._serial.close()
        if self._js:
            self._js.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopTeam23()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
