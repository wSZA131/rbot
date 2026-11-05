"""
rlai_teleop/keyboard_teleop.py

WASD / arrow-key teleoperation for rbot.

Publishes geometry_msgs/TwistStamped on /cmd_vel because the Jazzy
diff_drive_controller in this stack expects stamped velocity commands
(velocity_smoother is configured with enable_stamped_cmd_vel: true).

Key map:
  w / up arrow      forward
  s / down arrow    backward
  a / left arrow    turn left
  d / right arrow   turn right
  space or k        stop
  q / e             slower / faster
  Ctrl+C            quit

Diagonals are formed by pressing two orthogonal keys within COMBO_WINDOW
seconds of each other, for example 'w' then 'a' produces a forward-left arc.
A single key press keeps the classic behaviour (pure straight or pure spin).

Usage (simulation, sim clock):
  ros2 run rlai_teleop keyboard_teleop --ros-args -p use_sim_time:=true

Usage (real robot, wall clock):
  ros2 run rlai_teleop keyboard_teleop

Change the speed limits without editing code:
  ros2 run rlai_teleop keyboard_teleop --ros-args \
    -p use_sim_time:=true -p linear_speed:=0.5 -p angular_speed:=1.2
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

# Escape sequences produced by the arrow keys (ESC [ A/B/C/D).
ARROW_KEYS = {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left'}

# Terminal key -> logical direction.
DIRECTION_KEYS = {
    'w': 'forward', 'up': 'forward',
    's': 'back', 'down': 'back',
    'a': 'left', 'left': 'left',
    'd': 'right', 'right': 'right',
}

# Sorted pairs of orthogonal directions that collapse into a diagonal move.
DIAGONAL_PAIRS = (
    ('back', 'left'),
    ('back', 'right'),
    ('forward', 'left'),
    ('forward', 'right'),
)

HELP = """
rbot keyboard teleop — publishing TwistStamped on /cmd_vel

  w / up arrow      forward
  s / down arrow    backward
  a / left arrow    turn left
  d / right arrow   turn right

  Diagonals — press two keys together (within 0.2 s):
    w + a   forward-left arc        w + d   forward-right arc
    s + a   backward-left arc       s + d   backward-right arc

  space or k        stop
  q / e             slower / faster
  Ctrl+C            quit
"""

# One press of q/e multiplies the speed limits by this factor.
SPEED_STEP = 1.25
MIN_SCALE = 0.2
MAX_SCALE = 3.0

# Two orthogonal direction keys pressed within this window form a diagonal.
COMBO_WINDOW = 0.2


class KeyboardTeleop(Node):
    """Reads single key presses from the terminal and streams velocity commands."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('combo_window', COMBO_WINDOW)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        topic = str(self.get_parameter('cmd_vel_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._combo_window = float(self.get_parameter('combo_window').value)

        self._scale = 1.0
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_direction = None
        self._last_direction_time = 0.0

        self._publisher = self.create_publisher(TwistStamped, topic, 10)
        self._period = 1.0 / max(publish_rate, 1.0)

        self.get_logger().info(
            f'keyboard_teleop ready on {topic} '
            f'(linear {self._linear_speed:.2f} m/s, angular {self._angular_speed:.2f} rad/s)'
        )

    # ------------------------------------------------------------------ input

    @staticmethod
    def _read_key(fd: int):
        """Return the pressed key, 'quit', or None when nothing is pending."""
        readable, _, _ = select.select([fd], [], [], 0.0)
        if not readable:
            return None

        data = os.read(fd, 1)
        if not data:
            return None

        if data == b'\x1b':
            # Arrow keys arrive as ESC [ <letter>; give the rest a moment.
            if not select.select([fd], [], [], 0.02)[0]:
                return None
            sequence = os.read(fd, 2)
            return ARROW_KEYS.get(sequence[-1:])

        if data in (b'\x03', b'\x04'):  # Ctrl+C / Ctrl+D without a TTY signal
            return 'quit'

        return data.decode('utf-8', errors='ignore').lower()

    def _handle_key(self, key: str) -> None:
        direction = DIRECTION_KEYS.get(key)
        if direction is not None:
            self._handle_direction(direction)
            return

        if key in (' ', 'k'):
            self._last_direction = None
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            self._log_command('stop     ')
        elif key == 'e':
            self._scale = min(self._scale * SPEED_STEP, MAX_SCALE)
            self._report_speed()
        elif key == 'q':
            self._scale = max(self._scale / SPEED_STEP, MIN_SCALE)
            self._report_speed()

    def _is_combo(self, previous, direction: str, now: float) -> bool:
        """True when 'direction' completes a diagonal started by 'previous'."""
        if previous is None or previous == direction:
            return False
        if (now - self._last_direction_time) > self._combo_window:
            return False
        return tuple(sorted((previous, direction))) in DIAGONAL_PAIRS

    def _handle_direction(self, direction: str) -> None:
        """Apply a direction, merging with the previous key when pressed as a combo."""
        now = time.monotonic()
        previous = self._last_direction

        if self._is_combo(previous, direction, now):
            self._apply_diagonal(previous, direction)
            # Consume the pair so a third key press starts a new command.
            self._last_direction = None
            return

        self._last_direction = direction
        self._last_direction_time = now
        self._apply_single(direction)

    def _apply_single(self, direction: str) -> None:
        linear = self._linear_speed * self._scale
        angular = self._angular_speed * self._scale

        if direction == 'forward':
            self._cmd_linear = linear
            self._cmd_angular = 0.0
        elif direction == 'back':
            self._cmd_linear = -linear
            self._cmd_angular = 0.0
        elif direction == 'left':
            self._cmd_linear = 0.0
            self._cmd_angular = angular
        else:  # right
            self._cmd_linear = 0.0
            self._cmd_angular = -angular

        self._log_command(f'{direction:8s} ')

    def _apply_diagonal(self, first: str, second: str) -> None:
        linear_direction = first if first in ('forward', 'back') else second
        angular_direction = first if first in ('left', 'right') else second

        linear = self._linear_speed * self._scale
        angular = self._angular_speed * self._scale

        self._cmd_linear = linear if linear_direction == 'forward' else -linear
        self._cmd_angular = angular if angular_direction == 'left' else -angular

        self._log_command(f'{linear_direction}-{angular_direction} ')

    def _report_speed(self) -> None:
        self.get_logger().info(
            f'speed scale x{self._scale:.2f} '
            f'(linear {self._linear_speed * self._scale:.2f} m/s, '
            f'angular {self._angular_speed * self._scale:.2f} rad/s)'
        )

    def _log_command(self, label: str) -> None:
        self.get_logger().info(
            f'{label} linear {self._cmd_linear:+.2f} m/s   '
            f'angular {self._cmd_angular:+.2f} rad/s'
        )

    # ----------------------------------------------------------------- output

    def _publish(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = self._cmd_linear
        message.twist.angular.z = self._cmd_angular
        self._publisher.publish(message)

    def _stop(self) -> None:
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0

    # -------------------------------------------------------------------- run

    def run(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error(
                'keyboard_teleop needs an interactive terminal (stdin is not a TTY)'
            )
            return

        fd = sys.stdin.fileno()
        previous_attributes = termios.tcgetattr(fd)
        print(HELP, flush=True)

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                key = self._read_key(fd)
                if key == 'quit':
                    break
                if key:
                    self._handle_key(key)

                self._publish()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(self._period)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, previous_attributes)
            self._stop()
            self._publish()
            print('\nkeyboard_teleop stopped', flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
