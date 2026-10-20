"""Thin ROS 2 client for one motor of the mdx_gpio controller: read states, write commands."""
import csv
import math
import os
import time

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node

from mdx_ethercat import system

RECORD_DIR = os.path.join('log', 'records')  # relative to the current directory

# CiA402 control words
CW_SHUTDOWN = 0x06
CW_SWITCH_ON = 0x07
CW_ENABLE_OP = 0x0F
CW_HALT = 1 << 8
CW_NEW_SETPOINT = 0x1F  # enable operation + bit 4 (new set-point)
CW_FAULT_RESET = 0x80

# CiA402 status word bits
SW_FAULT = 1 << 3
SW_TARGET_REACHED = 1 << 10
SW_SETPOINT_ACK = 1 << 12


def drive_state(status_word):
    """Decode the CiA402 power state from the status word."""
    if status_word & 0x4F == 0x08:
        return 'Fault'
    return {0x00: 'Not ready to switch on', 0x40: 'Switch on disabled',
            0x21: 'Ready to switch on', 0x23: 'Switched on',
            0x27: 'Operation enabled', 0x07: 'Quick stop active',
            }.get(status_word & 0x6F, 'Unknown')


class MdxClient(Node):
    """Talks to the motor named by the ROS parameter `motor` (default: first in mdx_params.yaml)."""

    def __init__(self, name='mdx_client'):
        super().__init__(name)
        params = system.load_params()
        self.motor = self.declare_parameter('motor', params['motors'][0]).value
        self.profile = system.load_profile(self.motor)
        motion = self.profile['motion']
        mech = self.profile['mechanics']
        self.counts_per_rev = mech['pulses_per_rev'] * mech['gear_ratio']  # per output revolution
        self.state = {}
        self._record_sub = None
        self.command = system.command_defaults(system.load_drive(self.profile['drive']))
        for key, value in (('profile_velocity', motion['velocity']), ('profile_accel', motion['accel']),
                           ('profile_decel', motion['decel'])):
            if key in self.command:  # other drive models may not map these
                self.command[key] = value
        self.create_subscription(
            DynamicInterfaceGroupValues, '/mdx_gpio/gpio_states', self._on_state, 10)
        self.command_pub = self.create_publisher(
            DynamicInterfaceGroupValues, '/mdx_gpio/commands', 10)
        self.rate_hz = params['client_rate_hz']
        self.create_timer(1 / self.rate_hz, self._send_command)

    def _on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name == self.motor:
                self.state = dict(zip(group.interface_names, group.values))

    def _send_command(self):
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = [self.motor]
        msg.interface_values = [InterfaceValue(
            interface_names=list(self.command),
            values=[float(v) for v in self.command.values()])]
        self.command_pub.publish(msg)

    def start_recording(self, prefix, command_names=('target_position',)):
        """Record state samples and selected command values to a CSV under log/records/."""
        missing = [name for name in command_names if name not in self.command]
        if missing:
            raise ValueError(f'cannot record unknown command interface(s): {missing}')
        os.makedirs(RECORD_DIR, exist_ok=True)
        path = os.path.join(RECORD_DIR, f'{prefix}_{self.motor}_{time.strftime("%Y-%m-%d_%H-%M-%S")}.csv')
        self._record_file = open(path, 'w', newline='')
        writer = csv.writer(self._record_file)
        keys = list(self.state)
        writer.writerow(['time_s', *keys, *command_names])
        t0 = self.get_clock().now().nanoseconds

        def on_sample(msg):
            for name, group in zip(msg.interface_groups, msg.interface_values):
                if name == self.motor:
                    values = dict(zip(group.interface_names, group.values))
                    t = (self.get_clock().now().nanoseconds - t0) / 1e9
                    writer.writerow([f'{t:.4f}', *(values.get(k) for k in keys),
                                     *(self.command[name] for name in command_names)])

        self._record_sub = self.create_subscription(
            DynamicInterfaceGroupValues, '/mdx_gpio/gpio_states', on_sample, 200)
        print(f'Recording to {path}')

    def stop_recording(self):
        if self._record_sub:
            self.destroy_subscription(self._record_sub)
            self._record_file.close()
            self._record_sub = None

    def deg_to_counts(self, degrees):
        """Output-axis degrees to a drive count difference, including direction and gearing."""
        return round(degrees / 360 * self.counts_per_rev * self.profile['calibration']['direction'])

    def counts_to_deg(self, counts):
        return counts / self.counts_per_rev * 360 * self.profile['calibration']['direction']

    def position_deg(self):
        """Calibrated output position in degrees (zero_offset is the zero point)."""
        return self.counts_to_deg(self.state['position'] - self.profile['calibration']['zero_offset'])

    def spin_until(self, condition, timeout):
        """Spin until condition() is true; return False on timeout."""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not condition():
            if self.get_clock().now().nanoseconds > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def sleep(self, seconds):
        self.spin_until(lambda: False, seconds)

    def has_state(self):
        return bool(self.state) and not any(math.isnan(v) for v in self.state.values())

    def status_word(self):
        return int(self.state['status_word'])

    def enable(self, timeout=10, *, halt=False):
        """Enable the drive, optionally holding the Profile Velocity halt bit."""
        enabled_control_word = CW_ENABLE_OP | (CW_HALT if halt else 0)

        def step():
            if self.status_word() & SW_FAULT:
                self.command['control_word'] = CW_FAULT_RESET
            else:
                self.command['control_word'] = {0x40: CW_SHUTDOWN, 0x21: CW_SWITCH_ON}.get(
                    self.status_word() & 0x6F, enabled_control_word)
            return self.status_word() & 0x6F == 0x27
        return self.spin_until(step, timeout)
