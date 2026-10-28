"""Thin ROS 2 client for one motor of the elmo_gpio controller: read states, write commands."""
import math

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node

from elmo_ethercat import system

# CiA402 control words
CW_DISABLE_VOLTAGE = 0x00
CW_SHUTDOWN = 0x06
CW_SWITCH_ON = 0x07
CW_ENABLE_OP = 0x0F
CW_FAULT_RESET = 0x80
CW_HALT = 1 << 8
CW_NEW_SETPOINT = 0x1F  # enable operation + bit 4 (new set-point)

# CiA402 status word bits
SW_FAULT = 1 << 3
SW_TARGET_REACHED = 1 << 10
SW_SETPOINT_ACK = 1 << 12

STATE_MASK = 0x6F
OP_ENABLED = 0x27
SWITCH_ON_DISABLED = 0x40


def drive_state(status_word):
    """Decode the CiA402 power state from the status word."""
    if status_word & 0x4F == 0x08:
        return 'Fault'
    return {0x00: 'Not ready to switch on', 0x40: 'Switch on disabled',
            0x21: 'Ready to switch on', 0x23: 'Switched on',
            0x27: 'Operation enabled', 0x07: 'Quick stop active',
            }.get(status_word & STATE_MASK, 'Unknown')


def next_control_word(status_word):
    """Control word that moves the CiA402 state machine one step toward Operation enabled."""
    if status_word & SW_FAULT:
        return CW_FAULT_RESET
    return {0x40: CW_SHUTDOWN, 0x21: CW_SWITCH_ON}.get(status_word & STATE_MASK, CW_ENABLE_OP)


class ElmoClient(Node):
    """Talks to the motor named by the ROS parameter `motor` (default: first in elmo_params.yaml)."""

    def __init__(self, name='elmo_client'):
        super().__init__(name)
        params = system.load_params()
        self.motor = self.declare_parameter('motor', params['motors'][0]).value
        self.profile = system.load_profile(self.motor)
        mech = self.profile['mechanics']
        self.counts_per_rev = mech['pulses_per_rev'] * mech['gear_ratio']  # per output revolution
        self.state = {}
        self.command = system.command_defaults(system.load_drive(self.profile['drive']))
        motion = self.profile['motion']
        for key, value in (('profile_velocity', motion['velocity']), ('profile_accel', motion['accel']),
                           ('profile_decel', motion['decel'])):
            self.command[key] = value
        self.create_subscription(
            DynamicInterfaceGroupValues, f'/{system.CONTROLLER}/gpio_states', self._on_state, 10)
        self.command_pub = self.create_publisher(
            DynamicInterfaceGroupValues, f'/{system.CONTROLLER}/commands', 10)
        self.create_timer(1 / params['client_rate_hz'], self._send_command)

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

    def enable(self, timeout=10):
        """Walk the CiA402 state machine to Operation enabled (resets a fault on the way)."""
        def step():
            self.command['control_word'] = next_control_word(self.status_word())
            return self.status_word() & STATE_MASK == OP_ENABLED
        return self.spin_until(step, timeout)

    def disable(self, timeout=5):
        """Switch the power stage off (Switch on disabled)."""
        def step():
            self.command['control_word'] = CW_FAULT_RESET if self.status_word() & SW_FAULT else CW_DISABLE_VOLTAGE
            return self.status_word() & STATE_MASK == SWITCH_ON_DISABLED
        return self.spin_until(step, timeout)
