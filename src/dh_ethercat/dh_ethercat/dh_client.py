"""Thin ROS 2 client for the dh_gpio controller: read the states of the RS485 devices, write their commands.

States and commands are addressed per device: client.state['gripper1']['position'],
client.command['gripper1']['target_position'] (the ros2_control interface is gripper1_target_position).
"""
import math
import time

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node

from dh_ethercat import system

GRIP_STATES = {-1: 'init incomplete', 0: 'moving', 1: 'reached, no object',
               2: 'object gripped', 3: 'object dropped'}
INIT_STATES = {0: 'not initialised', 1: 'initialised', 2: 'initialising'}
HOME_STATES = {0: 'not homed', 1: 'homed', 2: 'homing'}
MOTION_STATES = {-1: 'homing incomplete', 0: 'moving', 1: 'reached', 2: 'stalled before target'}
NOTES = {  # (actuator type, state name) -> decoder
    ('gripper', 'init_state'): INIT_STATES, ('gripper', 'grip_state'): GRIP_STATES,
    ('gripper', 'rotation_init_state'): INIT_STATES,
    ('ecylinder', 'home_state'): HOME_STATES, ('ecylinder', 'motion_state'): MOTION_STATES,
}


class DhClient(Node):
    """Talks to one gateway of the dh_gpio controller (ROS parameter `gateway`, default: first in dh_params.yaml).

    ROS parameter `device` picks one RS485 device by profile name, or `all`. Unset it means every device of the
    gateway, or only the first one when `all_by_default` is False (scripts that move actuators).
    Commands are only published after start_commands(), so a read-only script never writes to the gateway.
    """

    def __init__(self, name='dh_client', all_by_default=True):
        super().__init__(name)
        params = system.load_params()
        self.gateway = self.declare_parameter('gateway', params['motors'][0]).value
        selected = self.declare_parameter('device', '').value
        self.gateway_cfg = system.load_gateway(self.gateway)
        self.drive = system.load_drive(self.gateway_cfg['drive'])
        self.profiles = system.devices(self.gateway)
        if not selected and not all_by_default:
            selected = next(iter(self.profiles))
        if selected and selected != 'all':
            if selected not in self.profiles:
                raise SystemExit(f'device {selected!r} is not listed by {self.gateway}: {list(self.profiles)}')
            self.profiles = {selected: self.profiles[selected]}
        self.state = {d: {} for d in self.profiles}
        self.command = {d: {n[len(d) + 1:]: v for n, v in
                            system.command_defaults(d, p, self.drive).items()}
                        for d, p in self.profiles.items()}
        self.rate = params['client_rate_hz']
        self.create_subscription(
            DynamicInterfaceGroupValues, f'/{system.CONTROLLER}/gpio_states', self._on_state, 10)
        self.command_pub = self.create_publisher(
            DynamicInterfaceGroupValues, f'/{system.CONTROLLER}/commands', 10)
        self.timer = None

    def start_commands(self):
        if self.timer is None:
            self.timer = self.create_timer(1 / self.rate, self._send_command)

    def _on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name != self.gateway:
                continue
            values = dict(zip(group.interface_names, group.values))
            for device in self.profiles:
                prefix = device + '_'
                self.state[device] = {k[len(prefix):]: v for k, v in values.items() if k.startswith(prefix)}

    def _send_command(self):
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = [self.gateway]
        names = [f'{d}_{n}' for d, command in self.command.items() for n in command]
        values = [float(v) for command in self.command.values() for v in command.values()]
        msg.interface_values = [InterfaceValue(interface_names=names, values=values)]
        self.command_pub.publish(msg)

    def has_state(self):
        """Every selected device reported, and every value is a number (interfaces start as NaN)."""
        return all(s and all(math.isfinite(v) for v in s.values()) for s in self.state.values())

    def spin_until(self, condition, timeout):
        """Spin until condition() is true; return False on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
        return False

    def wait_for_states(self, timeout=10.0):
        """Wait for the first state of every selected device, then let a few cycles through."""
        if not self.spin_until(self.has_state, timeout):
            print(f'No state from /{system.CONTROLLER}/gpio_states; is dh.launch.py running and the slave in OP?')
            return False
        self.spin_until(lambda: False, 0.5)
        return True

    def describe(self, device, name, value):
        decoder = NOTES.get((self.profiles[device]['type'], name))
        return decoder.get(int(value), '') if decoder else ''


def require_gripper(client, device):
    kind = client.profiles[device]['type']
    if kind != 'gripper':
        raise SystemExit(f'{device} is a {kind}: this test only drives grippers')


def home_gripper(client, device, timeout):
    """Write initialize = 1 and wait for init_state = 1. Returns True when the gripper reports initialised."""
    command, state = client.command[device], lambda: client.state[device]
    command['initialize'] = 1
    # the gripper reports "initialising" (2) for 0.5..3 s; give it a moment to start before looking for 1
    client.spin_until(lambda: state()['init_state'] == 2, 1.5)
    ok = client.spin_until(lambda: state()['init_state'] == 1, timeout)
    command['initialize'] = 0
    return ok
