"""Thin ROS 2 client for the tuven_gpio controller: read the sensors, write the actuators of an IO box.

States and commands are addressed per device: client.state['sensor1']['input'], client.command['actuator1']['output']
(the ros2_control interfaces are sensor1_input and actuator1_output). Levels are raw: 1 = high voltage.
"""
import math
import time

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node

from tuven_ethercat import system


class IoClient(Node):
    """Talks to one gateway of the tuven_gpio controller (ROS parameter `gateway`, default: first in tuven_params.yaml).

    ROS parameter `device` picks one device by profile name, or `all`; unset means every device, or only the first
    actuator and its sensor when `all_by_default` is False (scripts that move something).
    Commands are only published after start_commands(), so a read-only script never writes to the box.
    """

    def __init__(self, name='tuven_client', all_by_default=True):
        super().__init__(name)
        self.params = system.load_params()
        self.gateway = self.declare_parameter('gateway', self.params['motors'][0]).value
        selected = self.declare_parameter('device', '').value
        self.profiles = system.devices(self.gateway)
        if not selected and not all_by_default:
            selected = next(d for d, p in self.profiles.items() if p['type'] == 'actuator')
        if selected and selected != 'all':
            if selected not in self.profiles:
                raise SystemExit(f'device {selected!r} is not listed by {self.gateway}: {list(self.profiles)}')
            keep = {selected}
            sensor = self.profiles[selected].get('sensor')
            if sensor:
                keep.add(sensor)
            self.profiles = {d: p for d, p in self.profiles.items() if d in keep}
        self.state = {d: {} for d, p in self.profiles.items() if p['type'] == 'sensor'}
        self.command = {d: {'output': system.command_defaults(d, p)[system.interface_name(d, p)]}
                        for d, p in self.profiles.items() if p['type'] == 'actuator'}
        self.rate = self.params['client_rate_hz']
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
            for device in self.state:
                key = f'{device}_input'
                if key in values:
                    self.state[device] = {'input': values[key]}

    def _send_command(self):
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = [self.gateway]
        names = [f'{d}_{n}' for d, command in self.command.items() for n in command]
        values = [float(v) for command in self.command.values() for v in command.values()]
        msg.interface_values = [InterfaceValue(interface_names=names, values=values)]
        self.command_pub.publish(msg)

    def has_state(self):
        """Every selected sensor reported a number (interfaces start as NaN)."""
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
        """Wait for the first state of every selected sensor, then let a few cycles through."""
        if not self.spin_until(self.has_state, timeout):
            print(f'No state from /{system.CONTROLLER}/gpio_states; is tuven.launch.py running and the slave in OP?')
            return False
        self.spin_until(lambda: False, 0.5)
        return True

    # tool_logic IO object: the sequences are tested against a fake of these five methods

    def lock(self, actuator):
        self._set_level(actuator, system.lock_level(self.profiles[actuator]))

    def unlock(self, actuator):
        self._set_level(actuator, int(self.profiles[actuator]['unlock_level']))

    def _set_level(self, actuator, level):
        self.command[actuator]['output'] = level
        self.start_commands()
        self._send_command()

    def sensor_stable(self, sensor):
        level = self.state[sensor].get('input')
        return level is not None and level == self.profiles[sensor]['stable_level']

    def now(self):
        return time.monotonic()

    def step(self, seconds):
        self.spin_until(lambda: False, seconds)
