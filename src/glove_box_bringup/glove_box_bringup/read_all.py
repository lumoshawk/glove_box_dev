"""Read every motor on the bus, across all packages, in one table. Subscribe only: nothing is commanded.

  ros2 run glove_box_bringup read_all
  ros2 run glove_box_bringup read_all --ros-args -p raw:=true     # also dump every state interface

Needs `ros2 launch glove_box_bringup all.launch.py` running.
"""
import math
import sys

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues
from rclpy.node import Node

from glove_box_bringup import bus

TIMEOUT_S = 10.0
SW_FAULT = 1 << 3
REQUIRED_FIELDS = ('status_word', 'error_code', 'mode_display', 'position', 'velocity')


def feedback_issues(state):
    """Identify missing table fields and non-finite feedback before conversion."""
    issues = [f'{name}=missing' for name in REQUIRED_FIELDS if name not in state]
    issues.extend(f'{name}={value}' for name, value in state.items()
                  if not math.isfinite(value))
    return issues


class ReadAll(Node):
    def __init__(self):
        super().__init__('glove_box_read_all')
        self.raw = self.declare_parameter('raw', False).value
        config = bus.load_config()
        self.motors = [(e['package'], e['motor']) for e in config['bus']]  # bus order
        self.profiles = {}
        self.decode = {}
        self.states = {}
        for package in bus.motors_by_package(config):
            module = bus.package_system(package)
            self.decode[package] = bus.package_client(package).drive_state
            self.create_subscription(
                DynamicInterfaceGroupValues, f'/{module.CONTROLLER}/gpio_states',
                self._on_state, 10)
        for package, motor in self.motors:
            self.profiles[motor] = bus.package_system(package).load_profile(motor)

    def _on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name in self.profiles:
                self.states[name] = dict(zip(group.interface_names, group.values))

    def complete(self):
        return all(m in self.states and not feedback_issues(self.states[m])
                   for _, m in self.motors)

    def deg(self, motor, counts):
        mech = self.profiles[motor]['mechanics']
        cal = self.profiles[motor]['calibration']
        return (counts - cal['zero_offset']) / (mech['pulses_per_rev'] * mech['gear_ratio']) * 360 * cal['direction']

    def report(self):
        ok = True
        print(f'{"motor":8} {"drive":10} {"state":22} {"status":>7} {"err":>6} {"mode":>4} '
              f'{"position":>11} {"deg":>10} {"velocity":>9}  checks')
        for package, m in self.motors:
            drive = self.profiles[m]['drive']
            s = self.states.get(m)
            if not s:
                print(f'{m:8} {drive:10} NO DATA')
                ok = False
                continue
            issues = feedback_issues(s)
            if issues:
                print(f'{m:8} {drive:10} INVALID DATA  FAIL  ' + ', '.join(issues))
                ok = False
                continue
            sw, err = int(s['status_word']), int(s['error_code'])
            good = not sw & SW_FAULT and err == 0 and sw != 0
            ok &= good
            print(f'{m:8} {drive:10} {self.decode[package](sw):22} {sw:#7x} {err:#6x} '
                  f'{int(s["mode_display"]):>4} {int(s["position"]):>11} '
                  f'{self.deg(m, s["position"]):>10.2f} {int(s["velocity"]):>9}  '
                  f'{"PASS" if good else "FAIL"}')
        if self.raw:
            for _, m in self.motors:
                print(f'--- {m} raw state interfaces ---')
                for name, value in (self.states.get(m) or {}).items():
                    print(f'{name:20s} {int(value) if math.isfinite(value) else value}')
        return ok


def main():
    rclpy.init()
    node = ReadAll()
    end = node.get_clock().now().nanoseconds + int(TIMEOUT_S * 1e9)
    while not node.complete() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
