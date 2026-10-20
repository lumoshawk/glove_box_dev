"""Read every motor in eyou_params.yaml at once. Subscribe only: nothing is commanded.

  ros2 run eyou_ethercat read_all                  # summary table of all motors
  ros2 run eyou_ethercat read_all --ros-args -p raw:=true   # also dump every state interface
"""
import math
import sys

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues
from rclpy.node import Node

from eyou_ethercat import system
from eyou_ethercat.eyou_client import SW_FAULT, drive_state

TIMEOUT_S = 10.0


class ReadAll(Node):
    def __init__(self):
        super().__init__('eyou_read_all')
        self.raw = self.declare_parameter('raw', False).value
        self.motors = system.load_params()['motors']
        self.profiles = {m: system.load_profile(m) for m in self.motors}
        self.states = {}
        self.create_subscription(
            DynamicInterfaceGroupValues, '/eyou_gpio/gpio_states', self._on_state, 10)

    def _on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name in self.profiles:
                self.states[name] = dict(zip(group.interface_names, group.values))

    def complete(self):
        return all(m in self.states and not any(math.isnan(v) for v in self.states[m].values())
                   for m in self.motors)

    def deg(self, motor, counts):
        mech = self.profiles[motor]['mechanics']
        cal = self.profiles[motor]['calibration']
        per_rev = mech['pulses_per_rev'] * mech['gear_ratio']
        return (counts - cal['zero_offset']) / per_rev * 360 * cal['direction']

    def report(self):
        ok = True
        print(f'{"motor":8} {"drive":10} {"state":22} {"status":>7} {"err":>6} {"mode":>4} '
              f'{"position":>11} {"deg":>10} {"velocity":>9}  checks')
        for m in self.motors:
            s = self.states.get(m)
            if not s:
                print(f'{m:8} {self.profiles[m]["drive"]:10} NO DATA')
                ok = False
                continue
            sw, err = int(s['status_word']), int(s['error_code'])
            good = not sw & SW_FAULT and err == 0 and sw != 0
            ok &= good
            print(f'{m:8} {self.profiles[m]["drive"]:10} {drive_state(sw):22} {sw:#7x} {err:#6x} '
                  f'{int(s["mode_display"]):>4} {int(s["position"]):>11} '
                  f'{self.deg(m, s["position"]):>10.2f} {int(s["velocity"]):>9}  '
                  f'{"PASS" if good else "FAIL"}')
        if self.raw:
            for m in self.motors:
                print(f'--- {m} raw state interfaces ---')
                for name, value in (self.states.get(m) or {}).items():
                    print(f'{name:20s} {int(value)}')
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
