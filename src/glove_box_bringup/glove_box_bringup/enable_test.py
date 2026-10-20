"""Enable every motor on the bus for 1 s, one at a time, without commanding motion.

  ros2 run glove_box_bringup enable_test
  ros2 run glove_box_bringup enable_test --ros-args -p motors:="[mdx_x2, eyou_r1]" -p hold_s:=2.0

Needs `ros2 launch glove_box_bringup all.launch.py` running. Per motor: the target position is set to
the measured position, velocity/torque/offset targets stay at their drive-yaml default of 0, and the
mode stays at the drive default. The enable is aborted if the position drifts more than max_drift_deg.
The drive is always disabled again (control word 0) before moving to the next motor or exiting.
Faults are reset (0x80) while trying to enable; a drive whose fault persists is reported as FAIL.
"""
import sys

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node

from glove_box_bringup import bus
from glove_box_bringup.read_all import feedback_issues

FEEDBACK_TIMEOUT_S = 10.0
ENABLE_TIMEOUT_S = 5.0
DISABLE_TIMEOUT_S = 2.0
PUBLISH_HZ = 50
SW_FAULT = 1 << 3
STATE_MASK = 0x6F
OP_ENABLED = 0x27
READY_TO_SWITCH_ON = 0x21
SWITCH_ON_DISABLED = 0x40


def next_control_word(cw, status_word):
    """Control word that moves the CiA402 state machine one step toward Operation enabled.

    `cw` is a package's client module (mdx_client / eyou_client) providing the CW_* constants.
    """
    if status_word & SW_FAULT:
        return cw.CW_FAULT_RESET
    return {SWITCH_ON_DISABLED: cw.CW_SHUTDOWN, READY_TO_SWITCH_ON: cw.CW_SWITCH_ON}.get(
        status_word & STATE_MASK, cw.CW_ENABLE_OP)


def drift_deg(position, start, counts_per_rev):
    return abs(position - start) / counts_per_rev * 360


class EnableTest(Node):
    def __init__(self):
        super().__init__('glove_box_enable_test')
        self.hold_s = self.declare_parameter('hold_s', 1.0).value
        self.max_drift_deg = self.declare_parameter('max_drift_deg', 0.5).value
        selected = [m for m in self.declare_parameter('motors', ['']).value if m]
        config = bus.load_config()
        self.motors = [(e['package'], e['motor']) for e in config['bus']]  # bus order
        unknown = sorted(set(selected) - {m for _, m in self.motors})
        if unknown:
            raise SystemExit(f'unknown motor(s) {unknown}; bus has {[m for _, m in self.motors]}')
        if selected:
            self.motors = [(p, m) for p, m in self.motors if m in selected]

        self.profiles = {}
        self.states = {}
        self.commands = {}
        self.publishers_by_package = {}
        self.active = None  # (package, motor) whose command is published
        for package in {p for p, _ in self.motors}:
            controller = bus.package_system(package).CONTROLLER
            self.create_subscription(
                DynamicInterfaceGroupValues, f'/{controller}/gpio_states', self._on_state, 10)
            self.publishers_by_package[package] = self.create_publisher(
                DynamicInterfaceGroupValues, f'/{controller}/commands', 10)
        for package, motor in self.motors:
            system = bus.package_system(package)
            self.profiles[motor] = system.load_profile(motor)
            self.commands[motor] = system.command_defaults(system.load_drive(self.profiles[motor]['drive']))
        self.create_timer(1 / PUBLISH_HZ, self._publish)

    def _on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name in self.profiles:
                self.states[name] = dict(zip(group.interface_names, group.values))

    def _publish(self):
        if not self.active:
            return
        package, motor = self.active
        command = self.commands[motor]
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = [motor]
        msg.interface_values = [InterfaceValue(
            interface_names=list(command), values=[float(v) for v in command.values()])]
        self.publishers_by_package[package].publish(msg)

    def spin_until(self, condition, timeout):
        """Spin until condition() is true; return False on timeout."""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not condition():
            if self.get_clock().now().nanoseconds > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def feedback_complete(self):
        return all(m in self.states and not feedback_issues(self.states[m]) for _, m in self.motors)

    def status_word(self, motor):
        return int(self.states[motor]['status_word'])

    def wait_for_controller(self):
        return self.spin_until(
            lambda: all(p.get_subscription_count() > 0 for p in self.publishers_by_package.values()), 5.0)

    def test_motor(self, package, motor):
        """Enable one motor for hold_s and disable it again; returns a result dict."""
        cw = bus.package_client(package)
        profile = self.profiles[motor]
        counts_per_rev = profile['mechanics']['pulses_per_rev'] * profile['mechanics']['gear_ratio']
        command = self.commands[motor]
        start = int(self.states[motor]['position'])
        result = {'enabled': False, 'held': False, 'disabled': False, 'drift': 0.0, 'note': ''}

        if 'target_position' in command:
            command['target_position'] = start
        self.active = (package, motor)
        try:
            def step():
                command['control_word'] = next_control_word(cw, self.status_word(motor))
                return self.status_word(motor) & STATE_MASK == OP_ENABLED
            result['enabled'] = self.spin_until(step, ENABLE_TIMEOUT_S)
            if not result['enabled']:
                result['note'] = f'not enabled within {ENABLE_TIMEOUT_S:g} s'
                return result

            def violation():
                state = self.states[motor]
                result['drift'] = max(result['drift'], drift_deg(state['position'], start, counts_per_rev))
                if result['drift'] > self.max_drift_deg:
                    result['note'] = f'moved {result["drift"]:.2f} deg, aborted'
                    return True
                if self.status_word(motor) & STATE_MASK != OP_ENABLED:
                    result['note'] = 'left operation enabled'
                    return True
                return False
            result['held'] = not self.spin_until(violation, self.hold_s) and not result['note']
            return result
        finally:
            result['disabled'] = self.disable(cw, motor, command)
            self.active = None
            if not result['disabled']:
                result['note'] = (result['note'] + '; ' if result['note'] else '') + 'not back to switch on disabled'

    def disable(self, cw, motor, command):
        def settled(*states):
            sw = self.status_word(motor)
            return bool(sw & SW_FAULT) or sw & STATE_MASK in states
        command['control_word'] = cw.CW_SHUTDOWN
        self.spin_until(lambda: settled(READY_TO_SWITCH_ON, SWITCH_ON_DISABLED), DISABLE_TIMEOUT_S)
        command['control_word'] = 0
        self.spin_until(lambda: settled(SWITCH_ON_DISABLED), DISABLE_TIMEOUT_S)
        return self.status_word(motor) & STATE_MASK == SWITCH_ON_DISABLED

    def run(self):
        if not self.wait_for_controller():
            print('NO CONTROLLER: nothing subscribes to the command topic; is all.launch.py running?')
            return False
        if not self.spin_until(self.feedback_complete, FEEDBACK_TIMEOUT_S):
            bad = [m for _, m in self.motors if m not in self.states or feedback_issues(self.states[m])]
            print(f'NO VALID FEEDBACK from {bad}; run read_all first. Nothing was enabled.')
            return False

        print(f'{"motor":8} {"drive":10} {"enabled":>7} {"held":>5} {"disabled":>8} {"drift deg":>9} '
              f'{"err":>6}  result')
        ok = True
        for package, motor in self.motors:
            result = self.test_motor(package, motor)
            good = result['enabled'] and result['held'] and result['disabled']
            ok &= good
            err = int(self.states[motor].get('error_code', 0))
            print(f'{motor:8} {self.profiles[motor]["drive"]:10} {_yn(result["enabled"]):>7} '
                  f'{_yn(result["held"]):>5} {_yn(result["disabled"]):>8} {result["drift"]:>9.2f} '
                  f'{err:#6x}  {"PASS" if good else "FAIL " + result["note"]}', flush=True)
        return ok


def _yn(value):
    return 'yes' if value else 'no'


def main():
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)  # Ctrl-C must still let us disable
    node = None
    try:
        node = EnableTest()
        ok = node.run()
    except KeyboardInterrupt:
        print('interrupted; drive disabled')
        ok = False
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
