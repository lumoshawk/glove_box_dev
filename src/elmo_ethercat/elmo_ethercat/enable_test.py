"""Enable test: enable the drive, hold the current position for a while, disable it again. No motion is commanded.

Usage: ros2 run elmo_ethercat enable_test [--ros-args -p motor:=elmo_m1 -p hold_s:=2.0]
Needs `ros2 launch elmo_ethercat elmo.launch.py` running. Exit code 0 = all checks passed.

The mode is Profile Position with the target set to the measured position and no new set-point, so the
drive holds where it is. The hold is aborted if the position drifts more than max_drift_counts, and the
drive is always switched off again (control word 0), also after a failure. A fault is reset (0x80) while
trying to enable; a fault that persists is reported as FAIL.
"""
import sys

import rclpy

from elmo_ethercat.elmo_client import OP_ENABLED, STATE_MASK, SW_FAULT, ElmoClient, drive_state

MODE_PP = 1


def left_hold(node, start, max_drift):
    """True once the drive faulted, left Operation enabled, or moved too far."""
    status = node.status_word()
    return (bool(status & SW_FAULT) or status & STATE_MASK != OP_ENABLED
            or abs(int(node.state['position']) - start) > max_drift)


def main():
    rclpy.init()
    node = ElmoClient('elmo_enable_test')
    tests = node.profile['tests']['enable_test']
    hold_s = node.declare_parameter('hold_s', float(tests['hold_s'])).value
    max_drift = node.declare_parameter('max_drift_counts', int(tests['max_drift_counts'])).value
    results = {}
    try:
        if not node.spin_until(node.has_state, 10):
            print('[FAIL] no valid state received for this motor on /elmo_gpio/gpio_states')
            return 1

        start = int(node.state['position'])
        print(f'{node.motor}: {drive_state(node.status_word())} (status 0x{node.status_word():04X}), '
              f'position {start} counts')
        node.command['mode'] = MODE_PP
        node.command['target_position'] = start

        results['enabled (Operation enabled)'] = node.enable(timeout=5)
        print(f'after enable: {drive_state(node.status_word())} (status 0x{node.status_word():04X}, '
              f'error 0x{int(node.state["error_code"]):04X}, mode {int(node.state["mode_display"])})')

        if results['enabled (Operation enabled)']:
            results['mode display is PP'] = int(node.state['mode_display']) == MODE_PP
            aborted = node.spin_until(lambda: left_hold(node, start, max_drift), hold_s)
            print(f'drift while enabled: {int(node.state["position"]) - start} counts, '
                  f'current {int(node.state["current"])}, following error {int(node.state["following_error"])}')
            results[f'held {hold_s:g} s within {max_drift} counts'] = not aborted
    finally:
        # always leave the drive switched off
        results['disabled (Switch on disabled)'] = node.has_state() and node.disable(timeout=5)
        if node.has_state():
            print(f'after disable: {drive_state(node.status_word())} (status 0x{node.status_word():04X})')

    print('--- checks ---')
    for name, ok in results.items():
        print(f'[{"PASS" if ok else "FAIL"}] {name}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
