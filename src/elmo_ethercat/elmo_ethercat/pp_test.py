"""Profile Position test: move +move_counts from the current position, then -move_counts back.

Usage: ros2 run elmo_ethercat pp_test [--ros-args -p move_counts:=100000 -p velocity:=10000]
Needs `ros2 launch elmo_ethercat elmo.launch.py` running. THE MOTOR MOVES: clear the axis first.
Exit code 0 = all checks passed.

Targets are absolute: start + move_counts, then start again. Each move uses the CiA402 new set-point handshake
(bit 4 up, wait for the acknowledge in bit 12, bit 4 down, wait for target reached in bit 10). A move that does
not finish in time, or a fault, stops the axis with the halt bit, and the drive is always switched off at the end.
"""
import sys

import rclpy

from elmo_ethercat.elmo_client import (
    CW_ENABLE_OP, CW_HALT, CW_NEW_SETPOINT, OP_ENABLED, STATE_MASK, SW_FAULT, SW_SETPOINT_ACK, SW_TARGET_REACHED,
    ElmoClient, drive_state)

MODE_PP = 1
POSITION_TOLERANCE = 500  # counts between the final position and the target


def move_timeout(distance, velocity, accel):
    """Seconds a trapezoidal move of `distance` counts needs, with a generous margin."""
    return abs(distance) / velocity + 2 * velocity / accel + 5


def move_to(node, target, timeout, results, label):
    """Run one PP move with the set-point handshake. Returns True when the target was reached."""
    cmd = node.command
    start = int(node.state['position'])

    def failed():
        status = node.status_word()
        return bool(status & SW_FAULT) or status & STATE_MASK != OP_ENABLED

    def print_monitor():
        print(f'  position={int(node.state["position"])} counts, velocity={int(node.state["velocity"])}, '
              f'target={target}, current={int(node.state["current"])}, '
              f'following_error={int(node.state["following_error"])}, status 0x{node.status_word():04X}')

    cmd['target_position'] = target
    cmd['control_word'] = CW_NEW_SETPOINT
    acked = node.spin_until(lambda: failed() or node.status_word() & SW_SETPOINT_ACK, 5)
    cmd['control_word'] = CW_ENABLE_OP
    results[f'{label}: set-point acknowledged'] = bool(acked) and not failed()
    if not results[f'{label}: set-point acknowledged']:
        return False

    timer = node.create_timer(0.5, print_monitor)
    try:
        done = node.spin_until(lambda: failed() or node.status_word() & SW_TARGET_REACHED, timeout)
    finally:
        node.destroy_timer(timer)
    reached = bool(done) and not failed()
    results[f'{label}: target reached'] = reached
    if not reached:
        print(f'{label}: {"fault / left Operation enabled" if done else "timeout"} '
              f'(status 0x{node.status_word():04X}), halting')
        return False
    error = int(node.state['position']) - target
    print(f'{label}: {start} -> {int(node.state["position"])} counts (target {target}, error {error})')
    results[f'{label}: within {POSITION_TOLERANCE} counts of the target'] = abs(error) <= POSITION_TOLERANCE
    return True


def main():
    rclpy.init()
    node = ElmoClient('elmo_pp_test')
    cmd = node.command
    test = node.profile['tests']['pp_test']
    distance = int(node.declare_parameter('move_counts', int(test['move_counts'])).value)
    velocity = int(node.declare_parameter('velocity', int(test['velocity'])).value)
    motion = node.profile['motion']
    if distance == 0 or not 0 < velocity <= motion['max_speed']:
        raise SystemExit(f'move_counts must be nonzero and velocity in (0, motion.max_speed={motion["max_speed"]}]')
    if motion['accel'] <= 0 or motion['decel'] <= 0:
        raise SystemExit('motion.accel and motion.decel must be greater than zero')
    timeout = move_timeout(distance, velocity, min(motion['accel'], motion['decel']))

    results = {}
    try:
        if not node.spin_until(node.has_state, 10):
            print('[FAIL] no valid state received for this motor on /elmo_gpio/gpio_states')
            return 1
        origin = int(node.state['position'])
        print(f'{node.motor}: {drive_state(node.status_word())} (status 0x{node.status_word():04X}), '
              f'position {origin} counts')
        print(f'PP: +{distance} then {-distance} counts at {velocity} counts/s '
              f'(accel {motion["accel"]}, decel {motion["decel"]}), timeout {timeout:.0f} s per move')

        cmd['mode'] = MODE_PP
        cmd['profile_velocity'] = velocity
        cmd['target_position'] = origin  # hold position while enabling
        results['mode display is PP'] = node.spin_until(lambda: node.state['mode_display'] == MODE_PP, 5)
        if not results['mode display is PP']:
            return 1
        results['enabled (Operation enabled)'] = node.enable(timeout=5)
        if not results['enabled (Operation enabled)']:
            return 1

        if move_to(node, origin + distance, timeout, results, f'move {distance:+d}'):
            if move_to(node, origin, timeout, results, f'move {-distance:+d}'):
                net = int(node.state['position']) - origin
                print(f'back at the start within {net} counts')
                results[f'net displacement within {POSITION_TOLERANCE} counts'] = abs(net) <= POSITION_TOLERANCE
    finally:
        # stop whatever is still moving (halt), then always leave the drive switched off
        if node.has_state():
            if node.status_word() & STATE_MASK == OP_ENABLED and not all(results.values()):
                cmd['control_word'] = CW_ENABLE_OP | CW_HALT
                node.sleep(1.0)
            results['disabled (Switch on disabled)'] = node.disable(timeout=5)
            print(f'after disable: {drive_state(node.status_word())} (status 0x{node.status_word():04X})')

    print('--- checks ---')
    for name, ok in results.items():
        print(f'[{"PASS" if ok else "FAIL"}] {name}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
