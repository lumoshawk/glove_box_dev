"""Profile Velocity test: run at a fraction of the maximum speed forward, then backward, and repeat.

Usage: ros2 run elmo_ethercat pv_test [--ros-args -p speed_percent:=50.0 -p duration_s:=2.0 -p cycles:=1]
Needs `ros2 launch elmo_ethercat elmo.launch.py` running. THE MOTOR MOVES: clear the axis first.
Exit code 0 = all checks passed.

One cycle is `duration_s` at +speed, then `duration_s` at -speed, so the axis ends close to where it started.
Speed is speed_percent of motion.max_speed (counts/s) in the profile; ramps use motion.accel / motion.decel.
The run is aborted (target velocity 0, drive disabled) on a fault, when the drive leaves Operation enabled, or
when the measured speed exceeds 1.5 x the target. The drive is always switched off again at the end.
"""
from collections import deque
import sys
import time

import rclpy

from elmo_ethercat.elmo_client import OP_ENABLED, STATE_MASK, SW_FAULT, ElmoClient, drive_state

MODE_PV = 3
MAX_DURATION_S = 5.0
MAX_CYCLES = 10
RATE_WINDOW_S = 0.5      # position-derived speed is averaged over this window
SPEED_TOLERANCE = 0.2    # steady speed must be within +/-20 % of the target
TRAVEL_MIN = 0.5         # each leg must cover at least half of speed x duration
NET_MAX = 0.2            # back and forth must end within 20 % of one leg's travel from the start
STOP_TIMEOUT_S = 5.0


def window_rate(samples, window_s):
    """Counts/s over the newest `window_s` of (time, position) samples; None until that span is covered.

    Older samples are dropped, keeping the one at the start of the window.
    """
    if not samples:
        return None
    t_end, p_end = samples[-1]
    start = None
    for sample in samples:
        if sample[0] <= t_end - window_s:
            start = sample
        else:
            break
    if start is None:
        return None
    while samples[0] is not start:
        samples.popleft()
    return (p_end - start[1]) / (t_end - start[0])


def speed_ok(rate, target, tolerance=SPEED_TOLERANCE):
    return rate is not None and abs(rate - target) <= abs(target) * tolerance


def stop_threshold(speed):
    return max(50, speed * 0.05)


def run(node, results, samples, speed, duration, cycles):
    """Enable in PV mode and run the legs; returns early (with the failed check recorded) on any problem."""
    cmd = node.command

    def position():
        return int(node.state['position'])

    def rate():
        return window_rate(samples, RATE_WINDOW_S)

    def abort_reason(target):
        status = node.status_word()
        if status & SW_FAULT:
            return f'fault (status 0x{status:04X}, error 0x{int(node.state["error_code"]):04X})'
        if status & STATE_MASK != OP_ENABLED:
            return f'left Operation enabled (status 0x{status:04X})'
        measured = rate()
        if measured is not None and abs(measured) > abs(target) * 1.5 + 200:
            return f'overspeed {measured:.0f} counts/s, target {target}'
        return None

    def monitor_line():
        measured = rate()
        print(f'  position={position()} counts, speed={"-" if measured is None else f"{measured:.0f}"} counts/s '
              f'(drive reports {int(node.state["velocity"])}), target={int(cmd["target_velocity"])}, '
              f'current={int(node.state["current"])}, following_error={int(node.state["following_error"])}')

    cmd['mode'] = MODE_PV
    cmd['target_velocity'] = 0
    cmd['target_position'] = position()
    results['mode display is PV'] = node.spin_until(lambda: node.state['mode_display'] == MODE_PV, 5)
    if not results['mode display is PV']:
        return
    results['enabled (Operation enabled)'] = node.enable(timeout=5)
    print(f'{node.motor}: {drive_state(node.status_word())} (status 0x{node.status_word():04X}), '
          f'position {position()} counts')
    if not results['enabled (Operation enabled)']:
        return

    monitor = node.create_timer(0.5, monitor_line)
    try:
        print(f'PV at {speed} counts/s, {duration:g} s forward, {duration:g} s back, {cycles} cycle(s)')
        origin = position()
        for cycle in range(1, cycles + 1):
            for name, target in (('forward', speed), ('back', -speed)):
                label = f'cycle {cycle} {name}'
                leg_start = position()
                cmd['target_velocity'] = target
                aborted = node.spin_until(lambda: abort_reason(target) is not None, duration)
                results[f'{label}: no abort'] = not aborted
                if aborted:
                    print(f'ABORT: {abort_reason(target)}')
                    return
                measured = rate()
                travel = (position() - leg_start) * (1 if target > 0 else -1)
                print(f'{label}: speed {"-" if measured is None else f"{measured:.0f}"} counts/s, '
                      f'travel {travel} counts in {duration:g} s')
                results[f'{label}: speed within {SPEED_TOLERANCE:.0%} of {target}'] = speed_ok(measured, target)
                results[f'{label}: travel >= {TRAVEL_MIN:.0%} of {speed * duration:.0f} counts'] = (
                    travel >= TRAVEL_MIN * speed * duration)

        cmd['target_velocity'] = 0
        results['stopped at zero target velocity'] = node.spin_until(
            lambda: (r := rate()) is not None and abs(r) <= stop_threshold(speed), STOP_TIMEOUT_S)
        net = position() - origin
        print(f'back at the start within {net} counts')
        results[f'net displacement within {NET_MAX:.0%} of one leg'] = abs(net) <= NET_MAX * speed * duration
    finally:
        node.destroy_timer(monitor)


def main():
    rclpy.init()
    node = ElmoClient('elmo_pv_test')
    test = node.profile['tests']['pv_test']
    percent = float(node.declare_parameter('speed_percent', float(test['speed_percent'])).value)
    duration = float(node.declare_parameter('duration_s', float(test['duration_s'])).value)
    cycles = int(node.declare_parameter('cycles', int(test['cycles'])).value)
    motion = node.profile['motion']
    speed = int(motion['max_speed'] * percent / 100)
    if not 0 < percent <= 100 or speed == 0:
        raise SystemExit('speed_percent must be in (0, 100]')
    if not 0 < duration <= MAX_DURATION_S or not 1 <= cycles <= MAX_CYCLES:
        raise SystemExit(f'duration_s must be in (0, {MAX_DURATION_S}], cycles in [1, {MAX_CYCLES}]')
    if motion['accel'] <= 0 or motion['decel'] <= 0:
        raise SystemExit('motion.accel and motion.decel must be greater than zero')

    results = {}
    samples = deque()
    # position samples feed the speed measurement for the whole run, including the final ramp down
    sampler = node.create_timer(
        0.02, lambda: node.has_state() and samples.append((time.monotonic(), int(node.state['position']))))
    try:
        if not node.spin_until(node.has_state, 10):
            print('[FAIL] no valid state received for this motor on /elmo_gpio/gpio_states')
            return 1
        print(f'{percent:g} % of motion.max_speed {motion["max_speed"]} = {speed} counts/s')
        run(node, results, samples, speed, duration, cycles)
    finally:
        # ramp down, then always leave the drive switched off
        node.command['target_velocity'] = 0
        if node.has_state():
            if node.status_word() & STATE_MASK == OP_ENABLED:
                node.spin_until(
                    lambda: (r := window_rate(samples, RATE_WINDOW_S)) is not None and abs(r) <= stop_threshold(speed),
                    STOP_TIMEOUT_S)
            results['disabled (Switch on disabled)'] = node.disable(timeout=5)
            print(f'after disable: {drive_state(node.status_word())} (status 0x{node.status_word():04X})')

    print('--- checks ---')
    for name, ok in results.items():
        print(f'[{"PASS" if ok else "FAIL"}] {name}')
    node.destroy_timer(sampler)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
