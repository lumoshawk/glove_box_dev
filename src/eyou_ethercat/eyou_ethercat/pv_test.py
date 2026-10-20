"""Profile Velocity test: hold a bounded velocity command, then decelerate to zero.

Usage: ros2 run eyou_ethercat pv_test --ros-args -p motor:=motor1 [-p record:=true]
"""
from collections import deque
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareInterfaces

from eyou_ethercat.eyou_client import CW_ENABLE_OP, CW_HALT, CW_SHUTDOWN, EyouClient, drive_state

MAX_DURATION_S = 5.0


def require_target_velocity_interface(node):
    client = node.create_client(
        ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
    try:
        if not client.wait_for_service(timeout_sec=2):
            raise SystemExit('controller_manager/list_hardware_interfaces service unavailable')
        future = client.call_async(ListHardwareInterfaces.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if not future.done():
            raise SystemExit('timed out checking hardware command interfaces')
        response = future.result()
        if response is None:
            raise SystemExit('controller manager returned no hardware interface list')
        expected = f'{node.motor}/target_velocity'
        if expected not in {interface.name for interface in response.command_interfaces}:
            raise SystemExit(
                f'{expected} is not active; rebuild and relaunch eyou.launch.py before pv_test')
    finally:
        node.destroy_client(client)


def main():
    rclpy.init()
    node = EyouClient('eyou_pv_test')
    cmd = node.command
    record = node.declare_parameter('record', False).value
    test = node.profile['tests']['pv_test']
    velocity = int(test['velocity'])
    duration = float(test['duration_s'])
    motion = node.profile['motion']
    max_speed = motion['max_speed']
    decel = motion['decel']
    monitor = None
    position_samples = deque()
    overspeed = None
    speed_window_s = 0.25
    steady_window_s = 1.0

    def position_velocity():
        now = time.monotonic()
        position_samples.append((now, float(node.state['position'])))
        cutoff = now - speed_window_s
        while len(position_samples) > 2 and position_samples[1][0] <= cutoff:
            position_samples.popleft()
        first_time, first_position = position_samples[0]
        elapsed = now - first_time
        if elapsed < speed_window_s * 0.8:
            return None
        return (position_samples[-1][1] - first_position) / elapsed

    def below_stop_threshold(threshold):
        measured = position_velocity()
        return measured is not None and abs(measured) <= threshold

    def print_monitor():
        state = node.state
        window_velocity = position_velocity()
        window_text = ('warming up' if window_velocity is None
                       else f'{window_velocity:.0f} pulses/s')
        print(f'position={int(state["position"])} counts, '
              f'velocity={int(state["velocity"])} pulses/s, '
              f'position_rate={window_text}, '
              f'target_velocity={int(cmd["target_velocity"])} pulses/s, '
              f'torque={int(state["torque"])}, '
              f'following_error={int(state["following_error"])} counts, '
              f'status_word=0x{node.status_word():04X} ({drive_state(node.status_word())})')

    try:
        if not node.spin_until(node.has_state, 10):
            raise SystemExit('no valid state received')
        if velocity == 0 or abs(velocity) > max_speed:
            raise SystemExit(
                f'pv_test velocity must be nonzero and within +/-motion.max_speed ({max_speed})')
        if not 0 < duration <= MAX_DURATION_S:
            raise SystemExit(f'pv_test duration_s must be in (0, {MAX_DURATION_S}]')
        if motion['accel'] <= 0 or decel <= 0:
            raise SystemExit('motion.accel and motion.decel must be greater than zero')

        require_target_velocity_interface(node)
        cmd['mode'] = 3  # Profile Velocity
        cmd['target_position'] = int(node.state['position'])
        cmd['target_velocity'] = 0
        if not node.spin_until(lambda: node.state.get('mode_display') == 3, 5):
            raise SystemExit(f'timeout: Profile Velocity mode (display={node.state.get("mode_display")})')
        if not node.enable(halt=True):
            raise SystemExit(f'timeout: enable (status 0x{node.status_word():04X})')

        if record:
            node.start_recording('pv_test', ('control_word', 'target_velocity'))
        monitor = node.create_timer(0.2, print_monitor)
        print(f'{node.motor}: commanding {velocity} pulses/s for {duration:.2f} s')
        cmd['target_velocity'] = velocity
        node.sleep(0.1)
        cmd['control_word'] = CW_ENABLE_OP
        overspeed_limit = max(abs(velocity) * 1.5, abs(velocity) + 200)
        tolerance = max(abs(velocity) * 0.2, 50)
        steady_since = None

        def reached_steady_speed():
            nonlocal steady_since, overspeed
            measured = position_velocity()
            if measured is None:
                return False
            if abs(measured) > overspeed_limit:
                overspeed = measured
                return True
            directed_speed = measured if velocity > 0 else -measured
            if abs(directed_speed - abs(velocity)) <= tolerance:
                if steady_since is None:
                    steady_since = time.monotonic()
                return time.monotonic() - steady_since >= steady_window_s
            steady_since = None
            return False

        completed = node.spin_until(reached_steady_speed, duration)
        if overspeed is not None:
            raise SystemExit(
                f'overspeed: position-derived velocity={overspeed:.0f} pulses/s, '
                f'limit={overspeed_limit:.0f}; commanding stop')
        if not completed:
            measured = position_velocity()
            actual = 'unavailable' if measured is None else f'{measured:.0f} pulses/s'
            raise SystemExit(
                f'failed to reach steady speed: position-derived velocity={actual}, '
                f'target={velocity} pulses/s, tolerance=+/-{tolerance:.0f}')
        print(f'Steady speed verified for {steady_window_s:.1f} s')

        cmd['target_velocity'] = 0
        cmd['control_word'] = CW_ENABLE_OP | CW_HALT
        stop_threshold = max(50, abs(velocity) * 0.05)
        stop_timeout = max(5.0, abs(velocity) / decel * 2 + 2)
        if not node.spin_until(
            lambda: below_stop_threshold(stop_threshold), stop_timeout
        ):
            raise SystemExit(
                f'timeout: decelerating to zero '
                f'(position-derived velocity={position_velocity()} pulses/s)')
        print('Stopped at zero target velocity')
    finally:
        cmd['target_velocity'] = 0
        if node.state and node.state.get('mode_display') == 3:
            cmd['control_word'] = CW_ENABLE_OP | CW_HALT
            stop_threshold = max(50, abs(velocity) * 0.05)
            stop_timeout = max(5.0, abs(velocity) / decel * 2 + 2) if decel > 0 else 5.0
            if not node.spin_until(
                lambda: below_stop_threshold(stop_threshold), stop_timeout
            ):
                print(f'WARNING: velocity did not settle before shutdown '
                      f'(position-derived velocity={position_velocity()} pulses/s)')
        cmd['control_word'] = CW_SHUTDOWN
        if monitor is not None:
            node.destroy_timer(monitor)
        node.sleep(0.2)
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
