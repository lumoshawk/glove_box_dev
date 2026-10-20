"""Cyclic Synchronous Position test: run the segments of tests.csp_test in the motor profile.

Usage: ros2 run mdx_ethercat csp_test --ros-args -p motor:=motor1 [-p record:=true]

With record:=true every state sample is written to log/records/csp_test_<motor>_<timestamp>.csv;
plot it with `ros2 run mdx_ethercat plot_csv`.
"""
import rclpy

from mdx_ethercat.mdx_client import CW_SHUTDOWN, MdxClient, drive_state


def target_at(segments, t):
    """(counts travelled, counts/s) after t seconds; segments are (counts, counts/s), constant speed each."""
    position = 0
    for counts, speed in segments:
        sign = 1 if counts > 0 else -1
        if t < abs(counts) / speed:
            return position + speed * t * sign, speed * sign
        position += counts
        t -= abs(counts) / speed
    return position, 0


def main():
    rclpy.init()
    node = MdxClient('mdx_csp_test')
    cmd = node.command
    record = node.declare_parameter('record', False).value
    max_speed = node.profile['motion']['max_speed']
    segments = [(node.deg_to_counts(deg), fraction * max_speed)
                for deg, fraction in node.profile['tests']['csp_test']['segments']]
    total_time = sum(abs(counts) / speed for counts, speed in segments)
    if not node.spin_until(node.has_state, 10):
        print('no valid state received')
        return

    def wait(condition, what, timeout=5):
        if not node.spin_until(condition, timeout):
            raise SystemExit(f'timeout: {what} (status 0x{node.status_word():04X})')

    def print_monitor():
        s = node.state
        print(f'position={int(s["position"])} counts, velocity={int(s["velocity"])} counts/s, '
              f'current={int(s["current"])}, following_error={int(s["following_error"])} counts, '
              f'status_word=0x{node.status_word():04X} ({drive_state(node.status_word())})')

    if record:
        node.start_recording('csp_test', ('target_position', 'velocity_offset'))

    cmd['mode'] = 8  # Cyclic Synchronous Position
    cmd['target_position'] = int(node.state['position'])  # hold position while enabling
    wait(lambda: node.state['mode_display'] == 8, 'CSP mode')

    if not node.enable():
        raise SystemExit(f'timeout: enable (status 0x{node.status_word():04X})')
    print('Operation enabled')

    # CSP: a new target (and velocity feed-forward) must be sent every cycle
    start = int(node.state['position'])
    t0 = node.get_clock().now()

    def update_target():
        t = (node.get_clock().now() - t0).nanoseconds / 1e9
        counts, speed = target_at(segments, t)
        cmd['target_position'] = start + int(counts)
        if 'velocity_offset' in cmd:
            cmd['velocity_offset'] = int(speed)

    node.create_timer(1 / node.rate_hz, update_target)
    node.create_timer(0.2, print_monitor)
    print(f'{node.motor}: moving {sum(abs(c) for c, _ in segments)} counts in {total_time:.2f} s')
    node.sleep(total_time + 0.5)

    moved = int(node.state['position']) - start
    print(f'Done: moved {moved} counts = {node.counts_to_deg(moved):.1f} deg')

    cmd['control_word'] = CW_SHUTDOWN
    node.sleep(0.5)
    node.stop_recording()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
