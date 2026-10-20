"""Profile Position test: enable, rotate by tests.pp_test.move_degrees of the motor profile.

Usage: ros2 run eyou_ethercat pp_test --ros-args -p motor:=motor1 [-p record:=true]

With record:=true every state sample (published at cycle_frequency) is written to
log/records/pp_test_<motor>_<timestamp>.csv in the current directory.
"""
import rclpy

from eyou_ethercat.eyou_client import (
    CW_ENABLE_OP, CW_NEW_SETPOINT, CW_SHUTDOWN, SW_SETPOINT_ACK, SW_TARGET_REACHED, EyouClient,
    drive_state)


def main():
    rclpy.init()
    node = EyouClient('eyou_pp_test')
    cmd = node.command
    record = node.declare_parameter('record', False).value
    degrees = node.profile['tests']['pp_test']['move_degrees']
    if not node.spin_until(node.has_state, 10):
        print('no valid state received')
        return

    def wait(condition, what, timeout=5):
        if not node.spin_until(condition, timeout):
            raise SystemExit(f'timeout: {what} (status 0x{node.status_word():04X})')

    def print_monitor():
        s = node.state
        print(f'position={int(s["position"])} counts, velocity={int(s["velocity"])} counts/s, '
              f'torque={int(s["torque"])}, following_error={int(s["following_error"])} counts, '
              f'status_word=0x{node.status_word():04X} ({drive_state(node.status_word())})')

    if record:
        node.start_recording('pp_test', ('target_position', 'profile_velocity'))

    cmd['mode'] = 1  # Profile Position
    cmd['target_position'] = int(node.state['position'])  # hold position while enabling

    if not node.enable():
        raise SystemExit(f'timeout: enable (status 0x{node.status_word():04X})')
    print('Operation enabled')

    start = int(node.state['position'])
    cmd['target_position'] = start + node.deg_to_counts(degrees)
    print(f'{node.motor}: moving {start} -> {cmd["target_position"]} counts ({degrees} deg)')

    # New set-point handshake: raise bit 4, wait for ack (bit 12), then drop bit 4
    cmd['control_word'] = CW_NEW_SETPOINT
    wait(lambda: node.status_word() & SW_SETPOINT_ACK, 'set-point acknowledge')
    cmd['control_word'] = CW_ENABLE_OP

    timer = node.create_timer(0.2, print_monitor)
    wait(lambda: node.status_word() & SW_TARGET_REACHED, 'target reached', timeout=15)
    node.destroy_timer(timer)

    moved = int(node.state['position']) - start
    print(f'Done: moved {moved} counts = {node.counts_to_deg(moved):.1f} deg')

    cmd['control_word'] = CW_SHUTDOWN
    node.sleep(0.5)
    node.stop_recording()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
