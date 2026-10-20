"""Return-zero test: home a gripper (initialize = 1) and wait until it reports initialised.

    ros2 run dh_ethercat return_zero_test                                  # first device (gripper1)
    ros2 run dh_ethercat return_zero_test --ros-args -p device:=gripper3
    ros2 run dh_ethercat return_zero_test --ros-args -p device:=all        # one after the other

The gripper moves to its zero end stop (0.5-3 s) and keeps that position. Exit code 0 = all passed.
"""
import rclpy

from dh_ethercat.dh_client import DhClient, home_gripper, require_gripper


def run(client, device):
    require_gripper(client, device)
    cfg = client.profiles[device].get('tests', {}).get('return_zero_test', {})
    state, command = client.state[device], client.command[device]
    print(f'{device}: init_state={state["init_state"]:.0f} position={state["position"]:.0f}')
    command['target_position'] = state['position']  # hold where it is until the homing moves it
    print('  homing (the gripper moves to its zero end stop)...')
    ok = home_gripper(client, device, cfg.get('timeout_s', 15))
    state = client.state[device]
    print(f'  init_state={state["init_state"]:.0f} position={state["position"]:.0f} '
          f'grip_state={state["grip_state"]:.0f} error_code={state["error_code"]:.0f}')
    if not ok or state['error_code'] != 0:
        print('  FAIL')
        return False
    print('  PASS')
    return True


def main():
    rclpy.init()
    client = DhClient('dh_return_zero_test', all_by_default=False)
    try:
        if not client.wait_for_states():
            raise SystemExit(1)
        client.start_commands()
        results = [run(client, device) for device in client.profiles]
    finally:
        client.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
