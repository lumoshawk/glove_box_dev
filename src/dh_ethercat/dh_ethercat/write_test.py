"""Write test: command a gripper to close and open, and check the position feedback follows.

    ros2 run dh_ethercat write_test                                   # first device (gripper1)
    ros2 run dh_ethercat write_test --ros-args -p device:=gripper2
    ros2 run dh_ethercat write_test --ros-args -p device:=all         # one after the other
    ros2 run dh_ethercat write_test --ros-args -p init:=true          # home first if the gripper is not initialised

The gripper must be initialised (run return_zero_test, or pass init:=true). It writes force, speed and target
position from the profile's tests.write_test, then the open position again at the end. Exit code 0 = all passed.
"""
import rclpy

from dh_ethercat.dh_client import DhClient, home_gripper, require_gripper


def move(client, device, label, target, cfg):
    client.command[device]['target_position'] = target
    print(f'  {label} -> {target}')
    client.spin_until(lambda: False, 0.5)  # the gateway answers in ~30 ms; skip the stale state
    client.spin_until(lambda: client.state[device]['grip_state'] != 0, cfg['timeout_s'])
    state = client.state[device]
    # an object in the way (grip_state 2) legitimately stops the gripper short of the target
    reached = abs(state['position'] - target) <= cfg['tolerance'] or state['grip_state'] == 2
    print(f'    position={state["position"]:.0f} grip_state={state["grip_state"]:.0f} '
          f'{client.describe(device, "grip_state", state["grip_state"])} error_code={state["error_code"]:.0f}')
    return reached and state['error_code'] == 0


def run(client, device, init):
    require_gripper(client, device)
    profile = client.profiles[device]
    cfg = profile['tests']['write_test']
    limits = profile['limits']
    for name, value in (('force', cfg['force']), ('speed', cfg['speed']),
                        ('position', cfg['open_position']), ('position', cfg['close_position'])):
        low, high = limits[name]
        if not low <= value <= high:
            raise SystemExit(f'{device}: tests.write_test {name} {value} is outside limits {limits[name]}')
    state, command = client.state[device], client.command[device]
    print(f'{device}: init_state={state["init_state"]:.0f} position={state["position"]:.0f}')
    command['target_position'] = state['position']  # hold where it is
    command['force'] = cfg['force']
    command['speed'] = cfg['speed']
    if state['init_state'] != 1:
        if not init:
            print('  FAIL: not initialised; run return_zero_test or pass -p init:=true')
            return False
        print('  initialising...')
        if not home_gripper(client, device, profile['tests']['return_zero_test']['timeout_s']):
            print('  FAIL: initialisation did not finish')
            return False
    ok = move(client, device, 'close', cfg['close_position'], cfg)
    ok = move(client, device, 'open', cfg['open_position'], cfg) and ok
    print('  PASS' if ok else '  FAIL')
    return ok


def main():
    rclpy.init()
    client = DhClient('dh_write_test', all_by_default=False)
    init = client.declare_parameter('init', False).value
    try:
        if not client.wait_for_states():
            raise SystemExit(1)
        client.start_commands()
        results = [run(client, device, init) for device in client.profiles]
    finally:
        client.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
