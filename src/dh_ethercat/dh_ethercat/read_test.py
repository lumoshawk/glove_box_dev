"""Read test: print every state of the RS485 devices of a gateway. Never writes a command.

    ros2 run dh_ethercat read_test                                  # all devices of the first gateway
    ros2 run dh_ethercat read_test --ros-args -p device:=gripper2   # one device

A device passes when it answers (an empty or unlisted-but-mapped port reads all zeros, which fails) and its
error/alarm code is within the profile's tests.read_test.max_error_code. Exit code 0 = all passed.
"""
import rclpy

from dh_ethercat.dh_client import DhClient

ERROR_FIELDS = {'gripper': 'error_code', 'ecylinder': 'alarm_code'}


def check(client, device):
    profile = client.profiles[device]
    state = client.state[device]
    kind = profile['type']
    print(f'{device} (port {profile["port"]}, {kind}):')
    for name, value in state.items():
        print(f'  {name:20s} {value:8.0f}  {client.describe(device, name, value)}')
    limit = profile.get('tests', {}).get('read_test', {}).get('max_error_code', 0)
    code = state[ERROR_FIELDS[kind]]
    if not any(state.values()):
        print('  FAIL: all zeros, nothing answers on this port (actuator not connected, not powered, or not detected '
              'by the box; if it was connected or powered after the box, power-cycle the box and relaunch. '
              'Remove it from `devices:` in the gateway file if it is unused)')
        return False
    if code > limit:
        print(f'  FAIL: {ERROR_FIELDS[kind]} = {code:.0f} (limit {limit})')
        return False
    print('  PASS')
    return True


def main():
    rclpy.init()
    client = DhClient('dh_read_test')
    try:
        if not client.wait_for_states():
            raise SystemExit(1)
        results = [check(client, device) for device in client.profiles]
    finally:
        client.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
