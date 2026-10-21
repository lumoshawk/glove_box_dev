"""Read every sensor of the IO box and print its level. Never writes anything to the box.

  ros2 run tuven_ethercat read_test
  ros2 run tuven_ethercat read_test --ros-args -p device:=sensor1

PASS: the box delivers a level for the sensor and, when the profile sets tests.read_test.expect_stable, that
level is the stable one. Exit code 0 = all passed.
"""
import sys

import rclpy

from tuven_ethercat.io_client import IoClient


def main():
    rclpy.init()
    client = IoClient('tuven_read_test')
    ok = False
    try:
        if not client.wait_for_states():
            return 1
        ok = True
        for device, state in client.state.items():
            profile = client.profiles[device]
            level = int(state['input'])
            stable = client.sensor_stable(device)
            expect = profile.get('tests', {}).get('read_test', {}).get('expect_stable', False)
            passed = stable or not expect
            ok = ok and passed
            print(f'{device:10} DI{profile["channel"]:<2} level {level}  {"stable" if stable else "not stable":10} '
                  f'{"PASS" if passed else "FAIL (expected stable)"}')
        if not client.state:
            print('no sensor selected')
            ok = False
    finally:
        client.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
