"""Drive one lock actuator: lock, unlock for hold_s, lock again, and print its paired sensor on the way.

  ros2 run tuven_ethercat write_test                                  # first actuator (MOVES the lock)
  ros2 run tuven_ethercat write_test --ros-args -p device:=actuator2

The box has no output read-back: PASS means the sequence was commanded with the controller answering the whole
time and the actuator was left locked. Check the output LED / the lock itself for the physical effect.
The sensor levels are only printed, a tool may legitimately move when the lock opens.
Defaults to the FIRST actuator so a bare run never opens every lock. Exit code 0 = passed.
"""
import sys

import rclpy

from tuven_ethercat.io_client import IoClient


def main():
    rclpy.init()
    client = IoClient('tuven_write_test', all_by_default=False)
    ok = False
    actuator = next(d for d, p in client.profiles.items() if p['type'] == 'actuator')
    profile = client.profiles[actuator]
    sensor = profile.get('sensor')
    hold_s = float(profile.get('tests', {}).get('write_test', {}).get('hold_s', 1.0))

    def level():
        return int(client.state[sensor]['input']) if sensor in client.state else '-'

    try:
        if not client.wait_for_states():
            return 1
        steps = [('lock', client.lock), ('unlock', client.unlock), ('lock', client.lock)]
        for name, action in steps:
            action(actuator)
            client.spin_until(lambda: False, hold_s)
            print(f'{actuator} {name:6} -> commanded level {client.command[actuator]["output"]}, '
                  f'{sensor} level {level()}')
        # the controller must still be answering after the sequence
        ok = client.has_state() and client.command[actuator]['output'] == 1 - int(profile['unlock_level'])
        print(f'{actuator}: {"PASS" if ok else "FAIL"}')
    finally:
        client.lock(actuator)
        client.spin_until(lambda: False, 0.3)
        client.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
