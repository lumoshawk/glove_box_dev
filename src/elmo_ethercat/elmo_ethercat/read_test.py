"""Read test: print every state interface, decode the CiA402 status, run basic checks. Never writes.

Usage: ros2 run elmo_ethercat read_test [--ros-args -p motor:=elmo_m1]
Needs `ros2 launch elmo_ethercat elmo.launch.py` running. Exit code 0 = all checks passed.
"""
import sys

import rclpy

from elmo_ethercat.elmo_client import SW_FAULT, ElmoClient, drive_state


def main():
    rclpy.init()
    node = ElmoClient('elmo_read_test')
    try:
        if not node.spin_until(node.has_state, 10):
            print('[FAIL] no valid state received for this motor on /elmo_gpio/gpio_states')
            return 1

        # The client also publishes its default command, but the control word 0 keeps the drive disabled.
        s = node.state
        print(f'motor            {node.motor}')
        print('--- raw state interfaces ---')
        for name, value in s.items():
            print(f'{name:16s} {int(value)}')

        print('--- decoded ---')
        print(f'drive state      {drive_state(node.status_word())}')
        print(f'status word      0x{node.status_word():04X}')
        print(f'error code       0x{int(s["error_code"]):04X}')
        print(f'mode display     {int(s["mode_display"])}')
        print(f'position         {int(s["position"])} counts')
        print(f'position         {node.position_deg():.2f} deg (calibrated)')

        print('--- checks ---')
        checks = {
            'status word readable': node.status_word() != 0,
            'drive not in fault': not node.status_word() & SW_FAULT,
            'error code is 0': int(s['error_code']) == 0,
        }
        for name, ok in checks.items():
            print(f'[{"PASS" if ok else "FAIL"}] {name}')
        return 0 if all(checks.values()) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
