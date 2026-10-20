"""Read test: print every state interface, decode the CiA402 status, run basic checks."""
import rclpy

from eyou_ethercat.eyou_client import EyouClient, SW_FAULT, drive_state


def main():
    rclpy.init()
    node = EyouClient('eyou_read_test')
    if not node.spin_until(node.has_state, 10):
        print('[FAIL] no valid state received for this motor on /eyou_gpio/gpio_states')
        return

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
