"""get_tool / switch_tool: run a tool changer sequence on one lock actuator and its paired proximity sensor.

  ros2 run tuven_ethercat get_tool                              # first actuator (actuator1) and its sensor
  ros2 run tuven_ethercat switch_tool --ros-args -p device:=actuator2 -p stable_s:=6.0

Timings come from `scripts:` in tuven_params.yaml. The actuator is locked again whatever happens. Exit code 0 = the
sensor was stable long enough.
"""
import sys

import rclpy

from tuven_ethercat import tool_logic
from tuven_ethercat.io_client import IoClient


def _run(script, name):
    rclpy.init()
    client = IoClient(name, all_by_default=False)
    defaults = client.params['scripts'][name]
    delay_s, stable_s, timeout_s = (client.declare_parameter(k, float(defaults[k])).value
                                    for k in ('delay_s', 'stable_s', 'timeout_s'))
    actuator = next(d for d, p in client.profiles.items() if p['type'] == 'actuator')
    if client.profiles[actuator].get('sensor') is None:
        raise SystemExit(f'{actuator} has no `sensor` in its profile')
    sensor = client.profiles[actuator]['sensor']
    ok = False
    try:
        if not client.wait_for_states():
            return 1
        result = script(client, actuator, sensor, delay_s, stable_s, timeout_s,
                        log=lambda text: print(f'[{client.now() % 1000:8.2f}] {text}'))
        client.spin_until(lambda: False, 0.3)  # let the final lock go out
        ok = result.ok
        print(f'{name} {actuator}: {"PASS" if ok else "FAIL"} - {result.message} ({result.elapsed_s:.1f} s)')
    except KeyboardInterrupt:
        client.spin_until(lambda: False, 0.3)
        print(f'{name} {actuator}: interrupted, locked')
    finally:
        client.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


def get_tool():
    sys.exit(_run(tool_logic.get_tool, 'get_tool'))


def switch_tool():
    sys.exit(_run(tool_logic.switch_tool, 'switch_tool'))
