"""Probe tool: print the raw state interfaces of the gateway (never writes).

    ros2 run gcan_ethercat can_tool watch
"""
import sys
import time

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues
from rclpy.node import Node

from gcan_ethercat import system


class Watch(Node):
    def __init__(self):
        super().__init__('gcan_can_tool')
        self.gateway = system.load_params()['motors'][0]
        self.last = None
        self.create_subscription(DynamicInterfaceGroupValues, f'/{system.CONTROLLER}/gpio_states', self.on_state, 10)

    def on_state(self, msg):
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name == self.gateway:
                values = dict(zip(group.interface_names, group.values))
                if values != self.last:
                    self.last = values
                    print(' '.join(f'{k}={v:.0f}' for k, v in values.items()), flush=True)


def main():
    rclpy.init()
    node = Watch()
    try:
        end = time.monotonic() + (float(sys.argv[2]) if len(sys.argv) > 2 else 10.0)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
