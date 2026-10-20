import os
import tempfile

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

from glove_box_bringup import bus

# One GPIO controller per package keeps each package's topics (/mdx_gpio/..., /eyou_gpio/...)
# unchanged, so every package's own tests work against this combined launch.


def generate_launch_description():
    config = bus.load_config()
    out_dir = tempfile.mkdtemp(prefix='glove_box_')

    gpios = ''
    manager = {'update_rate': config['cycle_frequency']}
    controller_params = {}
    controllers = []
    # `gateways` are non-motor slaves (DH box): same build() contract, but read_all does not list them
    bus_entries = config['bus']
    if config.get('skip_absent', True):
        bus_entries, absent = bus.present_entries(bus_entries, config['master_id'])
        if absent:
            print('glove_box_bringup: not on the bus, skipped: ' +
                  ', '.join(f"{e['motor']} (ring {e.get('ring')})" for e in absent))
    entries = {'bus': bus_entries + config.get('gateways', [])}
    for package, motors in bus.motors_by_package(entries).items():
        module = bus.package_system(package)
        positions = {motor: ring for ring, motor in motors}
        xml, params, entry = module.build({'motors': [m for _, m in motors]}, out_dir, positions)
        gpios += xml
        manager[module.CONTROLLER] = entry
        controller_params[module.CONTROLLER] = params
        controllers.append(module.CONTROLLER)

    robot_description = f'''<?xml version="1.0"?>
<robot name="glove_box">
  <link name="world"/>
  <ros2_control name="glove_box_system" type="system">
    <hardware>
      <plugin>ethercat_driver/EthercatDriver</plugin>
      <param name="master_id">{config['master_id']}</param>
      <param name="control_frequency">{config['cycle_frequency']}</param>
    </hardware>{gpios}
  </ros2_control>
</robot>'''

    controllers_path = os.path.join(out_dir, 'controllers.yaml')
    with open(controllers_path, 'w') as f:
        yaml.safe_dump({'controller_manager': {'ros__parameters': manager}, **controller_params}, f)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_path],
        output='screen',
    )
    spawners = [Node(package='controller_manager', executable='spawner', arguments=[name])
                for name in controllers]
    return LaunchDescription([robot_state_publisher, control_node, *spawners])
