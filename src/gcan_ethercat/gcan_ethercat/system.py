"""Load the yaml config and generate the ros2_control files from the gateway, drive and node profiles."""
import os
import tempfile

import yaml

PREFIX = 'gcan_'
CONTROLLER = 'gcan_gpio'  # one GPIO controller serves every gateway


def _config_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('gcan_ethercat'), 'config')
    except Exception:  # not installed or no ROS environment: use the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


CONFIG_DIR = _config_dir()


def _load(*path):
    with open(os.path.join(CONFIG_DIR, *path)) as f:
        return yaml.safe_load(f)


def load_params():
    return _load('gcan_params.yaml')


def load_gateway(gateway):
    return _load('gateways', gateway + '.yaml')


def load_profile(node):
    return _load('profiles', node + '.yaml')


def load_drive(drive):
    return _load('drives', drive + '.yaml')


def devices(gateway):
    """{node name: profile} of the CANopen nodes listed by a gateway, in listing order."""
    result = {}
    ids = {}
    for name in load_gateway(gateway)['devices']:
        profile = load_profile(name)
        if profile['gateway'] != gateway:
            raise ValueError(f'{name}: profile says gateway {profile["gateway"]}, listed under {gateway}')
        node_id = int(profile['node_id'])
        if node_id in ids:
            raise ValueError(f'{name} and {ids[node_id]} both use CANopen node id {node_id} on {gateway}')
        ids[node_id] = name
        result[name] = profile
    return result


def command_defaults(drive):
    return {c['command_interface']: c['default'] for p in drive['rpdo'] for c in p['channels']}


def state_names(drive):
    return [c['state_interface'] for p in drive['tpdo'] for c in p['channels']]


def slave_config(drive):
    """PDO map in the format the generic slave plugin reads."""
    return {
        'vendor_id': drive['vendor_id'],
        'product_id': drive['product_id'],
        'assign_activate': drive['assign_activate'],
        'sm': drive['sm'],
        'rpdo': drive['rpdo'],
        'tpdo': drive['tpdo'],
    }


def _write_slave_config(out_dir, gateway, drive):
    path = os.path.join(out_dir, gateway + '_slave.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(slave_config(drive), f)
    return path


def slave_address(gateway_cfg, gateway, positions=None):
    """(alias, position) for the ros2_control module.

    A nonzero alias is the address: position is the offset from it and the ring position is ignored. Only with
    alias 0 does `positions` (the ring position from glove_box_bringup) override bus.position.
    """
    bus = gateway_cfg['bus']
    if bus['alias']:
        return bus['alias'], bus['position']
    return 0, (positions or {}).get(gateway, bus['position'])


def build(params, out_dir, positions=None):
    """Return (gpio_xml, controller_parameters, controller_manager_entry) for this package's gateways.

    Same contract as dh_ethercat.system.build so glove_box_bringup can merge packages into one
    ros2_control_node. `params['motors']` lists gateway names.
    """
    gpios = ''
    commands, states = {}, {}
    for gateway in params['motors']:
        gateway_cfg = load_gateway(gateway)
        drive = load_drive(gateway_cfg['drive'])
        alias, position = slave_address(gateway_cfg, gateway, positions)
        commands[gateway] = {'interfaces': list(command_defaults(drive))}
        states[gateway] = {'interfaces': state_names(drive)}
        config_path = _write_slave_config(out_dir, gateway, drive)
        gpios += f'''
    <gpio name="{gateway}">
      {''.join(f'<command_interface name="{n}"/>' for n in commands[gateway]['interfaces'])}
      {''.join(f'<state_interface name="{n}"/>' for n in states[gateway]['interfaces'])}
      <ec_module name="{gateway}_gateway">
        <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
        <param name="alias">{alias}</param>
        <param name="position">{position}</param>
        <param name="slave_config">{config_path}</param>
      </ec_module>
    </gpio>'''
    controller_params = {'ros__parameters': {
        'gpios': list(params['motors']),
        'command_interfaces': commands,
        'state_interfaces': states}}
    manager_entry = {'type': 'gpio_controllers/GpioCommandController'}
    return gpios, controller_params, manager_entry


def generate(params):
    """Return (robot_description, controllers_yaml_path) for all gateways in the params yaml."""
    out_dir = tempfile.mkdtemp(prefix=PREFIX)
    gpios, controller_params, manager_entry = build(params, out_dir)

    # robot_state_publisher needs at least one link
    robot_description = f'''<?xml version="1.0"?>
<robot name="{PREFIX.rstrip('_')}">
  <link name="world"/>
  <ros2_control name="{PREFIX}system" type="system">
    <hardware>
      <plugin>ethercat_driver/EthercatDriver</plugin>
      <param name="master_id">{params['master_id']}</param>
      <param name="control_frequency">{params['cycle_frequency']}</param>
    </hardware>{gpios}
  </ros2_control>
</robot>'''

    controllers = {
        'controller_manager': {'ros__parameters': {
            'update_rate': params['cycle_frequency'],
            CONTROLLER: manager_entry}},
        CONTROLLER: controller_params,
    }
    controllers_path = os.path.join(out_dir, 'controllers.yaml')
    with open(controllers_path, 'w') as f:
        yaml.safe_dump(controllers, f)
    return robot_description, controllers_path
