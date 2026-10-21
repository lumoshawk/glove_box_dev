"""Load the yaml config and generate the ros2_control files from the gateway, drive and device profiles."""
import os
import tempfile

import yaml

PREFIX = 'tuven_'
CONTROLLER = 'tuven_gpio'  # one GPIO controller serves every gateway

# profile type -> (drive file PDO section, channel name prefix, interface suffix)
KINDS = {'sensor': ('tpdo', 'in', 'input'), 'actuator': ('rpdo', 'out', 'output')}


def _config_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('tuven_ethercat'), 'config')
    except Exception:  # not installed or no ROS environment: use the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


CONFIG_DIR = _config_dir()


def _load(*path):
    with open(os.path.join(CONFIG_DIR, *path)) as f:
        return yaml.safe_load(f)


def load_params():
    return _load('tuven_params.yaml')


def load_gateway(gateway):
    return _load('gateways', gateway + '.yaml')


def load_profile(device):
    return _load('profiles', device + '.yaml')


def load_drive(drive):
    return _load('drives', drive + '.yaml')


def channel_name(profile):
    """Name of the drive-file channel a profile sits on: in<n> for a sensor, out<n> for an actuator."""
    return f'{KINDS[profile["type"]][1]}{int(profile["channel"])}'


def interface_name(device, profile):
    return f'{device}_{KINDS[profile["type"]][2]}'


def devices(gateway):
    """{device name: profile} of the channels listed by a gateway, in listing order, validated."""
    drive = load_drive(load_gateway(gateway)['drive'])
    available = {c['name'] for section in ('rpdo', 'tpdo') for p in drive[section] for c in p['channels']}
    result, used = {}, {}
    for name in load_gateway(gateway)['devices']:
        profile = load_profile(name)
        if profile['gateway'] != gateway:
            raise ValueError(f'{name}: profile says gateway {profile["gateway"]}, listed under {gateway}')
        if profile['type'] not in KINDS:
            raise ValueError(f'{name}: type must be one of {list(KINDS)}, not {profile["type"]!r}')
        channel = channel_name(profile)
        if channel not in available:
            raise ValueError(f'{name}: the box has no channel {channel}')
        if channel in used:
            raise ValueError(f'{name} and {used[channel]} are both on {channel} of {gateway}')
        used[channel] = name
        result[name] = profile
    for name, profile in result.items():
        if profile['type'] == 'actuator' and profile.get('sensor') not in (None, *result):
            raise ValueError(f'{name}: sensor {profile["sensor"]} is not listed under {gateway}')
        if profile['type'] == 'actuator' and profile.get('sensor') and result[profile['sensor']]['type'] != 'sensor':
            raise ValueError(f'{name}: {profile["sensor"]} is not a sensor')
    return result


def lock_level(profile):
    """Output level that locks an actuator: the opposite of its unlock level."""
    return 1 - int(profile['unlock_level'])


def command_defaults(device, profile):
    """{command interface: startup value} of an actuator (the profile `startup` wins, else locked)."""
    if profile['type'] != 'actuator':
        return {}
    return {interface_name(device, profile): profile.get('startup', {}).get('output', lock_level(profile))}


def state_names(device, profile):
    return [interface_name(device, profile)] if profile['type'] == 'sensor' else []


def slave_config(drive, device_profiles):
    """PDO map in the format the generic slave plugin reads.

    The PDO of the box is fixed (16 DI, 16 DO): every channel stays in it, the ones with a device get an interface.
    """
    by_channel = {channel_name(p): (d, p) for d, p in device_profiles.items()}
    config = {
        'vendor_id': int(drive['vendor_id']),
        'product_id': int(drive['product_id']),
        'assign_activate': int(drive['assign_activate']),
        'sm': [dict(sm) for sm in drive['sm']],
        'rpdo': [],
        'tpdo': [],
    }
    for section, kind in (('rpdo', 'command'), ('tpdo', 'state')):
        for pdo in drive[section]:
            channels = []
            for c in pdo['channels']:
                channel = {k: int(c[k]) if k != 'type' else c[k] for k in ('index', 'sub_index', 'type', 'mask')}
                if c['name'] in by_channel:
                    device, profile = by_channel[c['name']]
                    channel[f'{kind}_interface'] = interface_name(device, profile)
                    if kind == 'command':
                        channel['default'] = command_defaults(device, profile)[interface_name(device, profile)]
                channels.append(channel)
            config[section].append({'index': int(pdo['index']), 'channels': channels})
    return config


def _write_slave_config(out_dir, gateway, drive, device_profiles):
    path = os.path.join(out_dir, gateway + '_slave.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(slave_config(drive, device_profiles), f)
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
        device_profiles = devices(gateway)
        if not device_profiles:
            raise ValueError(f'{gateway}: no devices listed, nothing to map')
        alias, position = slave_address(gateway_cfg, gateway, positions)
        commands[gateway] = {'interfaces': [n for d, p in device_profiles.items() for n in command_defaults(d, p)]}
        states[gateway] = {'interfaces': [n for d, p in device_profiles.items() for n in state_names(d, p)]}
        config_path = _write_slave_config(out_dir, gateway, drive, device_profiles)
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
