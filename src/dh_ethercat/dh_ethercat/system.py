"""Load the yaml config and generate the ros2_control files from the gateway and device profiles."""
import os
import tempfile

import yaml

PREFIX = 'dh_'
CONTROLLER = 'dh_gpio'  # one GPIO controller serves every gateway


def _config_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('dh_ethercat'), 'config')
    except Exception:  # not installed or no ROS environment: use the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


CONFIG_DIR = _config_dir()


def _load(*path):
    with open(os.path.join(CONFIG_DIR, *path)) as f:
        return yaml.safe_load(f)


def load_params():
    return _load('dh_params.yaml')


def load_gateway(gateway):
    return _load('gateways', gateway + '.yaml')


def load_profile(device):
    return _load('profiles', device + '.yaml')


def load_drive(drive):
    return _load('drives', drive + '.yaml')


def object_index(drive, port, obj):
    """CoE index of field `obj` of RS485 port `port` (1..4)."""
    return drive['port_object_base'] + drive['port_object_stride'] * (port - 1) + obj


def devices(gateway):
    """{device name: profile} of the RS485 devices listed by a gateway, in listing order."""
    result = {}
    ports = {}
    for name in load_gateway(gateway)['devices']:
        profile = load_profile(name)
        if profile['gateway'] != gateway:
            raise ValueError(f'{name}: profile says gateway {profile["gateway"]}, listed under {gateway}')
        port = int(profile['port'])
        if port in ports:
            raise ValueError(f'{name} and {ports[port]} are both on port {port} of {gateway}')
        ports[port] = name
        result[name] = profile
    return result


def command_defaults(device, profile, drive):
    """Command interface names of a device with their PDO default values (profile `startup` wins)."""
    startup = profile.get('startup', {})
    return {f'{device}_{c["name"]}': startup.get(c['name'], c['default'])
            for c in drive['actuators'][profile['type']]['commands']}


def state_names(device, profile, drive):
    return [f'{device}_{s["name"]}' for s in drive['actuators'][profile['type']]['states']]


def _channels(drive, profile, device, entries, kind):
    port = int(profile['port'])
    startup = profile.get('startup', {})
    channels = []
    for e in entries:
        channel = {'index': object_index(drive, port, e['obj']), 'sub_index': e['sub'], 'type': 'int16',
                   f'{kind}_interface': f'{device}_{e["name"]}'}
        if kind == 'command':
            channel['default'] = startup.get(e['name'], e['default'])
        channels.append(channel)
    return channels


def slave_config(drive, device_profiles):
    """PDO map for the devices in use, in the format the generic slave plugin reads."""
    rpdo, tpdo = [], []
    for device, profile in device_profiles.items():
        port = int(profile['port'])
        spec = drive['actuators'][profile['type']]
        rpdo.append({'index': spec['rx_pdo'] + port - 1,
                     'channels': _channels(drive, profile, device, spec['commands'], 'command')})
        tpdo.append({'index': spec['tx_pdo'] + port - 1,
                     'channels': _channels(drive, profile, device, spec['states'], 'state')})
    return {
        'vendor_id': drive['vendor_id'],
        'product_id': drive['product_id'],
        'assign_activate': drive['assign_activate'],
        # SM0/SM1 mailbox, SM2 outputs (0x1C12), SM3 inputs (0x1C13). The box refuses SAFEOP with AL 0x001F
        # "invalid watchdog configuration" when the SM2 watchdog is disabled (its SII control byte 0x64 enables
        # it), so SM2 keeps the process-data watchdog: keep the master cycle running. SM3 has none (0x20).
        'sm': [{'index': 0, 'type': 'output'},
               {'index': 1, 'type': 'input'},
               {'index': 2, 'type': 'output', 'pdo': 'rpdo', 'watchdog': 'enable'},
               {'index': 3, 'type': 'input', 'pdo': 'tpdo', 'watchdog': 'disable'}],
        'rpdo': rpdo,
        'tpdo': tpdo,
    }


def _write_slave_config(out_dir, gateway, drive, device_profiles):
    path = os.path.join(out_dir, gateway + '_slave.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(slave_config(drive, device_profiles), f)
    return path


def build(params, out_dir, positions=None):
    """Return (gpio_xml, controller_parameters, controller_manager_entry) for this package's gateways.

    Same contract as mdx_ethercat.system.build so glove_box_bringup can merge packages into one
    ros2_control_node. `params['motors']` lists gateway names; `positions` maps gateway name -> ring
    position, overriding bus.position.
    """
    gpios = ''
    commands, states = {}, {}
    for gateway in params['motors']:
        gateway_cfg = load_gateway(gateway)
        drive = load_drive(gateway_cfg['drive'])
        device_profiles = devices(gateway)
        if not device_profiles:
            raise ValueError(f'{gateway}: no devices listed, nothing to map')
        position = (positions or {}).get(gateway, gateway_cfg['bus']['position'])
        commands[gateway] = {'interfaces': [n for d, p in device_profiles.items()
                                            for n in command_defaults(d, p, drive)]}
        states[gateway] = {'interfaces': [n for d, p in device_profiles.items()
                                          for n in state_names(d, p, drive)]}
        config_path = _write_slave_config(out_dir, gateway, drive, device_profiles)
        gpios += f'''
    <gpio name="{gateway}">
      {''.join(f'<command_interface name="{n}"/>' for n in commands[gateway]['interfaces'])}
      {''.join(f'<state_interface name="{n}"/>' for n in states[gateway]['interfaces'])}
      <ec_module name="{gateway}_gateway">
        <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
        <param name="alias">{gateway_cfg['bus']['alias']}</param>
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
