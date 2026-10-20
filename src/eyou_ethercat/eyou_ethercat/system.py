"""Load the yaml config and generate the ros2_control files from the motor profiles."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

CONFIG_DIR = os.path.join(get_package_share_directory('eyou_ethercat'), 'config')
PREFIX = 'eyou_'
CONTROLLER = 'eyou_gpio'  # one GPIO controller serves every motor


def _load(*path):
    with open(os.path.join(CONFIG_DIR, *path)) as f:
        return yaml.safe_load(f)


def load_params():
    return _load('eyou_params.yaml')


def load_profile(motor):
    return _load('profiles', motor + '.yaml')


def load_drive(drive):
    return _load('drives', drive + '.yaml')


def _channels(drive):
    return [c for pdo in drive['rpdo'] + drive['tpdo'] for c in pdo['channels']]


def command_defaults(drive):
    """Command interface names of a drive with their PDO default values."""
    return {c['command_interface']: c.get('default', 0)
            for pdo in drive['rpdo'] for c in pdo['channels'] if 'command_interface' in c}


def state_names(drive):
    return [c['state_interface'] for c in _channels(drive) if 'state_interface' in c]


def _write_slave_config(out_dir, motor, profile, drive):
    """Drive-type PDO map plus this motor's startup SDOs, in the format the driver reads."""
    config = {k: v for k, v in drive.items() if k != 'parameters'}
    config['sdo'] = [{k: v for k, v in drive['parameters'][name].items() if k != 'default'} | {'value': value}
                     for name, value in (profile['parameters'] or {}).items()]
    path = os.path.join(out_dir, motor + '_slave.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(config, f)
    return path


def build(params, out_dir, positions=None):
    """Return (gpio_xml, controller_parameters, controller_manager_entry) for this package's motors.

    Split out of generate() so a top-level launch can merge several packages into one ros2_control_node.
    `positions` overrides bus.position only for alias 0 (absolute ring addressing).
    Nonzero aliases retain the profile position as an offset from that alias.
    """
    gpios = ''
    commands, states = {}, {}
    for motor in params['motors']:
        profile = load_profile(motor)
        drive = load_drive(profile['drive'])
        commands[motor] = {'interfaces': list(command_defaults(drive))}
        states[motor] = {'interfaces': state_names(drive)}
        slave_config = _write_slave_config(out_dir, motor, profile, drive)
        gpios += f'''
    <gpio name="{motor}">
      {''.join(f'<command_interface name="{n}"/>' for n in commands[motor]['interfaces'])}
      {''.join(f'<state_interface name="{n}"/>' for n in states[motor]['interfaces'])}
      <ec_module name="{motor}_drive">
        <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
        <param name="alias">{profile['bus']['alias']}</param>
        <param name="position">{profile['bus']['position'] if profile['bus']['alias'] else (positions or {}).get(motor, profile['bus']['position'])}</param>
        <param name="slave_config">{slave_config}</param>
      </ec_module>
    </gpio>'''
    controller_params = {'ros__parameters': {
        'gpios': list(params['motors']),
        'command_interfaces': commands,
        'state_interfaces': states}}
    manager_entry = {'type': 'gpio_controllers/GpioCommandController'}
    return gpios, controller_params, manager_entry


def generate(params):
    """Return (robot_description, controllers_yaml_path) for all motors in the params yaml."""
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
