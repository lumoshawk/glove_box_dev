"""Load the yaml config and generate the ros2_control files from the motor profiles."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

CONFIG_DIR = os.path.join(get_package_share_directory('mdx_ethercat'), 'config')
CONTROLLER = 'mdx_gpio'  # one GPIO controller serves every motor


def _load(*path):
    with open(os.path.join(CONFIG_DIR, *path)) as f:
        return yaml.safe_load(f)


def load_params():
    return _load('mdx_params.yaml')


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


def generate(params):
    """Return (robot_description, controllers_yaml_path) for all motors in mdx_params.yaml."""
    out_dir = tempfile.mkdtemp(prefix='mdx_')
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
        <param name="position">{profile['bus']['position']}</param>
        <param name="slave_config">{slave_config}</param>
      </ec_module>
    </gpio>'''

    # robot_state_publisher needs at least one link
    robot_description = f'''<?xml version="1.0"?>
<robot name="mdx">
  <link name="world"/>
  <ros2_control name="mdx_system" type="system">
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
            CONTROLLER: {'type': 'gpio_controllers/GpioCommandController'}}},
        CONTROLLER: {'ros__parameters': {
            'gpios': list(params['motors']),
            'command_interfaces': commands,
            'state_interfaces': states}},
    }
    controllers_path = os.path.join(out_dir, 'controllers.yaml')
    with open(controllers_path, 'w') as f:
        yaml.safe_dump(controllers, f)
    return robot_description, controllers_path
