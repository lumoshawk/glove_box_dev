"""Load the top-level bus layout and reach each motor package's own modules."""
import importlib
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory

CONFIG_PATH = os.path.join(get_package_share_directory('glove_box_bringup'), 'config', 'bringup.yaml')


def present_entries(entries, master_id=0):
    """Drop entries whose ring position is not on the physical bus (`ethercat slaves`), with their names.

    A configured slave that is missing keeps the shared domain INCOMPLETE, and then the driver publishes no
    feedback for ANY slave (all states NaN). Returns (kept, dropped); everything is kept if the count cannot be read.
    """
    try:
        out = subprocess.run(['ethercat', '-m', str(master_id), 'slaves'], capture_output=True, text=True,
                             timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return list(entries), []
    count = len([line for line in out.splitlines() if line.strip()])
    kept = [e for i, e in enumerate(entries) if e.get('ring', i) < count]
    return kept, [e for e in entries if e not in kept]


def load_config():
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def motors_by_package(config):
    """{package: [(ring_position, motor), ...]} in bus order.

    The ring position is the entry's `ring` key if given (needed when non-motor slaves such as a
    junction sit in front of the drives), else the list index.
    """
    result = {}
    for index, entry in enumerate(config['bus']):
        result.setdefault(entry['package'], []).append((entry.get('ring', index), entry['motor']))
    return result


def package_system(package):
    return importlib.import_module(package + '.system')


def package_client(package):
    """The package's client module (mdx_client / eyou_client), for CiA402 constants and drive_state."""
    return importlib.import_module(package + '.' + package.replace('_ethercat', '') + '_client')
