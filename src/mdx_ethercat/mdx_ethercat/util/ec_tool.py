"""EtherCAT bus helper: scan the bus, write slave alias addresses, store the calibration zero.

  ros2 run mdx_ethercat ec_tool scan
  ros2 run mdx_ethercat ec_tool set-alias <ring_position> <alias>   # 0 removes the alias
  ros2 run mdx_ethercat ec_tool set-zero [motor]                    # launch must be running
  ros2 run mdx_ethercat ec_tool read-params [motor]                 # current drive values of `parameters:`
"""
import argparse
import glob
import os
import re
import subprocess

import rclpy

from mdx_ethercat import system
from mdx_ethercat.mdx_client import MdxClient


def ethercat(*args):
    return subprocess.run(['ethercat', *args], capture_output=True, text=True, check=True).stdout


def read_slaves():
    """One dict per slave on the bus, from `ethercat slaves` (alias, state) and `-v` (identity)."""
    brief = ethercat('slaves').splitlines()
    blocks = re.split(r'=== Master \d+, Slave \d+ ===', ethercat('slaves', '-v'))[1:]
    slaves = []
    for line, block in zip(brief, blocks):
        ring, alias, rel, state, flag, name = re.match(
            r'\s*(\d+)\s+(\d+):(\d+)\s+(\S+)\s+(\S)\s+(.*)', line).groups()
        ident = {k: int(re.search(rf'{k}:\s+(0x[0-9a-fA-F]+)', block).group(1), 16)
                 for k in ('Vendor Id', 'Product code', 'Serial number')}
        slaves.append({'ring': int(ring), 'alias': int(alias), 'rel': int(rel), 'state': state,
                       'flag': flag, 'name': name, **ident})
    return slaves


def scan():
    """List the slaves and match them to known drive types and motor profiles."""
    drive_types = {}
    for path in glob.glob(os.path.join(system.CONFIG_DIR, 'drives', '*.yaml')):
        drive = system.load_drive(os.path.basename(path)[:-5])
        drive_types[(drive['vendor_id'], drive['product_id'])] = os.path.basename(path)[:-5]
    profiles = {}
    for motor in system.load_params()['motors']:
        bus = system.load_profile(motor)['bus']
        profiles[(bus['alias'], bus['position'])] = motor

    print(f'{"ring":>4} {"alias:rel":>9} {"state":6} {"vendor":>10} {"product":>10} '
          f'{"serial":>10}  {"name":14} {"drive type":14} profile')
    for s in read_slaves():
        drive = drive_types.get((s['Vendor Id'], s['Product code']), '(unknown)')
        motor = profiles.get((s['alias'], s['rel']), '(none)')
        print(f'{s["ring"]:>4} {s["alias"]:>6}:{s["rel"]:<2} {s["state"]:6} '
              f'{s["Vendor Id"]:#010x} {s["Product code"]:#010x} {s["Serial number"]:#010x}  '
              f'{s["name"]:14} {drive:14} {motor}')


def set_alias(position, alias):
    """Write the alias into the slave's SII EEPROM so it is found by alias, not by wiring order."""
    ethercat('alias', '-p', str(position), str(alias))
    ethercat('rescan')
    print(f'Wrote alias {alias} to slave {position}. If scan still shows the old alias, '
          f'power-cycle the drive.')


def set_zero(motor):
    """Store the current position as the output zero (calibration.zero_offset) in the profile."""
    rclpy.init(args=['--ros-args', '-p', f'motor:={motor}'])
    node = MdxClient('mdx_ec_tool')
    if not node.spin_until(node.has_state, 10):
        raise SystemExit('no valid state received, is the launch running?')
    position = int(node.state['position'])
    node.destroy_node()
    rclpy.shutdown()

    # realpath leads back to src/ when built with --symlink-install
    path = os.path.realpath(os.path.join(system.CONFIG_DIR, 'profiles', motor + '.yaml'))
    with open(path) as f:
        text = f.read()
    text = re.sub(r'(zero_offset:\s*)-?\d+', lambda m: f'{m.group(1)}{position}', text)
    with open(path, 'w') as f:
        f.write(text)
    print(f'{motor}: zero_offset = {position} written to {path}')


def read_params(motor):
    """Read every `parameters:` object of the motor's drive type over SDO and compare with defaults."""
    profile = system.load_profile(motor)
    drive = system.load_drive(profile['drive'])
    bus = profile['bus']
    ring = next((s['ring'] for s in read_slaves()
                 if (s['alias'], s['rel']) == (bus['alias'], bus['position'])), None)
    if ring is None:
        raise SystemExit(f'{motor}: no slave at alias {bus["alias"]} position {bus["position"]}')
    configured = profile['parameters'] or {}
    print(f'{motor} (ring position {ring})')
    print(f'{"parameter":24} {"index":>8} {"current":>12} {"default":>9}  profile')
    for name, p in drive['parameters'].items():
        try:
            out = ethercat('upload', '-p', str(ring), hex(p['index']), str(p['sub_index']),
                           '--type', p['type'])
            current = out.split()[-1]
        except subprocess.CalledProcessError:
            current = 'n/a'
        default = p.get('default', '')
        mark = '' if default == '' or str(default) == current else '  <- differs'
        print(f'{name:24} {p["index"]:#8x} {current:>12} {default!s:>9}  '
              f'{configured.get(name, "")!s}{mark}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('scan')
    alias = sub.add_parser('set-alias')
    alias.add_argument('position', type=int, help='ring position shown by scan')
    alias.add_argument('alias', type=int, help='16 bit alias; 0 removes it')
    zero = sub.add_parser('set-zero')
    zero.add_argument('motor', nargs='?', default=system.load_params()['motors'][0])
    read = sub.add_parser('read-params')
    read.add_argument('motor', nargs='?', default=system.load_params()['motors'][0])
    args = parser.parse_args()

    if args.command == 'scan':
        scan()
    elif args.command == 'set-alias':
        set_alias(args.position, args.alias)
    elif args.command == 'read-params':
        read_params(args.motor)
    else:
        set_zero(args.motor)


if __name__ == '__main__':
    main()
