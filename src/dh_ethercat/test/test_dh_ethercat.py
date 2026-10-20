"""Offline checks: configs load, the generated PDO map is what the slave exposes, the ESI is in sync."""
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from dh_ethercat import esi, system

DRIVE = system.load_drive('esc_4rs485gateway')


def test_every_profile_loads_and_ports_are_unique():
    for gateway in system.load_params()['motors']:
        profiles = system.devices(gateway)
        assert len(profiles) == 4
        assert sorted(int(p['port']) for p in profiles.values()) == [1, 2, 3, 4]
        for profile in profiles.values():
            assert profile['type'] in DRIVE['actuators']


def test_slave_config_matches_the_slave_pdo_map():
    config = system.slave_config(DRIVE, system.devices('dh_box1'))
    assert (config['vendor_id'], config['product_id']) == (0xDF0, 0x14)
    assert [p['index'] for p in config['rpdo']] == [0x1600, 0x1601, 0x1602, 0x1603]
    assert [p['index'] for p in config['tpdo']] == [0x1A00, 0x1A01, 0x1A02, 0x1A03]
    rx = [(c['index'], c['sub_index']) for c in config['rpdo'][0]['channels']]
    assert rx == [(0x2000, 0), (0x2001, 0), (0x2003, 0), (0x2004, 0), (0x2005, 1), (0x2005, 2),
                  (0x2006, 0), (0x2007, 0), (0x2008, 0), (0x205D, 0), (0x2061, 0)]
    tx = [(c['index'], c['sub_index']) for c in config['tpdo'][3]['channels']]
    assert tx == [(0x380E, 0), (0x380F, 0), (0x3810, 0), (0x3811, 0), (0x3812, 0), (0x3813, 0),
                  (0x3816, 1), (0x3816, 2), (0x3817, 0), (0x3818, 0), (0x3819, 0), (0x381A, 0)]
    assert all(c['type'] == 'int16' for p in config['rpdo'] + config['tpdo'] for c in p['channels'])


def test_interfaces_are_prefixed_by_device_and_unique(tmp_path):
    gpios, controller, entry = system.build({'motors': ['dh_box1']}, str(tmp_path))
    commands = controller['ros__parameters']['command_interfaces']['dh_box1']['interfaces']
    states = controller['ros__parameters']['state_interfaces']['dh_box1']['interfaces']
    assert len(set(commands)) == len(commands) == 4 * 11
    assert len(set(states)) == len(states) == 4 * 12
    assert 'gripper3_target_position' in commands and 'gripper3_position' in states
    assert '<param name="position">3</param>' in gpios
    assert entry['type'] == 'gpio_controllers/GpioCommandController'
    slave = yaml.safe_load((tmp_path / 'dh_box1_slave.yaml').read_text())
    assert slave['product_id'] == 0x14


def test_ring_position_override(tmp_path):
    gpios, _, _ = system.build({'motors': ['dh_box1']}, str(tmp_path), {'dh_box1': 7})
    assert '<param name="position">7</param>' in gpios


def test_startup_values_come_from_the_profile():
    profile = system.load_profile('gripper1')
    defaults = system.command_defaults('gripper1', profile, DRIVE)
    assert defaults['gripper1_force'] == profile['startup']['force']
    assert defaults['gripper1_initialize'] == 0


def _esi_pdos(text):
    pdos = {}
    for pdo in ET.fromstring(text).iter():
        if pdo.tag in ('RxPdo', 'TxPdo'):
            index = int(pdo.find('Index').text[2:], 16)
            pdos[index] = [(int(e.find('Index').text[2:], 16), int(e.find('SubIndex').text[2:], 16),
                            int(e.find('BitLen').text)) for e in pdo.findall('Entry')]
    return pdos


def test_esi_file_is_up_to_date():
    with open(esi.DEFAULT_ESI, encoding='utf-8') as f:
        assert f.read() == esi.generate(), 'run `ros2 run dh_ethercat gen_esi`'


def test_esi_pdos_match_the_slave_config():
    pdos = _esi_pdos(esi.generate())
    config = system.slave_config(DRIVE, system.devices('dh_box1'))
    for pdo in config['rpdo'] + config['tpdo']:
        assert pdos[pdo['index']] == [(c['index'], c['sub_index'], 16) for c in pdo['channels']]
    assert len(pdos) == 16  # 2 actuator types x (rx, tx) x 4 ports


def test_esi_makes_gripper_and_ecylinder_pdos_of_a_port_exclusive():
    root = ET.fromstring(esi.generate())
    exclude = {int(p.find('Index').text[2:], 16): [int(x.text[2:], 16) for x in p.findall('Exclude')]
               for p in root.iter() if p.tag in ('RxPdo', 'TxPdo')}
    for port in range(4):
        assert exclude[0x1600 + port] == [0x1610 + port] and exclude[0x1610 + port] == [0x1600 + port]
        assert exclude[0x1A00 + port] == [0x1A10 + port] and exclude[0x1A10 + port] == [0x1A00 + port]


def _live_pdos(position):
    import subprocess
    out = subprocess.run(['ethercat', 'xml', '-p', str(position)], capture_output=True, timeout=10, check=True)
    text = out.stdout.decode('latin1')  # the slave's names are not valid text
    pdos = {}
    for m in re.finditer(r'<(?:Rx|Tx)Pdo[^>]*>\s*<Index>#x([0-9A-Fa-f]+)</Index>(.*?)</(?:Rx|Tx)Pdo>', text, re.S):
        pdos[int(m.group(1), 16)] = [
            (int(e.group(1), 16), int(e.group(2)), int(e.group(3)))
            for e in re.finditer(r'<Entry>\s*<Index>#x([0-9A-Fa-f]+)</Index>\s*<SubIndex>(\d+)</SubIndex>\s*'
                                 r'<BitLen>(\d+)</BitLen>', m.group(2))]
    return pdos


def test_esi_matches_the_slave_on_the_bus():
    """Only runs with the IgH master up and the box at the profile's ring position."""
    import subprocess
    position = system.load_gateway('dh_box1')['bus']['position']
    try:
        out = subprocess.run(['ethercat', 'slaves', '-v', '-p', str(position)], capture_output=True,
                             text=True, timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        pytest.skip('no EtherCAT master')
    if 'Product code:    0x%08x' % DRIVE['product_id'] not in out:
        pytest.skip('the DH box is not at ring position %d' % position)
    # the slave only reports the PDOs currently assigned (all 16 on a fresh power-up, the gripper ones after a
    # run), so every PDO it reports must equal the ESI's, not the other way round
    live, described = _live_pdos(position), _esi_pdos(esi.generate())
    assert live
    assert all(described[index] == entries for index, entries in live.items())
