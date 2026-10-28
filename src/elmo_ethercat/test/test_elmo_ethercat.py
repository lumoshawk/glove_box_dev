"""Offline checks: the drive file matches the ESI, the PDO map fits the drive, config generation works,
and the CiA402 helpers behave. No ROS node or hardware is needed."""
import os
import tempfile
from collections import deque
import xml.etree.ElementTree as ET

import pytest
import yaml

from elmo_ethercat import elmo_client, pp_test, pv_test, system

ESI_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'ref', 'Elmo ECAT 00010420 V11.xml'))
DRIVE = system.load_drive('elmo_gold')
PROFILE = system.load_profile('elmo_m1')
SIZES = {'int8': 8, 'uint8': 8, 'int16': 16, 'uint16': 16, 'int32': 32, 'uint32': 32}
ESI_TYPES = {'SINT': 'int8', 'USINT': 'uint8', 'INT': 'int16', 'UINT': 'uint16', 'DINT': 'int32', 'UDINT': 'uint32'}


@pytest.fixture(scope='module')
def esi_device():
    if not os.path.exists(ESI_PATH):
        pytest.skip('ref/Elmo ECAT 00010420 V11.xml not available')
    for device in ET.parse(ESI_PATH).iter('Device'):
        type_ = device.find('Type')
        if type_.get('ProductCode') == '#x00030924' and type_.get('RevisionNo') == '#x00010420':
            return device
    pytest.fail('drive revision 0x00010420 not in the ESI')


def _objects(device):
    return {o.find('Index').text.lower(): o for o in device.find('Profile/Dictionary/Objects')}


def _hex(text):
    return int(text.replace('#x', '0x'), 16)


def test_addressed_by_ring_position_because_the_drive_cannot_store_an_alias():
    assert PROFILE['bus'] == {'alias': 0, 'position': 5}
    assert system.load_params()['motors'] == ['elmo_m1']


def test_identity_matches_esi(esi_device):
    type_ = esi_device.find('Type')
    assert DRIVE['product_id'] == _hex(type_.get('ProductCode'))
    assert DRIVE['vendor_id'] == 0x9A  # reported by the slave (SII and CoE 0x1018)
    assert 'GCON Drive' in esi_device.find('Name').text


def test_dc_sync_matches_esi(esi_device):
    modes = {m.find('Name').text: _hex(m.find('AssignActivate').text) for m in esi_device.findall('Dc/OpMode')}
    assert DRIVE['assign_activate'] == modes['DC Sync']


def _esi_type(types, type_name, sub_index):
    """Basic type of one sub-index of a composite object (record entry or array element)."""
    for sub in types[type_name].findall('SubItem'):
        if sub.find('SubIdx') is not None and sub.find('SubIdx').text == str(sub_index):
            return ESI_TYPES[sub.find('Type').text]
        if sub.find('SubIdx') is None:  # array: the element type carries the base type and bounds
            array = types[sub.find('Type').text]
            low = int(array.find('ArrayInfo/LBound').text)
            if low <= sub_index < low + int(array.find('ArrayInfo/Elements').text):
                return ESI_TYPES[array.find('BaseType').text]
    return None


def test_every_mapped_and_parameter_object_exists_with_the_esi_type(esi_device):
    objects = _objects(esi_device)
    types = {t.find('Name').text: t for t in esi_device.findall('Profile/Dictionary/DataTypes/DataType')}
    channels = [c for pdo in DRIVE['rpdo'] + DRIVE['tpdo'] for c in pdo['channels']]
    for entry in channels + list(DRIVE['parameters'].values()):
        type_name = objects[f"#x{entry['index']:04x}".lower()].find('Type').text
        if type_name in ESI_TYPES:
            assert entry['sub_index'] == 0 and ESI_TYPES[type_name] == entry['type'], hex(entry['index'])
        else:
            assert _esi_type(types, type_name, entry['sub_index']) == entry['type'], hex(entry['index'])


def test_pdos_are_mappable_in_their_direction(esi_device):
    objects = _objects(esi_device)
    for pdos, direction in ((DRIVE['rpdo'], 'R'), (DRIVE['tpdo'], 'T')):
        for pdo in pdos:
            for c in pdo['channels']:
                obj = objects[f"#x{c['index']:04x}".lower()]
                mapping = obj.find('Flags/PdoMapping')
                # 0x603F is read-only but flagged R in this ESI; the drive accepts it in the TxPDO
                assert mapping is not None and (mapping.text == direction or c['index'] == 0x603F), hex(c['index'])


def test_pdo_maps_are_the_free_ones_and_fit_the_sync_managers(esi_device):
    free = {_hex(p.find('Index').text) for p in esi_device.findall('RxPdo') + esi_device.findall('TxPdo')
            if p.get('Fixed') == '0'}
    for pdos in (DRIVE['rpdo'], DRIVE['tpdo']):
        assert len(pdos) == 1 and pdos[0]['index'] in free
        channels = pdos[0]['channels']
        assert len(channels) <= 8  # DT1607 / DT1A07: 272 bit = count + 8 x 32 bit entries
        assert sum(SIZES[c['type']] for c in channels) // 8 <= 0x20  # sync manager 2 / 3 size
    sm = {sm.text: _hex(sm.get('MaxSize')) for sm in esi_device.findall('Sm')}
    assert sm['Outputs'] == sm['Inputs'] == 0x20


def test_interfaces_are_unique_and_cover_the_cia402_tests():
    commands = system.command_defaults(DRIVE)
    states = system.state_names(DRIVE)
    assert len(states) == len(set(states))
    assert {'control_word', 'mode', 'target_position'} <= set(commands)
    assert {'status_word', 'error_code', 'mode_display', 'position', 'velocity'} <= set(states)  # read_all needs these


def test_profile_parameters_exist_in_the_drive_file_and_match_the_cycle():
    assert set(PROFILE['parameters']) <= set(DRIVE['parameters'])
    cycle_ms = 1000 / system.load_params()['cycle_frequency']
    assert PROFILE['parameters']['interpolation_time'] == cycle_ms


def test_build_generates_slave_config_for_the_plugin():
    with tempfile.TemporaryDirectory() as out_dir:
        gpios, controller, entry = system.build(system.load_params(), out_dir)
        assert '<param name="alias">0</param>' in gpios and '<param name="position">5</param>' in gpios
        assert '<param name="position">7</param>' in system.build(system.load_params(), out_dir, {'elmo_m1': 7})[0]  # bringup ring
        assert entry == {'type': 'gpio_controllers/GpioCommandController'}
        assert list(controller['ros__parameters']['command_interfaces']['elmo_m1']['interfaces']) == list(
            system.command_defaults(DRIVE))
        config = yaml.safe_load(open(os.path.join(out_dir, 'elmo_m1_slave.yaml')))
    assert config['vendor_id'] == 0x9A and config['product_id'] == 0x30924
    assert config['sdo'] == [{'index': 0x60C2, 'sub_index': 1, 'type': 'uint8', 'value': 2}]
    assert 'parameters' not in config


def test_pp_test_defaults_move_100000_counts_and_back():
    test = PROFILE['tests']['pp_test']
    assert test['move_counts'] == 100000
    assert 0 < test['velocity'] <= PROFILE['motion']['max_speed']


def test_pp_move_timeout_covers_the_move_plus_ramps():
    assert pp_test.move_timeout(100000, 10000, 20000) == pytest.approx(10 + 1 + 5)
    assert pp_test.move_timeout(-100000, 10000, 20000) == pp_test.move_timeout(100000, 10000, 20000)


def test_pv_test_defaults_are_within_the_test_limits():
    test = PROFILE['tests']['pv_test']
    assert 0 < test['speed_percent'] <= 100
    assert 0 < test['duration_s'] <= pv_test.MAX_DURATION_S
    assert 1 <= test['cycles'] <= pv_test.MAX_CYCLES


def test_window_rate_needs_a_full_window_and_measures_counts_per_second():
    samples = deque((t / 50, t * 100) for t in range(10))  # 0.18 s at 5000 counts/s
    assert pv_test.window_rate(samples, 0.5) is None
    samples = deque((t / 50, t * 100) for t in range(60))  # 1.18 s
    assert pv_test.window_rate(samples, 0.5) == pytest.approx(5000)
    assert samples[0][0] <= samples[-1][0] - 0.5 and len(samples) < 60  # old samples dropped
    assert pv_test.window_rate(deque(), 0.5) is None


def test_window_rate_is_signed():
    samples = deque((t / 50, -t * 100) for t in range(40))
    assert pv_test.window_rate(samples, 0.5) == pytest.approx(-5000)


def test_speed_ok_uses_a_relative_tolerance():
    assert pv_test.speed_ok(4500, 5000) and pv_test.speed_ok(5900, 5000)
    assert not pv_test.speed_ok(3900, 5000) and not pv_test.speed_ok(-5000, 5000) and not pv_test.speed_ok(None, 5000)
    assert pv_test.speed_ok(-4800, -5000)


def test_state_machine_walks_to_operation_enabled():
    for status_word, expected in ((0x0250, 0x06), (0x0231, 0x07), (0x0233, 0x0F), (0x0237, 0x0F)):
        assert elmo_client.next_control_word(status_word) == expected
    assert elmo_client.next_control_word(0x0218) == 0x80  # fault: reset first


def test_drive_state_decodes_status_words():
    assert elmo_client.drive_state(0x0250) == 'Switch on disabled'
    assert elmo_client.drive_state(0x0237) == 'Operation enabled'
    assert elmo_client.drive_state(0x0218) == 'Fault'
