"""Generate the EtherCAT Slave Information (ESI) file of the DH EtherCAT Box 4 Port from config/drives/esc_4rs485gateway.yaml.

    ros2 run dh_ethercat gen_esi                  # rewrite esi/DH_EtherCAT_Box_4Port.xml in the source tree
    ros2 run dh_ethercat gen_esi --check          # exit 1 if the file is out of date
    ros2 run dh_ethercat gen_esi -o /tmp/dh.xml
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

from dh_ethercat import system

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESI_NAME = 'DH_EtherCAT_Box_4Port.xml'


def _default_esi():
    """The source tree's esi/ file when running from the source tree, else the installed copy."""
    if os.path.exists(os.path.join(PACKAGE_DIR, 'setup.py')):
        return os.path.join(PACKAGE_DIR, 'esi', ESI_NAME)
    return os.path.join(os.path.dirname(system.CONFIG_DIR), 'esi', ESI_NAME)


DEFAULT_ESI = _default_esi()
ESI_DRIVE = 'esc_4rs485gateway'
EEPROM_CONFIG_DATA = '800E00CC8813F000'  # first 8 SII bytes of the slave: PDI control/config, sync impulse, ext. PDI config
EEPROM_BYTES = 2048

RO = {'access': 'ro', 'category': 'o'}
RW = {'access': 'rw', 'category': 'o'}
RW_PREOP = {'access': 'rw', 'category': 'o', 'pre_op': True, 'backup': True}


def _hex(value, digits=4):
    return f'#x{value:0{digits}X}'


def _data(value, size):
    return value.to_bytes(size, 'little', signed=value < 0).hex().upper()


def _text(parent, tag, text, **attrib):
    element = ET.SubElement(parent, tag, {k: str(v) for k, v in attrib.items()})
    element.text = str(text)
    return element


def _flags(parent, access=None, category=None, pre_op=False, backup=False):
    flags = ET.SubElement(parent, 'Flags')
    if access:
        _text(flags, 'Access', access, **({'WriteRestrictions': 'PreOP'} if pre_op else {}))
    if category:
        _text(flags, 'Category', category)
    if backup:
        _text(flags, 'Backup', 1)


def _subitem(parent, name, type_, bits, offset, subidx=None, flags=None):
    item = ET.SubElement(parent, 'SubItem')
    if subidx is not None:
        _text(item, 'SubIdx', subidx)
    _text(item, 'Name', name)
    _text(item, 'Type', type_)
    _text(item, 'BitSize', bits)
    _text(item, 'BitOffs', offset)
    _flags(item, **(flags or {}))


def _record(parent, name, subitems, comment=None):
    """Record type. subitems: [(name, type, bits, flags)]; sub-index 0 (USINT) is padded to 16 bits."""
    datatype = ET.SubElement(parent, 'DataType')
    if comment:
        datatype.append(ET.Comment(comment))
    _text(datatype, 'Name', name)
    _text(datatype, 'BitSize', 16 + sum(bits for _, _, bits, _ in subitems[1:]))
    offset = 0
    for subidx, (sub_name, sub_type, bits, flags) in enumerate(subitems):
        _subitem(datatype, sub_name, sub_type, bits, offset, subidx, flags)
        offset += 16 if subidx == 0 else bits


def _array_record(parent, name, base, count, bits, flags):
    """Array object with a count in sub-index 0 (the 0x1C0x/0x1C1x layout)."""
    array = ET.SubElement(parent, 'DataType')
    _text(array, 'Name', f'{name}ARR')
    _text(array, 'BaseType', base)
    _text(array, 'BitSize', bits * count)
    info = ET.SubElement(array, 'ArrayInfo')
    _text(info, 'LBound', 1)
    _text(info, 'Elements', count)

    datatype = ET.SubElement(parent, 'DataType')
    _text(datatype, 'Name', name)
    _text(datatype, 'BitSize', 16 + bits * count)
    _subitem(datatype, 'SubIndex 000', 'USINT', 8, 0, subidx=0, flags=flags)
    item = ET.SubElement(datatype, 'SubItem')
    _text(item, 'Name', 'Elements')
    _text(item, 'Type', f'{name}ARR')
    _text(item, 'BitSize', bits * count)
    _text(item, 'BitOffs', 16)
    _flags(item, **flags)


def _object(parent, index, name, type_, bits, flags, category=None, defaults=None, default_data=None):
    """defaults: [(sub name, value, byte size)] documented as the power-on value of each sub-index."""
    obj = ET.SubElement(parent, 'Object')
    _text(obj, 'Index', _hex(index))
    _text(obj, 'Name', name)
    _text(obj, 'Type', type_)
    _text(obj, 'BitSize', bits)
    if default_data is not None or defaults:
        info = ET.SubElement(obj, 'Info')
        if default_data is not None:
            _text(info, 'DefaultData', default_data)
        for sub_name, value, size in defaults or []:
            sub = ET.SubElement(info, 'SubItem')
            _text(sub, 'Name', sub_name)
            _text(ET.SubElement(sub, 'Info'), 'DefaultData', _data(value, size))
    _flags(obj, **{**flags, **({'category': category} if category else {})})


def _pdo_families(drive):
    """[(first PDO index, 'rx'|'tx', actuator, entries)], outputs first."""
    return [(spec[f'{kind}_pdo'], kind, actuator, spec[key])
            for kind, key in (('rx', 'commands'), ('tx', 'states'))
            for actuator, spec in drive['actuators'].items()]


def _port_objects(drive):
    """{(index, sub): {'names': [str], 'access': 'rw'|'ro'}}: every manufacturer object of every port."""
    objects = {}
    for port in range(1, drive['ports'] + 1):
        for base, kind, actuator, entries in _pdo_families(drive):
            for e in entries:
                entry = objects.setdefault((system.object_index(drive, port, e['obj']), e['sub']),
                                           {'names': [], 'access': 'rw' if kind == 'rx' else 'ro'})
                name = f'{actuator.capitalize()}{port} {e["label"]}'
                if name not in entry['names']:
                    entry['names'].append(name)
    return objects


def _dictionary(device, drive):
    ports = drive['ports']
    dictionary = ET.SubElement(ET.SubElement(device, 'Profile'), 'Dictionary')
    types = ET.SubElement(dictionary, 'DataTypes')
    for name, bits in (('BOOL', 1), ('USINT', 8), ('UINT', 16), ('INT', 16), ('UDINT', 32), ('ULINT', 64),
                       ('STRING(4)', 32), ('STRING(10)', 80)):
        datatype = ET.SubElement(types, 'DataType')
        datatype.append(ET.Comment('Std type (see ETG.1020)'))
        _text(datatype, 'Name', name)
        _text(datatype, 'BitSize', bits)
    _record(types, 'DT1018', [
        ('SubIndex 000', 'USINT', 8, RO), ('Vendor ID', 'UDINT', 32, RO), ('Product code', 'UDINT', 32, RO),
        ('Revision', 'UDINT', 32, RO), ('Serial number', 'UDINT', 32, RO)], 'Identity object (0x1018) type')
    _record(types, 'DT10F1', [
        ('SubIndex 000', 'USINT', 8, RO), ('Local Error Reaction', 'UDINT', 32, RW),
        ('Sync Error Counter Limit', 'UINT', 16, RW)], 'Error settings object (0x10F1) type')
    _array_record(types, 'DT1C00', 'USINT', 4, 8, RO)
    _array_record(types, 'DT1C12', 'UINT', ports, 16, RW_PREOP)
    _array_record(types, 'DT1C13', 'UINT', ports, 16, RO)
    for base, kind, actuator, entries in _pdo_families(drive):
        _record(types, f'DT{base:04X}', [('Number of Entries', 'USINT', 8, RW_PREOP)] +
                [(f'Mapping Entry {i}', 'UDINT', 32, RW_PREOP) for i in range(1, len(entries) + 1)])
    port_objects = _port_objects(drive)
    records = sorted({index for index, sub in port_objects if sub})  # 2-element objects: angle low/high
    for index in records:
        access = RW if port_objects[(index, 1)]['access'] == 'rw' else RO
        _record(types, f'DT{index:04X}', [
            ('SubIndex 000', 'USINT', 8, RO), ('Low', 'INT', 16, access), ('High', 'INT', 16, access)])

    objects = ET.SubElement(dictionary, 'Objects')
    string = lambda text: text.encode().hex().upper()  # noqa: E731
    _object(objects, 0x1000, 'Device type', 'UDINT', 32, RO, 'm', default_data=_data(0x1389, 4))
    _object(objects, 0x1001, 'Error register', 'USINT', 8, RO, default_data='00')
    _object(objects, 0x1008, 'Device name', 'STRING(10)', 80, RO, default_data=string('LAN9252-SP'))
    _object(objects, 0x1009, 'Hardware version', 'STRING(4)', 32, RO, default_data=string('n.a.'))
    _object(objects, 0x100A, 'Software version', 'STRING(4)', 32, RO, default_data=string('5.11'))
    _object(objects, 0x1018, 'Identity', 'DT1018', 144, RO, 'm', defaults=[
        ('SubIndex 000', 4, 1), ('Vendor ID', drive['vendor_id'], 4), ('Product code', drive['product_id'], 4),
        ('Revision', drive['revision'], 4), ('Serial number', 0, 4)])
    _object(objects, 0x10F1, 'Error Settings', 'DT10F1', 64, RO, defaults=[
        ('SubIndex 000', 2, 1), ('Local Error Reaction', 1, 4), ('Sync Error Counter Limit', 4, 2)])
    _object(objects, 0x10F8, 'Timestamp Object', 'ULINT', 64, RW)

    for base, kind, actuator, entries in _pdo_families(drive):
        title = 'Receive' if kind == 'rx' else 'Transmit'
        for port in range(1, ports + 1):
            mapping = [(f'Mapping Entry {i}',
                        (system.object_index(drive, port, e['obj']) << 16) | (e['sub'] << 8) | 16, 4)
                       for i, e in enumerate(entries, 1)]
            _object(objects, base + port - 1, f'{actuator.capitalize()}{port} {title} PDO Mapping',
                    f'DT{base:04X}', 16 + 32 * len(entries), RW,
                    defaults=[('Number of Entries', len(entries), 1)] + mapping)

    _object(objects, 0x1C00, 'Sync manager type', 'DT1C00', 48, RO, defaults=[('SubIndex 000', 4, 1)] + [
        (f'SubIndex {i:03d}', i, 1) for i in range(1, 5)])
    gripper = drive['actuators']['gripper']
    for index, name, flags, first in ((0x1C12, 'SM2 PDO assignment', RW, gripper['rx_pdo']),
                                      (0x1C13, 'SM3 PDO assignment', RO, gripper['tx_pdo'])):
        _object(objects, index, name, f'DT{index:04X}', 16 + 16 * ports, flags, defaults=[
            ('SubIndex 000', ports, 1)] + [(f'SubIndex {i:03d}', first + i - 1, 2) for i in range(1, ports + 1)])

    for (index, sub), info in sorted(port_objects.items()):
        if not sub:
            _object(objects, index, ' / '.join(info['names']), 'INT', 16, RW if info['access'] == 'rw' else RO)
    for index in records:
        name = re.sub(r' (Low|Lo)$', '', port_objects[(index, 1)]['names'][0])
        _object(objects, index, name, f'DT{index:04X}', 48, RW if port_objects[(index, 1)]['access'] == 'rw' else RO)


def build_esi(drive):
    root = ET.Element('EtherCATInfo', {'Version': '1.2'})
    vendor = ET.SubElement(root, 'Vendor')
    _text(vendor, 'Id', _hex(drive['vendor_id'], 8))
    _text(vendor, 'Name', drive['vendor_name'])

    descriptions = ET.SubElement(root, 'Descriptions')
    group = ET.SubElement(ET.SubElement(descriptions, 'Groups'), 'Group', {'SortOrder': '0'})
    group_type = drive['group'].replace(' ', '')
    _text(group, 'Type', group_type)
    _text(group, 'Name', drive['group'], LcId=1033)

    device = ET.SubElement(ET.SubElement(descriptions, 'Devices'), 'Device', {'Physics': 'YY'})
    _text(device, 'Type', drive['device_name'], ProductCode=_hex(drive['product_id'], 8),
          RevisionNo=_hex(drive['revision'], 8))
    _text(device, 'Name', drive['device_name'], LcId=1033)
    info = ET.SubElement(device, 'Info')
    state_machine = ET.SubElement(ET.SubElement(info, 'StateMachine'), 'Timeout')
    for tag, value in (('PreopTimeout', 2000), ('SafeopOpTimeout', 9000), ('BackToInitTimeout', 5000),
                       ('BackToSafeopTimeout', 200)):
        _text(state_machine, tag, value)
    mailbox_timeout = ET.SubElement(ET.SubElement(info, 'Mailbox'), 'Timeout')
    _text(mailbox_timeout, 'RequestTimeout', 100)
    _text(mailbox_timeout, 'ResponseTimeout', 2000)
    _text(info, 'IdentificationReg134', 'true')
    _text(device, 'GroupType', group_type)

    _dictionary(device, drive)

    for name in ('Outputs', 'Inputs', 'MBoxState'):
        _text(device, 'Fmmu', name)
    for sm in drive['sync_managers']:
        _text(device, 'Sm', sm['name'], StartAddress=_hex(sm['start']), ControlByte=_hex(sm['control'], 2),
              DefaultSize=sm['default_size'], Enable=1)

    families = _pdo_families(drive)
    for base, kind, actuator, entries in families:
        rivals = [b for b, k, a, _ in families if k == kind and a != actuator]  # same port, other actuator type
        for port in range(1, drive['ports'] + 1):
            pdo = ET.SubElement(device, 'RxPdo' if kind == 'rx' else 'TxPdo',
                                {'Fixed': '1', 'Sm': '2' if kind == 'rx' else '3'})
            _text(pdo, 'Index', _hex(base + port - 1))
            _text(pdo, 'Name', f'{actuator.capitalize()}{port} {"Control" if kind == "rx" else "Feedback"}')
            for rival in rivals:
                _text(pdo, 'Exclude', _hex(rival + port - 1))
            for e in entries:
                entry = ET.SubElement(pdo, 'Entry')
                _text(entry, 'Index', _hex(system.object_index(drive, port, e['obj'])))
                _text(entry, 'SubIndex', f'#x{e["sub"]:X}')
                _text(entry, 'BitLen', 16)
                _text(entry, 'Name', f'{actuator.capitalize()}{port} {e["label"]}')
                _text(entry, 'DataType', 'INT')

    mailbox = ET.SubElement(device, 'Mailbox', {'DataLinkLayer': 'true'})
    ET.SubElement(mailbox, 'CoE', {'SdoInfo': 'true', 'SegmentedSdo': 'false', 'CompleteAccess': 'true',
                                   'PdoAssign': 'true', 'PdoConfig': 'false'})
    op_mode = ET.SubElement(ET.SubElement(device, 'Dc'), 'OpMode')
    _text(op_mode, 'Name', 'Synchron')
    _text(op_mode, 'Desc', 'SM-Synchron')
    _text(op_mode, 'AssignActivate', _hex(drive['assign_activate']))
    _text(op_mode, 'CycleTimeSync0', 0, Factor=1)
    _text(op_mode, 'ShiftTimeSync0', 0)
    eeprom = ET.SubElement(device, 'Eeprom')
    _text(eeprom, 'ByteSize', EEPROM_BYTES)
    _text(eeprom, 'ConfigData', EEPROM_CONFIG_DATA)

    ET.indent(root, space='\t')
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def generate():
    return build_esi(system.load_drive(ESI_DRIVE))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-o', '--output', default=DEFAULT_ESI, help='file to write (default: esi/ in the source tree)')
    parser.add_argument('--check', action='store_true', help='do not write; exit 1 if the file differs')
    args = parser.parse_args(argv)
    text = generate()
    if args.check:
        current = open(args.output, encoding='utf-8').read() if os.path.exists(args.output) else ''
        if current != text:
            print(f'{args.output} is out of date: run `ros2 run dh_ethercat gen_esi`', file=sys.stderr)
            return 1
        print(f'{args.output} is up to date')
        return 0
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
