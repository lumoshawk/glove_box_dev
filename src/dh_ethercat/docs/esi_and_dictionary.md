# ESI and dictionary: how the three sources were reconciled

Three sources describe the box and they disagree.

| Source | Objects | Used for |
|---|---|---|
| `resource/EC盒子字典.xlsx` ("电爪-标准1接4-EC盒子字典") | `0x2000..` per port (+`0x800` per port), PDOs `0x1600/0x1A00` (gripper), 4-in-1 names for the e-cylinder | field names, units, 485 register addresses, value ranges |
| the slave itself (`ethercat pdos/xml/sdos/sii_read -p 3`) | exactly the `0x2xxx/0x3xxx` objects and PDOs below | **authoritative** indices, order, sizes, identity |
| `ref/ESC_4RS485网关用户手册_v2.2.pdf` (ESI `_2_0`) | `0x6000/0x7000/0xF000/0xF001`, PDOs `0x1600..` with other contents | descriptions of actuator semantics only; it documents a different firmware |

The box on the bus (SII vendor `0xDF0`, product `0x14`, revision `0x301`, CoE software `5.11`, LAN9252 based) has no
`0x6000/0x7000/0xF001`. The old map taken from the manual made it refuse SAFEOP (AL `0x001E`).

## What is on the slave

Port X (1..4) uses the object block `0x2000 + 0x800*(X-1)`:

| PDO | Direction | Entries (object offset:sub) |
|---|---|---|
| `0x1600+X-1` Gripper control (11) | SM2 out | 00 initialize, 01 force, 03 position, 04 speed, 05:1 angle lo, 05:2 angle hi, 06 rot. speed, 07 rot. force, 08 rot. relative, 5D rot. stop, 61 rot. reset |
| `0x1A00+X-1` Gripper feedback (12) | SM3 in | 0E init state, 0F grip state, 10 position, 11 speed, 12 current, 13 error, 16:1/16:2 angle lo/hi, 17 rot. init, 18 rot. state, 19 rot. speed, 1A rot. current |
| `0x1610+X-1` E-cylinder control (16) | SM2 out | 00 return home, 01 push force, 09 push band, 03 position, 04 speed, 0A acceleration, 0B relative, 0C jog, 5D stop, 25 push band speed, 68..6D servo position/push band/speed/accel/push force/control word |
| `0x1A10+X-1` E-cylinder feedback (14) | SM3 in | 0E home state, 0F motion state, 10 position, 11 speed, 12 current, 13 alarm, 14 real force, 15 reserve, 6E..73 servo position/current/speed/error/reserve/status word |

All entries are 16-bit INT. Gripper and e-cylinder PDOs of one port are mutually exclusive. Defaults: the slave
assigns all eight PDOs of each direction, which is not a valid combination; the master must assign one family per port.

## Where the xlsx is wrong (the slave wins)

- it lists gripper target position / speed / angle at `0x2002 / 0x2003 / 0x2004 / 0x2005`; the slave has
  `0x2003 / 0x2004 / 0x2005:1 / 0x2005:2` (no `0x2002`), the angle is a 2-sub-index record;
- it lists Grip State at `0x201E`; the slave has `0x200F`;
- it gives rotation-angle range `-35767..35767` (a 16-bit value is `-32768..32767`);
- it has no indices for the e-cylinder: they were taken from the slave's `0x1610/0x1A10` PDOs and matched to the xlsx
  field names by order (16 control / 14 feedback entries match exactly).

Every PDO of the generated ESI equals the slave's own list (checked by `test_esi_matches_the_slave_on_the_bus`).

## Other findings

- SII sync managers: SM0 `0x1000/128`, SM1 `0x1080/128`, SM2 `0x1100/216` control `0x64`, SM3 `0x1400/212` control `0x20`.
  FMMUs: outputs, inputs, mailbox state. Mailbox CoE with SDO info and PDO assign, no PDO configuration.
- CoE `0x1018` = `0x9 / 0x9252 / 0x20111` (LAN9252 stack defaults) while the SII says `0xDF0 / 0x14 / 0x301`; the
  IgH master matches on the SII values.
- CoE object names are garbled; `0x1C32/0x1C33` report a minimum cycle time of 100 µs and free-run (0).
- `0xF000` exists with two sub-indices of unknown meaning; there is no gateway alarm / connection object, so a
  missing actuator can only be recognised by its all-zero feedback.
- The slave refuses SAFEOP with AL `0x001F` when the SM2 watchdog is disabled; the plugin config therefore enables it.

## Regenerating

```bash
ros2 run dh_ethercat gen_esi            # esi/DH_EtherCAT_Box_4Port.xml from config/drives/esc_4rs485gateway.yaml
xmllint --noout --schema EtherCATInfo.xsd esi/DH_EtherCAT_Box_4Port.xml    # schema from the ETG
```
