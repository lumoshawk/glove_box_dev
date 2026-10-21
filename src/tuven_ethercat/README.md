# tuven_ethercat

TUVEN (图稳, TW series) **EtherCAT IO box `D1616BP`** (TWEC-D1616BP: 16 DI PNP/NPN + 16 DO PNP 0.5 A), a CoE slave
controlled over ROS 2 (`/tuven_gpio`) through the IgH master and `ethercat_generic_plugins/GenericEcSlave`.
Station alias **52** (`ref/alias_def.md`). Wired as the tool changer I/O:

| Device | Channel | Meaning |
|---|---|---|
| `sensor1`, `sensor2` | DI 1, 2 (`0x6000:1/2`, "input0:1/2") | proximity sensor, **high voltage = state stable** |
| `actuator1`, `actuator2` | DO 1, 2 (`0x7000:1/2`, "output0:1/2") | lock actuator, **high voltage = active = unlock**, 0 = locked |

```
config/tuven_params.yaml        rates, master id, which boxes to load, get_tool / switch_tool timings
config/gateways/io_box1.yaml    the box: alias 52 + the wired devices
config/profiles/                one file per wired channel (sensor1/2, actuator1/2)
config/drives/d1616bp.yaml      the box's PDO map (generated from the ESI, do not edit)
esi/TUVEN_D1616BP.xml           the D1616BP device excerpted from ref/TUVEN_ESI_V7.6.xml
launch/tuven.launch.py          standalone launch (or use glove_box_bringup all.launch.py)
tuven_ethercat/                 system.py (config -> ros2_control), esi.py, io_client.py, tool_logic.py, the scripts
test/                           pytest: config, ESI, PDO map, sequences with a fake clock, live identity check
```

## 1. How the package was built, step by step

1. **Identify the slave.** `ethercat slaves -v` showed `D1616BP` (vendor `0x345`, product `0x6d` = 109, revision
   `0x20111`). The manual (`EtherCAT一体式远程IO及网关...V1.2.pdf`, §4.4.2) says what it is: 16 DI PNP/NPN + 16 DO PNP, 24 V,
   0.5 A per output with short-circuit protection. The naming figure decodes the model: `D` digital, `16` inputs,
   `16` outputs, `BP` inputs PNP&NPN compatible, outputs PNP.
2. **Get the ESI.** The vendor ESI `ref/TUVEN_ESI_V7.6.xml` describes 44 TUVEN devices. The device is matched by
   `Type`/`ProductCode`/`RevisionNo` (`D1616BP`, 109, `#x00020111`: the same numbers the slave reports). It lists the sync managers
   (SM0/1 mailbox, SM2 outputs `0x64`, SM3 inputs `0x20`) and one fixed PDO each way: `RxPdo 0x1600` = `0x7000:1..16`
   `Data Out1..16`, `TxPdo 0x1A00` = `0x6000:1..16` `Data In1..16`, all 1-bit BOOL.
3. **Excerpt it.** `gen_drive` copies the vendor block and only this device into `esi/TUVEN_D1616BP.xml`.
4. **Generate the drive file.** The same command reads the excerpt and writes `config/drives/d1616bp.yaml` in the format
   the generic slave plugin loads: ids, `sm` (direction from the control byte, watchdog from bit 6: SM2 on, SM3 off),
   and per PDO the channels with `type: bool` and `mask` = the bit inside its byte (1, 2, 4 ... 128, then again for
   the second byte). Each channel is named `in<n>` / `out<n>` after its sub-index.
5. **Probe the real slave.** `ethercat sdos/upload -p 2`: it is a CoE slave with *no* `0x6000/0x7000/0x1600/0x1A00` objects in its
   dictionary (only `0xF030` module list `0x1016`, `0x2016`); the PDO layout exists only in its SII and ESI. So the
   master must configure exactly the ESI map: the whole fixed 16 + 16 bit PDO, nothing smaller.
6. **Profiles.** One yaml per wired channel (`config/profiles/`), listed in `config/gateways/io_box1.yaml`. The alias, the
   channel numbers and the active levels come from `ref/alias_def.md`.
7. **Alias.** `ethercat alias -p 2 52` wrote 52 into the slave's SII EEPROM (word 4). `ethercat slaves` now lists it as
   `52:0`. The box is addressed by `alias 52 + position 0`, so it can move on the ring (CU1128 junction) without
   touching the config. The ESC loads the alias at power-up: power-cycle once to make it permanent in hardware.
8. **ros2_control files.** `system.py` turns drive + gateway + profiles into the `<ros2_control>` xml, the controller yaml and the
   slave config. Wired channels get interfaces: state `sensor1_input`, command `actuator1_output` (raw level 0/1).
9. **Scripts and tests.** `read_test`, `write_test`, `get_tool`, `switch_tool`, and the pytest suite.

## 2. Run

Only one process can own the EtherCAT master (`pgrep -af "ros2_control_node|robot_state_publisher"` must be empty).

```bash
source /opt/ros/jazzy/setup.bash
cd ~/glove_box_dev && colcon build --packages-select tuven_ethercat --symlink-install && source install/setup.bash
ros2 launch tuven_ethercat tuven.launch.py          # the box alone, 100 Hz; `ethercat slaves` must show it in OP
```

```bash
ros2 topic echo /tuven_gpio/gpio_states             # sensor1_input, sensor2_input
# unlock actuator 1 by hand (and lock again with 0.0)
ros2 topic pub --once /tuven_gpio/commands control_msgs/msg/DynamicInterfaceGroupValues \
 "{interface_groups: [io_box1], interface_values: [{interface_names: [actuator1_output], values: [1.0]}]}"
```

The controller latches commands. At start-up every output holds its `startup` value (**0 = locked**), and the box
outputs 0 again when the master stops (SM2 process data watchdog), so a lock closes by itself.

## 3. Tests and scripts

All are ROS nodes talking to `/tuven_gpio`; parameters via `--ros-args -p name:=value`; exit code 0 = pass.

| Command | Does |
|---|---|
| `ros2 run tuven_ethercat read_test` | prints every sensor level, never writes. PASS: levels arrive (and `expect_stable` is met when set in the profile) |
| `ros2 run tuven_ethercat write_test [-p device:=actuator2]` | lock, unlock for `hold_s`, lock; prints the paired sensor. **MOVES the lock.** First actuator by default. The box has no output read-back: look at the output LED / lock |
| `ros2 run tuven_ethercat get_tool [-p device:=actuator2]` | actuator **lock** (slot empty), delay, **unlock** (tool is attached now), paired sensor stable for 3 s, **lock** |
| `ros2 run tuven_ethercat switch_tool [-p device:=actuator2]` | the same with a tool attached, sensor stable for 5 s |

`get_tool` / `switch_tool` timings are `scripts:` in `config/tuven_params.yaml` (`delay_s`, `stable_s`, `timeout_s`),
or one run: `-p stable_s:=6.0`. "Stable" means the sensor stays at its stable level (high) for the whole time; any
drop restarts the count. On timeout, error or Ctrl-C the actuator is locked again and the exit code is 1.
Assumptions: the delay is 1 s (not in `alias_def.md`) and the timeout 60 s. Because the sensor is already high with
a tool in place, `switch_tool` finishes once the sensor has been stable for 5 s after the unlock; it does not wait for the old
tool to leave first.

Package tests (offline, plus one live check of identity and alias when the box is on the bus):

```bash
cd src/tuven_ethercat && python3 -m pytest test -q
```

## 4. Configure

**Add a wired channel:** copy a profile in `config/profiles/` (set `type` `sensor`/`actuator` and `channel` 1..16) and list it
in `config/gateways/io_box1.yaml`. Unlisted channels stay in the PDO but have no interface.

**Another box:** copy `config/gateways/io_box1.yaml` (its own alias, devices with unique names and matching `gateway:`),
add it under `motors:` in `tuven_params.yaml` and in `glove_box_bringup/config/bringup.yaml` under `gateways:`.

**Alias:** `ros2 run mdx_ethercat ec_tool set-alias <ring> <alias>` (or `ethercat alias -p <ring> <alias>`), with no launch running.

**Another TUVEN model:** `gen_drive --model <model>` extracts it; `drive_from_esi` only handles BOOL entries so far
(digital modules) and tells you when it meets another type.

## 5. The ESI file

```bash
ros2 run tuven_ethercat gen_drive            # rewrite esi/TUVEN_D1616BP.xml and config/drives/d1616bp.yaml from ref/
ros2 run tuven_ethercat gen_drive --check    # exit 1 if they are out of date
```

`pytest` fails if the drive file and the ESI excerpt disagree. Import the ESI excerpt into TwinCAT
(`C:\TwinCAT\3.1\Config\Io\EtherCAT`) or any master that wants an ESI.

## 6. Troubleshooting

| Symptom | Cause |
|---|---|
| `Failed to reserve master: Device or resource busy` | another launch owns the master; stop it |
| slave stays PREOP, `dmesg` AL status `0x001E` / `0x001F` | PDO map not matching the slave: do not shrink the 16 + 16 bit PDO |
| `No state from /tuven_gpio/gpio_states` | launch not running, slave not in OP |
| `ethercat slaves` shows `0:<n>` instead of `52:0` | the alias was not written; `ethercat alias -p <ring> 52` |
