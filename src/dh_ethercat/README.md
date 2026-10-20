# dh_ethercat

DH-Robotics **EtherCAT Box 4 Port** (`ESC_4RS485Gateway`): a CoE slave that bridges EtherCAT to four RS485 ports,
one DH gripper (PGE / PGC / PGS / PGEA ...) or e-cylinder per port. It runs under the IgH master through
`ethercat_generic_plugins/GenericEcSlave` and is controlled over ROS 2 (`/dh_gpio`). Each RS485 device has its own
profile, and the package ships the slave's ESI file.

```
config/dh_params.yaml          rates, master id, which gateways to load
config/gateways/dh_box1.yaml   the box: ring position + the RS485 devices wired to it
config/profiles/gripper1..4    one RS485 device each: port, type, startup values, limits, test parameters
config/drives/esc_4rs485gateway.yaml   the box's object dictionary: PDO map (source of truth)
esi/DH_EtherCAT_Box_4Port.xml  ESI generated from the drive file (TwinCAT / other masters)
resource/EC盒子字典.xlsx        vendor dictionary the drive file was derived from
launch/dh.launch.py            standalone launch (or use glove_box_bringup all.launch.py)
dh_ethercat/                   system.py (config -> ros2_control), esi.py, dh_client.py, the three tests
test/                          pytest: config, PDO map, ESI, and a live comparison with the slave on the bus
docs/esi_and_dictionary.md     how the xlsx / manual / slave were reconciled
```

## 1. Run

Only one process can own the EtherCAT master. Stop any other launch first
(`pgrep -af "ros2_control_node|robot_state_publisher"` must be empty).

```bash
source /opt/ros/jazzy/setup.bash
cd ~/glove_box_dev && colcon build --packages-select dh_ethercat glove_box_bringup && source install/setup.bash

ros2 launch dh_ethercat dh.launch.py              # the box alone, 100 Hz
ros2 launch glove_box_bringup all.launch.py       # the box together with the MDX / EYOU drives, 500 Hz
```

The box is in `glove_box_bringup/config/bringup.yaml` under `gateways:` (`ring: 3`), so the combined launch maps
it into the same `ros2_control_node`. `ethercat slaves` must show it in `OP`. If MDX / EYOU drives listed in
`bus:` are absent from the physical bus, the shared domain stays incomplete and **no** package gets feedback
(states are NaN): remove the absent ones from `bus:` or use `dh.launch.py`.

## 2. The ROS interface

One GPIO controller, `dh_gpio` (`gpio_controllers/GpioCommandController`), group = gateway name (`dh_box1`).
Interface name = `<device>_<field>`:

| Topic | Type | Content |
|---|---|---|
| `/dh_gpio/gpio_states` | `control_msgs/DynamicInterfaceGroupValues` | all state interfaces, e.g. `gripper3_position` |
| `/dh_gpio/commands` | `control_msgs/DynamicInterfaceGroupValues` | command interfaces; a message may list any subset |

```bash
ros2 topic echo /dh_gpio/gpio_states
# home gripper 1, then open it: write initialize = 1 ... later set it back to 0
ros2 topic pub --once /dh_gpio/commands control_msgs/msg/DynamicInterfaceGroupValues \
 "{interface_groups: [dh_box1], interface_values: [{interface_names: [gripper1_initialize], values: [1.0]}]}"
```

The controller latches every command: a value stays until you write another one. **At start-up each command holds
its `startup` value** (profile) or the drive-file default; for a gripper that is force 50 %, speed 50 %, target
position 1000 (open). An initialised gripper therefore opens when the launch starts.

Gripper fields (all 16-bit integers; ranges are enforced by the gripper, not by the box):

| Command | Meaning |
|---|---|
| `initialize` | `1` home to the zero end stop, `0xA5` (165) full re-calibration. Set back to `0` afterwards. Wait for `init_state` = 1 before other commands |
| `force` 20..100 % / `speed` 1..100 % / `target_position` 0..1000 ‰ (0 closed) | the normal open/close control |
| `angle_lo`, `angle_hi`, `rotation_speed`, `rotation_force`, `rotation_relative`, `rotation_stop`, `rotation_reset` | rotary grippers only |

| State | Meaning |
|---|---|
| `init_state` | 0 not initialised, 1 initialised, 2 initialising |
| `grip_state` | -1 init incomplete, 0 moving, 1 reached (no object), 2 object gripped, 3 object dropped |
| `position` (‰), `speed`, `current` (relative), `error_code` | feedback; error 1 undervolt, 2 overvolt, 3 overcurrent, 4 overheat, 8 overload, 11 overspeed, +100 per unanswered command |
| `angle_lo/hi`, `rotation_*` | rotary grippers |

E-cylinder fields (`type: ecylinder`) are listed in the drive file (`return_home`, `push_force`, `target_position` in
0.01 mm, `speed`, `acceleration`, `jog`, `stop`, ... / `home_state`, `motion_state`, `position`, `alarm_code`, ...).

## 3. The tests

All are ROS 2 nodes that talk to `/dh_gpio`, so they work with either launch. Pass parameters with
`--ros-args -p name:=value`.

```bash
ros2 run dh_ethercat read_test                                   # all devices, never writes anything
ros2 run dh_ethercat read_test --ros-args -p device:=gripper3
ros2 run dh_ethercat return_zero_test --ros-args -p device:=gripper3   # homes: the gripper MOVES to its zero stop
ros2 run dh_ethercat write_test --ros-args -p device:=gripper3         # close then open, checks position feedback (MOVES)
ros2 run dh_ethercat write_test --ros-args -p device:=gripper3 -p init:=true   # home first if needed
```

| Test | Does | Pass |
|---|---|---|
| `read_test` | prints every state of the device(s) | the port answers (all zeros = nothing connected: FAIL) and `error_code` <= `tests.read_test.max_error_code` |
| `return_zero_test` | `initialize = 1`, waits for `init_state = 1`, then `initialize = 0` | initialised within `tests.return_zero_test.timeout_s`, `error_code` 0 |
| `write_test` | sets force/speed, moves to `close_position`, then `open_position` | position within `tolerance` of each target (or object gripped), `error_code` 0 |

`device:=` takes a profile name or `all`; `return_zero_test` and `write_test` default to the **first** device so a
bare run never moves every gripper. Parameters: `gateway` (default: first in `dh_params.yaml`), `device`, and for
`write_test` `init`. Exit code 0 = all passed. Only grippers are driven by the tests; an e-cylinder is still
mapped and usable over the topics.

## 4. Configure

**Add / remove a device:** a device is one file in `config/profiles/` (copy `gripper1.yaml`: set `port`, `type`) and one
line under `devices:` in `config/gateways/dh_box1.yaml`. Only listed devices get their PDOs mapped. List only what is
physically connected: a listed port with nothing behind it just reads zeros (OP is still reached), and
`read_test` reports it as FAIL. An actuator on a port whose profile is not listed is simply not served.
The combined launch skips `bus:` motors that are not physically on the bus (`skip_absent` in `bringup.yaml`);
without that, one missing slave leaves the shared domain incomplete and every state reads NaN.

**One actuator type per port.** The gripper PDO and the e-cylinder PDO of a port exclude each other (also in the
ESI); `type:` in the profile picks which one is mapped.

**Several boxes:** copy `config/gateways/dh_box1.yaml` to `dh_box2.yaml` (new `bus.position`, its own devices; device
profile names must be unique and their `gateway:` must match), add it under `motors:` in `dh_params.yaml` and, for
the combined launch, under `gateways:` in `bringup.yaml`.

**Ring position** (`bus.position`, 3 behind the CU1128 junction): check with `ethercat slaves`.

After editing: `colcon build --packages-select dh_ethercat` (or `--symlink-install`) and relaunch.

## 4a. Notes on the box

- Identity for the master is the **SII** one: vendor `0xDF0`, product `0x14`, revision `0x301`. The CoE `0x1018` of this
  firmware reports `0x9 / 0x9252` (LAN9252 defaults): ignore it.
- SM2 keeps its process-data watchdog (the box refuses SAFEOP with AL `0x001F` otherwise): the master cycle must keep
  running; if it stops, the box leaves OP.
- No distributed clocks (`assign_activate 0`). The gateway refreshes the grippers over RS485 in ~30 ms, so a
  faster EtherCAT cycle only repeats the data.
- The object names the slave returns over CoE are garbled (firmware bug); the ESI file carries the real names.
- This firmware has **no** `0x6000/0x7000/0xF001` objects, unlike `ref/ESC_4RS485网关用户手册_v2.2.pdf`. See
  `docs/esi_and_dictionary.md`.

## 5. The ESI file

`esi/DH_EtherCAT_Box_4Port.xml` is generated from the drive file and validates against `EtherCATInfo.xsd`:

```bash
ros2 run dh_ethercat gen_esi            # rewrite esi/ in the source tree
ros2 run dh_ethercat gen_esi --check    # exit 1 if it is out of date
```

Import it into TwinCAT (`C:\TwinCAT\3.1\Config\Io\EtherCAT`) or any master that wants an ESI. Change PDOs only in
`config/drives/esc_4rs485gateway.yaml`, then regenerate; `pytest` fails if the file and the drive file disagree.

## 6. Tests of the package

```bash
cd src/dh_ethercat && python3 -m pytest test -q
```

Offline checks of the configs, the generated slave config and the ESI; the last test compares the ESI's PDO list
with `ethercat xml -p <ring>` of the real slave and is skipped when no master / box is present.

## 7. Troubleshooting

| Symptom | Cause |
|---|---|
| `Failed to reserve master: Device or resource busy` | another launch owns the master; stop it |
| slave stays PREOP with `E`, `dmesg`: `AL status 0x001E` / `0x001F` | PDO map not matching the slave / SM2 watchdog disabled; use this package's map |
| `No state from /dh_gpio/gpio_states` | launch not running, slave not in OP, or (combined launch) all states are NaN because other listed slaves are missing |
| a device reads all zeros | nothing answers on that RS485 port: check wiring/power; if the actuator was connected or powered after the box, power-cycle the box and relaunch (seen once with ports 1, 2, 4, which later read fine). `ethercat upload -p 3 0x2010 0 --type int16` (port 1 position; `0x2810`, `0x3010`, `0x3810` for ports 2-4) shows whether the box itself has data |
| gripper ignores commands | not initialised (`init_state` 0): run `return_zero_test` |
