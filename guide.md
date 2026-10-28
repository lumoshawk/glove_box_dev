```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/glove_box_dev/install/setup.bash

sudo systemctl start ethercat
ethercat master          # should list Master0 and a slave count
ethercat slaves          # currently 6: 0-1 and 3 CU1128 junction, 2 D1616BP IO box (alias 52), 4-5 MDX+_EC
ethercat graph           # shows the topology

```

The CU1128 junction is transparent: it appears as three slaves (0-2) that need no configuration
(no PDOs, no mailbox). The MDX drives are therefore at ring positions 4 and 5, not 0 and 1
(`bus.position` in `config/profiles/motor*.yaml`, `ring` in `glove_box_bringup/config/bringup.yaml`).
Re-run `ethercat slaves` after any re-wiring; if the order changes, update these positions or give
the drives an alias with `ros2 run mdx_ethercat ec_tool set-alias <ring> <alias>`.

## Packages

| Package | Role |
|---|---|
| `mdx_ethercat` | MDX+_EC drives (motor1, motor2): launch, config, read/pp/pv/csp/stress tests |
| `eyou_ethercat` | EYOU PHU/RHU joint modules (eyou1, eyou2): same structure and tests as mdx_ethercat |
| `dh_ethercat` | DH EtherCAT Box 4 Port (4x RS485 grippers, ring 3): ESI, one profile per RS485 device, read / write / return-zero tests; see its README |
| `tuven_ethercat` | TUVEN IO box D1616BP (16 DI / 16 DO, **alias 52**): tool changer proximity sensors (DI 1-2) and lock actuators (DO 1-2), ESI excerpt, read / write tests, `get_tool` / `switch_tool`; see its README |
| `elmo_ethercat` | Elmo Gold EtherCAT drive (**ring position 5**, no SII alias possible; CiA402): launch, config, read / enable tests; see its README (NetworkManager must ignore `eoe*`) |
| `glove_box_bringup` | top level: one master, one `ros2_control_node`, every motor of every package |

Only one launch can own the EtherCAT master at a time. Use either a package's own launch (that
package's motors only) or the top-level launch (all motors). Stop the other first:
`pgrep -af "ros2_control_node|robot_state_publisher"` must be empty before a new launch. A leftover
`robot_state_publisher` (e.g. from an old `ros2 launch ...` that was never stopped) publishes its own
`/robot_description`, and `ros2_control_node` may pick that one instead of the new launch's: stop it with
`kill <pid>` (the `ros2 launch` process first, then its `robot_state_publisher`).

## Run one package on its own

```bash
ros2 launch mdx_ethercat mdx.launch.py
ros2 run mdx_ethercat read_test
ros2 run mdx_ethercat pp_test

ros2 run mdx_ethercat stress_test --ros-args -p motor:=motor1 -p duration_s:=60.0 -p roundtrips:=500
# add -p sdo:=true -p sdo_count:=200 [-p sdo_write:=true] for the mailbox phase
```

```bash
ros2 launch eyou_ethercat eyou.launch.py           # standalone EYOU bus, 1000 Hz
ros2 run eyou_ethercat read_all                    # table of every EYOU motor
ros2 run eyou_ethercat read_test --ros-args -p motor:=eyou1
ros2 run eyou_ethercat pp_test --ros-args -p motor:=eyou1
```

```bash
ros2 launch dh_ethercat dh.launch.py               # standalone DH box (ring 3), 100 Hz; or glove_box_bringup all.launch.py
ros2 run dh_ethercat read_test                     # states of every RS485 device (never writes)
ros2 run dh_ethercat return_zero_test --ros-args -p device:=gripper3   # home a gripper (it moves)
ros2 run dh_ethercat write_test --ros-args -p device:=gripper3         # close / open and check feedback (it moves)
```

```bash
# Elmo Gold drive (ring position 5). One-time setup per PC so NetworkManager ignores the master's EoE interface
# (otherwise the drive's SDO setup times out and it stays in SAFEOP, see src/elmo_ethercat/README.md):
#   sudo cp src/elmo_ethercat/config/networkmanager/99-ethercat-eoe.conf /etc/NetworkManager/conf.d/ && sudo systemctl reload NetworkManager
ros2 launch elmo_ethercat elmo.launch.py           # standalone Elmo drive, 500 Hz; or glove_box_bringup all.launch.py
ros2 run elmo_ethercat read_test                   # states, status word, checks (never writes)
ros2 run elmo_ethercat enable_test                 # enable (PP, hold position), hold 2 s, disable; no motion
ros2 run elmo_ethercat pp_test                     # MOVES: PP +100000 counts from the current position, then back
ros2 run elmo_ethercat pv_test                     # MOVES: PV forward then back (speed_percent / duration_s / cycles in the profile)
ros2 run elmo_ethercat pv_test --ros-args -p speed_percent:=30.0 -p duration_s:=1.0 -p cycles:=3
ros2 topic echo /elmo_gpio/gpio_states --once
```

The Elmo cannot store an alias (its EEPROM is read-only; `ethercat alias` only changes the master's memory until the
next rescan), so it is addressed by ring position: `ethercat slaves` must show it at position 5 (`52:3`).

```bash
ros2 launch tuven_ethercat tuven.launch.py         # standalone IO box (alias 52), 100 Hz; or glove_box_bringup all.launch.py
ros2 run tuven_ethercat read_test                  # sensor1 / sensor2 levels (never writes)
ros2 run tuven_ethercat write_test                 # actuator1: lock, unlock, lock (the lock moves)
ros2 run tuven_ethercat get_tool                   # lock, delay, unlock, sensor stable 3 s, lock
ros2 run tuven_ethercat switch_tool                # lock, delay, unlock, sensor stable 5 s, lock
ros2 topic echo /tuven_gpio/gpio_states --once     # sensor1_input, sensor2_input (high = stable)
ros2 run tuven_ethercat gen_drive --check          # ESI excerpt + drive file match ref/TUVEN_ESI_V7.6.xml
```

The IO box is addressed by its alias (52, written with `ethercat alias -p <ring> 52`), so it does not depend on
its ring position. Logic (`ref/alias_def.md`): sensor high = stable, actuator high = unlock, 0 = locked; outputs start
locked and return to 0 when the master stops. The box has no output read-back: confirm `write_test` on the output LED.

## Command parameters (`--ros-args -p name:=value`)

Every test is a ROS 2 node; pass a parameter with `--ros-args -p <name>:=<value>`, repeat `-p` for
several. Without a parameter the default from the package's `config/*.yaml` is used. No space
around `:=`, booleans are `true`/`false`, and a wrong name is silently ignored.

| Command | Parameter | Default | Meaning |
|---|---|---|---|
| mdx `read_test`, `pp_test`, `pv_test`, `csp_test`, `stress_test` | `motor` | first in `mdx_params.yaml` | which motor profile (`motor1`, `motor2`) |
| eyou `read_test`, `pp_test`, `pv_test`, `csp_test` | `motor` | first in `eyou_params.yaml` | which motor profile (`eyou1`, `eyou2`) |
| mdx/eyou `pp_test`, `pv_test`, `csp_test` | `record` | `false` | `true` writes a CSV under `log/records/` |
| mdx `stress_test` | `duration_s` | `60.0` | test length in seconds |
| mdx `stress_test` | `roundtrips` | `500` | command/feedback roundtrips |
| mdx `stress_test` | `sdo`, `sdo_count`, `sdo_write` | `false`, `200`, `false` | add the mailbox (SDO) phase, its count, and writes |
| dh `read_test` | `gateway`, `device` | first gateway, all devices | which gateway (`dh_box1`) and RS485 device profile (`gripper1`..`gripper4`, `all`) |
| dh `return_zero_test`, `write_test` | `gateway`, `device` | first gateway, **first** device | `device:=all` moves every gripper in turn |
| dh `write_test` | `init` | `false` | home first when the gripper is not initialised |
| tuven `read_test` | `gateway`, `device` | first gateway, all devices | which IO box (`io_box1`) and sensor (`sensor1`, `sensor2`, `all`) |
| tuven `write_test`, `get_tool`, `switch_tool` | `gateway`, `device` | first gateway, **first** actuator | which lock (`actuator1`, `actuator2`); its paired sensor is used automatically |
| tuven `get_tool`, `switch_tool` | `delay_s`, `stable_s`, `timeout_s` | `scripts:` in `tuven_params.yaml` (1 / 3 or 5 / 60 s) | pause before unlock, required stable time, give-up time (the lock closes again) |
| `read_all` (eyou, glove_box_bringup) | `raw` | `false` | also print every raw state interface |
| `enable_test` (glove_box_bringup) | `motors`, `hold_s`, `max_drift_deg` | all, `1.0`, `0.5` | which motors, enable time per motor, drift that aborts the test |
| elmo `enable_test` | `motor`, `hold_s`, `max_drift_counts` | `elmo_m1`, `2.0`, `2000` | which drive, hold time, drift (counts) that aborts the test |
| elmo `pp_test` | `motor`, `move_counts`, `velocity` | `elmo_m1`, `100000`, `10000` | which drive, counts to move forward and then back, profile velocity (counts/s, at most `motion.max_speed`) |
| elmo `pv_test` | `motor`, `speed_percent`, `duration_s`, `cycles` | `elmo_m1`, `50`, `2.0`, `1` | which drive, speed as % of `motion.max_speed`, seconds per leg (max 5), forward+back repetitions (max 10) |

Examples:

```bash
ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor2 -p record:=true
ros2 run eyou_ethercat pv_test --ros-args -p motor:=eyou1
ros2 run mdx_ethercat stress_test --ros-args -p motor:=motor1 -p duration_s:=30.0 -p sdo:=true
ros2 run dh_ethercat write_test --ros-args -p device:=gripper1
ros2 run tuven_ethercat switch_tool --ros-args -p device:=actuator2 -p stable_s:=6.0
```

Values that are not parameters (distances, speeds, force, open/close positions) live in the motor
profile `tests:` section (`config/profiles/<name>.yaml`); edit it and rebuild (or use `--symlink-install`).
List what a running node accepts with `ros2 param list /<node_name>`.

## Run everything from the top level

```bash
ros2 launch glove_box_bringup all.launch.py        # terminal 1, leave running
ros2 run glove_box_bringup read_all                # terminal 2: all motors, one table
ros2 run glove_box_bringup read_all --ros-args -p raw:=true   # plus every raw interface
ros2 run glove_box_bringup enable_test            # enable each motor for 1 s, one by one, no motion
ros2 run glove_box_bringup enable_test --ros-args -p motors:="[elmo_m1]"   # only the Elmo drive
```

Each package keeps its own controller and topics under the combined launch (`/mdx_gpio/...`,
`/elmo_gpio/...`, `/eyou_gpio/...`), so the per-package tests work unchanged, e.g.
`ros2 run eyou_ethercat pp_test --ros-args -p motor:=eyou1`. Only reading and the enable test are at the top level so far.

`read_all` exits non-zero if any listed motor has no data, a fault or an error code.

`enable_test` sets each target position to the measured one (velocity and torque targets stay 0), enables
the motor for `hold_s`, then disables it again before the next one. It resets faults while enabling and
aborts a motor that drifts more than `max_drift_deg`. It exits non-zero if any motor fails to enable, hold
or disable.

## Bus layout (top level)

`src/glove_box_bringup/config/bringup.yaml` lists the motors on the physical bus. Each entry names
the package that owns the motor and its ring position from `ethercat slaves`:

```yaml
bus:
  - {package: mdx_ethercat, motor: motor1, ring: 4}
  - {package: mdx_ethercat, motor: motor2, ring: 5}
  # - {package: eyou_ethercat, motor: eyou1, ring: 6}
  # - {package: eyou_ethercat, motor: eyou2, ring: 7}
  - {package: elmo_ethercat, motor: elmo_m1, ring: 5}   # no SII alias possible: addressed by ring position
```

A motor that is listed but not wired (its alias absent) leaves the shared domain incomplete and every state turns NaN:
comment out the `bus:` entries of drives that are not on the bus (e.g. the MDX ones while only the Elmo is connected).

`ring` overrides `bus.position` in the profiles; without it the list index is used. The junction
is not listed; the DH box and the TUVEN IO box (`ring: 2`, found by alias 52) are under `gateways:` (launched and controlled over `/dh_gpio` and `/tuven_gpio`, but not shown by read_all).
`bus:` entries whose ring position is not on the physical bus are skipped at launch (`skip_absent: true`, printed
as "not on the bus, skipped"): a configured but missing slave leaves the shared domain incomplete and no package
gets feedback. The EYOU entries are commented out until those
modules are on the bus. The cycle rate is shared: 500 Hz, which the MDX needs.

## EYOU notes (not yet confirmed on hardware under this package)

- Identity: vendor `0x1097`, product `0x10002` as discovered. The SDO `0x1018:02` reads `0x20001`;
  the master matches on the discovered value, as in the earlier tested setup.
- `gear_ratio` is 1.0 in the profiles: the manual gives 524288 counts per OUTPUT revolution. The old
  `eyou_speed_test.py` assumed 524288 per motor revolution and 50:1. Turn the output shaft one
  turn by hand and read `position` to settle it. The profile `tests:` are small (10 deg) until then.
- The EYOU was only tested at 1000 Hz. At 500 Hz on a mixed bus it is untested; if it will not reach OP,
  try it alone with `eyou.launch.py` first.
- EYOU exposes `torque`, not `current`; the EYOU copies of the tests print `torque`.
