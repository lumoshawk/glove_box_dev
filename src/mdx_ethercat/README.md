# MDX+ EtherCAT servo control with ROS 2 

Motor: MDX+ `mdxrr42j5becs000` (EtherCAT, CiA402 profile), ROS 2 Jazzy, IgH EtherCAT master.

## 1. Run it

```bash
source /opt/ros/jazzy/setup.bash
cd ~/glove_box_dev && colcon build --packages-select mdx_ethercat
source install/setup.bash

# terminal 1: master + driver + controller (leave running)
ros2 launch mdx_ethercat mdx.launch.py

# terminal 2
ros2 run mdx_ethercat read_test   # read all states + checks
ros2 run mdx_ethercat pp_test     # enable, rotate 90 deg in PP mode, monitor, disable
ros2 run mdx_ethercat pv_test --ros-args -p record:=true  # bounded constant-speed test
```

Prerequisite: the IgH master service runs and `ethercat slaves` lists `MDX+_EC`
(`/dev/EtherCAT0` must exist).

Related packages in `src/`: `eyou_ethercat` drives the EYOU PHU/RHU joint modules with the same
structure and tests, and `glove_box_bringup` starts one master for the MDX and EYOU motors together
(`ros2 launch glove_box_bringup all.launch.py`, `ros2 run glove_box_bringup read_all`). This package
still runs on its own. Only one launch can own the master at a time. See `guide.md` and `cheatsheet.md`.

## 2. EtherCAT in five minutes

- **Master / slave**: the PC is the master, each drive is a slave. Frames travel through all slaves in a
  ring; each slave reads its output data and inserts its input data "on the fly".
- **AL state machine** (the slave's network state): `INIT -> PREOP -> SAFEOP -> OP`.
  The master configures the slave in PREOP, starts cyclic data in SAFEOP, and the drive only obeys
  commands in OP. Check with `ethercat slaves`.
- **Two kinds of data**
  - **PDO** (process data): small, fixed, exchanged *every cycle* (here 500 Hz). Used for control word,
    target position, status word, actual position.
  - **SDO** (service data): on-demand read/write of any object in the drive's *object dictionary*
    (parameters). Slow, not cyclic.
- **Object dictionary**: every value in the drive has an address `index:subindex`, e.g. `0x6040:0` is
  the control word. The list for this drive is in `ref/AMA Servo EtherCAT v3.4.22.xml` (the ESI file)
  and `ref/MDX-EtherCAT-UserManual-*.pdf`.
- **PDO mapping**: you choose which objects go into the cyclic frame. Master -> drive are the **RxPDOs**
  (called `rpdo` in our yaml; this drive uses `0x1600` and the velocity-mode `0x1603`), drive -> master
  are the **TxPDOs** (`tpdo`, `0x1A00`).
- **DC (distributed clocks)**: slaves share a synchronized clock; `assign_activate: 0x0300` enables it.

## 3. CiA402 (the motor profile)

All EtherCAT servo drives use the same state machine, driven by the **control word** (`0x6040`) and
reported by the **status word** (`0x6041`):

```
Switch on disabled --0x06--> Ready to switch on --0x07--> Switched on --0x0F--> Operation enabled
        ^                                                                           |
        +------------------ 0x80 (fault reset) <------------ Fault ----------------+
```

| Object | Name | Meaning |
|---|---|---|
| `0x6040` | control word | what we ask (0x06 / 0x07 / 0x0F / 0x1F / 0x80) |
| `0x6041` | status word | `&0x6F`: `0x21` ready, `0x23` switched on, `0x27` enabled; bit3 = fault |
| `0x6060` / `0x6061` | mode of operation / display | `1` = PP, `3` = PV, `8` = CSP |
| `0x607A` | target position | in drive counts (10000 per revolution) |
| `0x60FF` | target velocity | pulses/s in Profile Velocity mode |
| `0x6081/83/84` | profile velocity / accel / decel | counts/s, counts/s^2 |
| `0x6064` | actual position | counts |
| `0x606C` | actual velocity | pulses/s (per the MDX+ EtherCAT manual) |
| `0x603F` | error code | e.g. `0x7500` communication error |

**Profile Position move handshake** (what `pp_test.py` does):
1. write `target_position`, then control word `0x1F` (`0x0F` + bit 4 "new set-point");
2. wait for status bit 12 (set-point acknowledged);
3. set control word back to `0x0F` (drop bit 4);
4. wait for status bit 10 (target reached).

PP is a finite position trajectory, so `0x606C` ramps up and down instead of staying constant.
For a steady-speed check, use Profile Velocity mode (`0x6060 = 3`): `pv_test` commands `0x60FF`
for the configured duration, releasing the manual's velocity-mode Halt bit only after the target is
staged. It then asserts Halt, commands zero, and waits for deceleration. Its speed and duration
are in `config/profiles/motor1.yaml`; speed is capped by `motion.max_speed` and the test duration
cannot exceed 5 seconds. Steady speed is verified from a 250 ms position window: measured speed
must remain within 20% of the target for one second. The test aborts if that measured speed exceeds
the greater of 1.5x the request or request + 200 pulses/s, and first verifies that the live
controller exports `target_velocity`. The MDX+ profile lowers `0x2A5A` from its 20000 default to
5000 (500 Hz) to reduce speed overshoot/vibration per the manual.
The target is mapped through the ESI's velocity RxPDO `0x1603`, not added to position RxPDO `0x1600`.
Record with
`-p record:=true` and plot to compare `target_velocity` with measured `velocity`.

## 4. System architecture

```
 read_test.py / pp_test.py / pv_test.py (your code, plain rclpy node: mdx_client.py)
      |  ^   topics: /mdx_gpio/commands  (publish)   /mdx_gpio/gpio_states (subscribe)
      v  |
 mdx_gpio  = gpio_controllers/GpioCommandController      (ros2_control controller)
      |  ^   "command interfaces" / "state interfaces" (named float values)
      v  |
 EthercatDriver (ethercat_driver)     <- ros2_control hardware plugin (SystemInterface)
      |  ^
 GenericEcSlave (ethercat_generic_plugins) <- one per drive, reads its generated slave yaml
      |  ^   RxPDO / TxPDO, every cycle (500 Hz)
      v  |
 IgH EtherCAT master (kernel module, /dev/EtherCAT0)
      |  ^
   EtherCAT cable --> MDX+ drive
```

ROS 2 concepts used:
- **Node / topic**: processes exchange messages on named topics (`ros2 topic list`).
- **ros2_control**: a framework with a real-time loop (`controller_manager`, 500 Hz).
  *Hardware interface* talks to the device; *controllers* expose it to ROS.
  Data crosses between them as named **command interfaces** (you write) and **state interfaces**
  (you read).
- **Launch file**: starts several nodes with parameters in one command.
- **colcon / package**: `colcon build` compiles, `source install/setup.bash` makes the packages visible.

Source layout:

```
src/ethercat_ros2_driver_mod-main/   third-party driver (EthercatDriver, GenericEcSlave, ...)
src/mdx_ethercat/                    our package
  launch/mdx.launch.py               starts everything
  config/mdx_params.yaml             system settings + list of motors
  config/drives/<type>.yaml          one drive model: identity, PDO map
  config/profiles/<motor>.yaml       one motor: bus address, gearing, calibration, limits
  mdx_ethercat/system.py             yaml -> generated URDF / controller yaml / slave yaml
  mdx_ethercat/mdx_client.py         ROS node wrapper for one motor (state in, commands out)
  mdx_ethercat/read_test.py          read test
  mdx_ethercat/pp_test.py            PP motion test
  mdx_ethercat/pv_test.py            bounded PV steady-speed test
  mdx_ethercat/csp_test.py           CSP motion test
  mdx_ethercat/util/ec_tool.py       scan the bus, set alias, set zero
ref/                                 ESI xml, manual, reference yaml
```

## 5. The config files, one by one

Three levels, from general to specific:

| File | Level | Content |
|---|---|---|
| `config/mdx_params.yaml` | whole system | cycle frequency, master id, which motors to load |
| `config/drives/<type>.yaml` | one drive MODEL | identity, PDO map, startup SDOs a profile may set |
| `config/profiles/<motor>.yaml` | one MOTOR | which drive, bus address, gearing, calibration, motion limits, SDO values, test values |

`mdx.launch.py` reads them and **generates** the ros2_control URDF, the controller yaml and one slave
yaml per motor (in a temp folder, `/tmp/mdx_xxxx/`). You do not write xacro or a controller yaml by hand.

### 5.1 `config/mdx_params.yaml` -- system-wide

```yaml
cycle_frequency: 500    # Hz; EtherCAT PDO cycle and controller_manager update rate
master_id: 0            # /dev/EtherCAT<n>
client_rate_hz: 500     # how often the Python scripts publish commands (= cycle_frequency for smooth CSP)
motors:                 # profiles to load, one drive on the bus each
  - motor1
```

### 5.2 `config/drives/mdx_plus_ec.yaml` -- a drive model

```yaml
vendor_id: 0x00000168       # from the ESI / `ec_tool scan`; must match the drive
product_id: 0x00000011
assign_activate: 0x0300     # enable distributed clock

parameters:                 # names a profile may set; each becomes one startup SDO write
  max_profile_velocity: {index: 0x607F, sub_index: 0, type: uint32}
  software_limit_min:   {index: 0x607D, sub_index: 1, type: int32}

rpdo:                       # master -> drive (RxPDO)
  - index: 0x1600
    channels:
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word, default: 0}
tpdo:                       # drive -> master (TxPDO)
  - index: 0x1A00
    channels:
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: status_word}
```

Each `channel` is one object in the cyclic frame: its address (`index`/`sub_index`), its `type`
(must match the object size in the ESI) and the **interface name** it is bound to.
`command_interface` = value written by us, `state_interface` = value read from the drive.
The launch takes the interface names straight from these channels, so adding a channel here is enough.
`vendor_id` + `product_id` identify the **model**; they are how the driver finds a compatible slave.

A new drive model = a new file in `config/drives/`. Copy the PDO map from its ESI/manual.

### 5.3 `config/profiles/motor1.yaml` -- one motor

```yaml
drive: mdx_plus_ec        # config/drives/mdx_plus_ec.yaml
bus: {alias: 0, position: 0}   # which physical drive this is (see section 11)
mechanics: {pulses_per_rev: 10000, gear_ratio: 1.0}
calibration: {direction: 1, zero_offset: 0}
motion: {velocity: 10000, accel: 20000, decel: 20000, max_speed: 10000}
parameters: {}            # startup SDOs, e.g. {max_profile_velocity: 20000}
tests: {pp_test: {move_degrees: 360.0}, pv_test: {velocity: 3000, duration_s: 1.0},
        csp_test: {segments: [[360, 0.8], [360, 0.4]]}}
```

| Section | Used for |
|---|---|
| `bus` | the launch tells the driver which slave to bind to this motor (`alias`, `position`) |
| `mechanics` | counts per output revolution = `pulses_per_rev * gear_ratio`; degrees in scripts convert with it |
| `calibration` | `direction` flips the sign of moves; `zero_offset` is the drive count of output zero, set with `ec_tool set-zero` and used by `position_deg()` |
| `motion` | PP profile defaults and the 100 % speed for `csp_test` |
| `parameters` | startup SDOs written once in PREOP (limits, following error window, max torque, homing...) |
| `tests` | values used by `pp_test` and `csp_test` for this motor |

Common CiA402 parameters (all of them exist in this drive's ESI, see the `parameters:` table in the drive file):

| Object | Name | Typical use |
|---|---|---|
| `0x607F` | max profile velocity | drive-side cap on the profile speed |
| `0x6065` | following error window | fault if the position error is larger |
| `0x607D` | software position limits (min/max) | refuse to move out of range |
| `0x6072` | max torque (per mille) | protect the mechanics |
| `0x6085` | quick stop deceleration | how fast a quick stop brakes |
| `0x607E` | polarity | reverse direction inside the drive |
| `0x607C` | home offset | zero shift after homing |
| `0x6098` / `0x6099` / `0x609A` | homing method / speeds / acceleration | homing mode (6) |

### 5.4 `mdx_ethercat/system.py` -- from yaml to ros2_control

`generate(params)` loops over `motors:` and for each one writes:
1. a slave yaml = drive file (`parameters` removed) + `sdo:` list built from the profile's `parameters`;
2. a `<gpio name="motor1">` with the command/state interfaces of the drive and one `<ec_module>`
   (`GenericEcSlave`, `alias`/`position` from `bus`, `slave_config` = the yaml of step 1);

then the full URDF (one `EthercatDriver` system, one master) and the controller yaml
(`gpios: [motor1, ...]`). The functions `load_params / load_profile / load_drive` are also used by the scripts.

### 5.5 `launch/mdx.launch.py` -- start it all

```python
params = system.load_params()
robot_description, controllers = system.generate(params)      # generated files
robot_state_publisher = Node(package='robot_state_publisher', ...)   # publishes /robot_description
control_node = Node(package='controller_manager', executable='ros2_control_node', parameters=[controllers])
spawner = Node(package='controller_manager', executable='spawner', arguments=['mdx_gpio'])
```

Flow: `robot_state_publisher` publishes the URDF -> `ros2_control_node` loads `EthercatDriver`, which requests
the master, configures every slave from its yaml (SDOs first), goes to OP and starts the cycle -> the controller
`mdx_gpio` is activated and publishes the states of all motors.

### 5.6 `setup.py`

`data_files` copies config/launch files into `install/.../share/mdx_ethercat` (globs, so new yaml files are
picked up on the next build); `entry_points` makes `ros2 run mdx_ethercat read_test` work.

## 6. The Python code

`mdx_client.py` hides the message format. `client.state` is a dict of the latest states, and
`client.command` is a dict that is republished every 20 ms:

```python
class MdxClient(Node):
    def __init__(self, name='mdx_client'):
        super().__init__(name)
        self.motor = self.declare_parameter('motor', ...).value      # ROS parameter: which profile
        self.profile = system.load_profile(self.motor)
        self.state = {}
        self.command = system.command_defaults(...)   # interface names + defaults from the drive yaml
        self.create_subscription(DynamicInterfaceGroupValues, '/mdx_gpio/gpio_states', self._on_state, 10)
        self.command_pub = self.create_publisher(DynamicInterfaceGroupValues, '/mdx_gpio/commands', 10)
        self.create_timer(1 / client_rate_hz, self._send_command)   # keeps commands streaming
```

Commands must be streamed continuously: the controller holds the last value, and the drive expects a
fresh PDO each cycle. So a script only needs to **change a dict value**:

```python
cmd['target_position'] = start + node.deg_to_counts(90)   # includes gearing and direction
cmd['control_word'] = CW_NEW_SETPOINT        # 0x1F, rising edge of bit 4 starts the move
node.spin_until(lambda: node.status_word() & SW_SETPOINT_ACK, 5)   # wait for the drive's answer
```

`deg_to_counts(deg)`, `counts_to_deg(counts)` and `position_deg()` convert with the motor's `mechanics`/`calibration`.
`spin_until(condition, timeout)` runs the ROS callbacks (so `state` updates) until the condition holds.
`pp_test.py` uses it to walk the state machine: write control word -> wait for the matching status word.

## 7. Next steps for development

**Read/command another object** (e.g. torque `0x6077`):
1. add a channel to the `rpdo` or `tpdo` of `config/drives/<type>.yaml` with a new interface name;
2. rebuild, relaunch: the URDF, the controller yaml and `node.state` / `node.command` pick the name up automatically;
3. read it in Python via `node.state['name']`, or write via `node.command['name']`.

Not every object can be mapped to a PDO; check the "PDO mapping" column in the ESI/manual.

**Other modes**: change `mode` (`0x6060`): `1` PP, `3` PV (target velocity `0x60FF`), `4` torque
(`0x6071`), `8` CSP (cyclic position, set `target_position` every cycle), `9` CSV. Add the needed
objects to the PDO map first.

`pv_test` uses mode `3`, holds a fixed `target_velocity`, then commands zero so the drive decelerates
using `motion.decel`. The configured duration includes the acceleration ramp.

**Homing** is mode `6` with its own handshake (see the manual).

**SDO access** (parameters not in PDO, one-off settings): use the CLI, e.g.
`ethercat upload -p 0 0x6081 0` / `ethercat download -p 0 0x6081 0 10000`.
For values every start should apply, use the profile's `parameters:` instead.

**Turn the scripts into a ROS service/action**: wrap the PP handshake in an action server (`rclpy.action`)
with a "goal position" and feedback; reuse `MdxClient`.

**Multiple drives**: see section 11.

**Standard ROS integration**: switch to `<joint>` + the CiA402 plugin in CSP mode and use
`joint_trajectory_controller` / MoveIt, with units in radians (see `ref/drive_canopen.yaml`).

## 8. Troubleshooting

| Symptom | Check |
|---|---|
| `no valid state received` | launch not running / not sourced; `ros2 control list_controllers` |
| Launch waits for `robot_description` | `robot_state_publisher` died (generated URDF error, see the launch log) |
| `ethercat slaves` shows no slave | cable, master service, `/dev/EtherCAT0` |
| Slave stays PREOP with flag `E` | cycle too fast (1000 Hz failed here), wrong PDO map; try a lower `cycle_frequency` |
| Slave never reaches OP | wrong `vendor_id`/`product_id`, wrong PDO types/size, DC settings |
| Status word shows Fault, error `0x7500` | send `0x80` fault reset (done in `pp_test`); if it returns, fix the cause |
| Control word accepted but no PP motion | not in state `0x27`; `mode_display` != 1; set-point bit not toggled |
| PP velocity is not steady | PP is a finite move with acceleration/deceleration; use `pv_test` to check a constant velocity command |
| "Could not enable FIFO RT scheduling" | harmless warning; to fix, grant realtime rights to the user |

Useful commands:

```bash
ethercat slaves -v                      # bus state, PDO info
ethercat pdos                           # PDOs the drive supports
ros2 control list_hardware_interfaces   # all command/state interfaces
ros2 topic echo /mdx_gpio/gpio_states   # live values
```

## 9. After you edit a file: what to do

Run from `~/glove_box_dev`. Our package is `ament_python`, so **yaml/launch files are copied to
`install/` at build time**. ROS reads the installed copy, not the file in `src/`.
Editing `src/` without rebuilding changes nothing.

| You edited | Action |
|---|---|
| `mdx_ethercat/*.py` (read_test, pp_test, mdx_client) | `colcon build --packages-select mdx_ethercat --symlink-install`, then rerun the script. No relaunch needed. |
| `config/mdx_params.yaml`, `config/drives/*.yaml`, `config/profiles/*.yaml` | rebuild, then **stop and relaunch** `mdx.launch.py` (the files are read at launch; scripts re-read them each run) |
| `mdx_ethercat/system.py` | rebuild, relaunch |
| `launch/mdx.launch.py` | rebuild, relaunch |
| `setup.py` (new file or new script entry) | rebuild, then `source install/setup.bash` again in terminals that need the new command |
| `package.xml` (dependencies) | rebuild |
| a new file you created | add it to `data_files` in `setup.py` first, then rebuild and relaunch |
| `src/ethercat_ros2_driver_mod-main/**` (C++ driver) | `colcon build --packages-select <that package>` (or `--packages-up-to mdx_ethercat`), relaunch |

Commands:

```bash
cd ~/glove_box_dev
colcon build --packages-select mdx_ethercat      # rebuild only our package
source install/setup.bash                         # needed in each terminal after the first build
                                                  # or when entry points / new packages appear
pkill -f ros2_control_node                        # stop a stale launch before relaunching
ros2 launch mdx_ethercat mdx.launch.py
```

Notes:
- Faster loop while developing: `colcon build --packages-select mdx_ethercat --symlink-install`
  links `install/` to `src/`, so edits to yaml/launch/py apply **without rebuilding**
  (you still relaunch for config changes). Entry points in `setup.py` still need a rebuild.
- Check that the installed copy is current: `diff config/mdx_params.yaml ~/glove_box_dev/install/mdx_ethercat/share/mdx_ethercat/config/mdx_params.yaml`.
- Always stop the old launch before starting a new one; two masters cannot own `/dev/EtherCAT0`
  and a stale `ros2_control_node` keeps the old config.
- Changes outside the workspace are separate: the IgH master settings (`/etc/ethercat.conf`,
  NIC MAC) need `sudo systemctl restart ethercat`; the drive's own parameters (SDO writes) live in
  the drive and may need a power cycle or save command (see the manual).
- After a C++ change, a failed build leaves the old binary in `install/`; read the colcon error
  output before assuming the change took effect.

## 10. Workspace map

```
~/glove_box_dev/                  workspace root; run colcon from here
  build/  install/  log/      generated by colcon (never edit)
  src/                        all source code (edit here)
  ref/                        reference documents
  guide.md                    notes
```

### Generated folders (do not edit, safe to delete and rebuild)

| Folder | Content |
|---|---|
| `build/` | per-package compiler work files (CMake cache, objects, test binaries). Needed for fast incremental builds. |
| `install/` | what ROS actually runs: libraries, executables, and copies of yaml/launch. `setup.bash` here adds them to your environment. |
| `log/` | one `build_<date>` folder per `colcon build`, with compiler output; `latest_build` points to the newest. Look here when a build fails. |

If something behaves strangely after many edits: `rm -rf build install log && colcon build`.

### `ref/` (reference only, not used at runtime)

| File | Use |
|---|---|
| `AMA Servo EtherCAT v3.4.22.xml` | ESI file: the drive's vendor/product id, supported PDOs, object dictionary |
| `MDX-EtherCAT-UserManual-CN20250224-A1-L.pdf` | drive manual: modes, error codes, homing, parameters |
| `drive_canopen.yaml` | example CiA402 config; source of the 10000 counts/rev scaling |

### `src/mdx_ethercat/` (our package, ament_python)

| File | Role |
|---|---|
| `mdx_ethercat/mdx_client.py` | ROS node wrapper for one motor: `state` dict in, `command` dict out, `deg_to_counts`, `spin_until`, `enable`, `drive_state` |
| `mdx_ethercat/system.py` | loads the yaml config and generates the URDF, controller yaml and slave yamls |
| `mdx_ethercat/read_test.py` | reads all states, decodes status, prints PASS/FAIL checks |
| `mdx_ethercat/pp_test.py` | enable drive, PP move by `tests.pp_test.move_degrees`, monitor, disable |
| `mdx_ethercat/pv_test.py` | enable drive, command bounded PV speed, decelerate to zero, disable |
| `mdx_ethercat/csp_test.py` | enable drive, CSP move through `tests.csp_test.segments`, monitor, disable |
| `mdx_ethercat/util/ec_tool.py` | bus tool: `scan`, `set-alias`, `set-zero`, `read-params` (current SDO values vs defaults) |
| `mdx_ethercat/util/plot_csv.py` | Run directly as `./src/mdx_ethercat/mdx_ethercat/util/plot_csv.py [file.csv] [-c cols]`, or via `ros2 run mdx_ethercat plot_csv`; interactively plots a recorded CSV (newest CSV under `log/` by default) |
| `config/mdx_params.yaml` | system settings and the list of motors |
| `config/drives/<type>.yaml` | one drive model: identity, PDO map, settable startup SDOs |
| `config/profiles/<motor>.yaml` | one motor: bus address, mechanics, calibration, motion, parameters, tests |
| `launch/mdx.launch.py` | generates the config, starts robot_state_publisher, ros2_control_node, controller spawner |
| `setup.py` | install rules (data files) and the `ros2 run` commands |
| `package.xml` | package name, dependencies |
| `setup.cfg`, `resource/mdx_ethercat` | boilerplate required by ament_python; leave as is |
| `README.md` | this document |

### `src/ethercat_ros2_driver_mod-main/` (third-party EtherCAT stack, change only when needed)

Layers from bottom to top:

| Package | Role |
|---|---|
| `ethercat_manager` | low-level wrapper around the IgH master library: request master, configure slaves, domains, DC, run the cyclic send/receive |
| `ethercat_interface` | the C++ base classes for slave plugins (`EcSlave`) and PDO helpers; the contract between driver and slave plugins |
| `ethercat_driver` | the ros2_control hardware plugin `ethercat_driver/EthercatDriver` (`src/ethercat_driver.cpp`): loads the slave plugins, runs the cycle in `read()` / `write()`, maps PDO data to ros2_control interfaces. `ethercat_driver_plugin.xml` registers it with pluginlib; `examples/` and `test/` show usage |
| `ethercat_generic_plugins/ethercat_generic_slave` | `GenericEcSlave`: configures any slave from a yaml PDO map (what we use) |
| `ethercat_generic_plugins/ethercat_generic_cia402_drive` | `EcCiA402Drive`: ready-made CiA402 drive with joint interfaces, CSP/CSV/CST only (no PP), hence we use the generic slave |
| `ethercat_msgs` | service definitions (e.g. for SDO access) |
| `ethercat_driver_ros2` | meta-package plus docs (`doxygen/`, `sphinx/`) |
| `README.md`, `INSTALL.md` | upstream usage and IgH installation; `CONTRIBUTING.md`, `LICENSE` upstream project files |

Data path in one cycle (every 10 ms), inside `EthercatDriver`:
`read()`: receive frame from the master -> `GenericEcSlave` copies TxPDO bytes into state interfaces.
`write()`: command interfaces -> RxPDO bytes -> send frame to the master.

## 11. Several motors, several profiles

### 11.1 How a motor is identified on the bus

| Identity | Where it comes from | What it tells you |
|---|---|---|
| vendor id + product code (+ revision) | the drive's SII EEPROM, set by the manufacturer | the drive MODEL; same for every drive of that model |
| serial number | SII EEPROM | unique, but many drives leave it 0 (the MDX+_EC does) |
| **ring position** | order on the cable, counted from the master (0, 1, 2...) | changes if you re-wire |
| **alias address** | a 16 bit number written into the SII EEPROM by you | stays with the physical drive wherever it is wired |

The driver addresses a slave with `alias` + `position`: `alias: 0` means "count `position` from the bus start";
`alias: N` means "the slave with alias N, then `position` slaves further" (normally `position: 0`).

### 11.2 Do I connect one motor at a time to set its id?

No. All drives can stay on the bus; `ec_tool set-alias` selects the slave by its ring position:

```bash
ros2 run mdx_ethercat ec_tool scan                 # list every slave, its model and its matching profile
ros2 run mdx_ethercat ec_tool set-alias 0 101      # write alias 101 to the slave at ring position 0
ros2 run mdx_ethercat ec_tool set-alias 1 102
ros2 run mdx_ethercat ec_tool scan                 # alias column now shows 101 / 102
```

Stop the launch first. If scan still shows the old alias, power-cycle the drives.
Connecting one at a time is only useful to know which PHYSICAL motor is which: connect (or power) one,
`scan`, set its alias, label the motor, then add the next. With the alias in each profile
(`bus: {alias: 101, position: 0}`), re-wiring or inserting a drive no longer breaks the mapping.
Mixing models is fine: each profile names its own `drive:` file, the identity check
(vendor/product) is done by the master when the slave is configured.

### 11.3 Add a motor

1. Same model: copy `config/profiles/motor1.yaml` to `motor2.yaml`, change `bus`, `mechanics`, `calibration`.
   New model: also create `config/drives/<model>.yaml` (identity from the ESI, PDO map from the manual).
2. Add `motor2` under `motors:` in `config/mdx_params.yaml`.
3. Rebuild (`colcon build --packages-select mdx_ethercat --symlink-install`), relaunch.
4. Use it: `ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor2`.

All motors share one master and one cycle; the controller `mdx_gpio` has one interface group per motor
(`/mdx_gpio/commands` and `/mdx_gpio/gpio_states` carry every group, the scripts use the one they were told to).

### 11.4 Calibration

Each profile stores what is specific to the physical motor:

| Key | Meaning | How to set |
|---|---|---|
| `mechanics.pulses_per_rev`, `gear_ratio` | counts per output revolution | from the manual / mechanics |
| `calibration.direction` | `1` or `-1`; flips the sign of every move | test with a small move |
| `calibration.zero_offset` | drive count of the output zero | move the axis to its zero, then `ec_tool set-zero motor1` (launch running) |
| `motion.max_speed` | counts/s of the motor's real maximum | manual |
| `parameters` | drive-side limits (software limits, following error window, max torque) | edit and relaunch |

`set-zero` rewrites the `zero_offset:` line of the profile file in place. With a normal build that is the copy in
`install/` (lost on rebuild); build with `--symlink-install` so it edits the file in `src/`.

How other robot stacks manage it (see the reference workspace): one yaml per drive model, one entry per joint with
its bus position, and per-arm folders of copies for individual motors. This package keeps the same idea
(drive model file + per-motor entry) but with a profile file per motor, so calibration lives next to the motor it belongs to.
