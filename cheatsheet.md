# Command cheatsheet

## Setup (every new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/glove_box_dev/install/setup.bash
```

## Build

```bash
cd ~/glove_box_dev
colcon build --packages-select mdx_ethercat                  # our package only
colcon build --packages-select mdx_ethercat --symlink-install  # yaml/py/launch edits apply without rebuild
colcon build --packages-select mdx_ethercat eyou_ethercat glove_box_bringup --symlink-install  # all three
colcon build                                                 # everything (driver + package)
rm -rf build install log && colcon build                     # clean rebuild
source install/setup.bash                                    # after a build, in each terminal
```

Rebuild needed: C++ changes, new files, `setup.py`, `package.xml`. Relaunch needed after any config yaml change.

## Run

```bash
ros2 launch mdx_ethercat mdx.launch.py                       # terminal 1, leave running
ros2 run mdx_ethercat read_test                              # read all states + checks
ros2 run mdx_ethercat pp_test                                # Profile Position move
ros2 run mdx_ethercat csp_test                               # Cyclic Synchronous Position move
ros2 run mdx_ethercat pv_test                                # Profile Velocity hold (speed/duration from profile tests.pv_test)
ros2 run mdx_ethercat pv_test --ros-args -p record:=true     # same, record to log/records/pv_test_*.csv
ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor2    # pick a motor profile
ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor1 -p record:=true
ros2 run mdx_ethercat csp_test --ros-args -p record:=true    # record states to log/records/*.csv
pgrep -af "mdx.launch|ros2_control_node"; kill <pid>         # stop a launch by PID (list first)
pkill -f mdx.launch.py; pkill -f ros2_control_node           # stop a stale launch
pgrep -af "ros2_control_node|robot_state_publisher"          # must be empty before a new launch (stale ones break it)
```

## Run: EYOU and all motors

Same commands as above with `eyou_ethercat` (launch file `eyou.launch.py`, motors `eyou1`, `eyou2`,
controller `/eyou_gpio`). Only one launch owns the master: stop any other first.

```bash
ros2 launch eyou_ethercat eyou.launch.py                     # EYOU motors only, 1000 Hz
ros2 run eyou_ethercat read_all                              # table of every EYOU motor
ros2 run eyou_ethercat pp_test --ros-args -p motor:=eyou1

ros2 launch glove_box_bringup all.launch.py                  # ALL motors, one master, 500 Hz
ros2 run glove_box_bringup read_all                          # all motors in one table, PASS/FAIL each
ros2 run glove_box_bringup read_all --ros-args -p raw:=true  # plus every raw state interface
ros2 run glove_box_bringup enable_test                       # enable each motor 1 s, one by one, no motion; PASS/FAIL each
ros2 run glove_box_bringup enable_test --ros-args -p motors:="[mdx_y2, eyou_r1]" -p hold_s:=2.0
ros2 topic echo /eyou_gpio/gpio_states --once                # EYOU states (MDX: /mdx_gpio/gpio_states)
```

Bus layout for the combined launch: `src/glove_box_bringup/config/bringup.yaml` (`package`, `motor`,
`ring` from `ethercat slaves`). EYOU entries there are commented out until the modules are connected.

## Run: TUVEN IO box D1616BP (alias 52, tool changer sensors / locks)

```bash
ros2 launch tuven_ethercat tuven.launch.py                   # the box alone, 100 Hz (stop other launches first)
ros2 run tuven_ethercat read_test                            # sensor1 / sensor2 levels (never writes)
ros2 run tuven_ethercat write_test                           # actuator1: lock, unlock 1 s, lock (MOVES the lock)
ros2 run tuven_ethercat get_tool [--ros-args -p device:=actuator2]      # lock, delay, unlock, sensor stable 3 s, lock
ros2 run tuven_ethercat switch_tool [--ros-args -p device:=actuator2]   # same, sensor stable 5 s
ros2 topic echo /tuven_gpio/gpio_states --once               # sensor1_input, sensor2_input (high = stable)
ros2 run tuven_ethercat gen_drive --check                    # ESI excerpt + drive file in sync with ref/TUVEN_ESI_V7.6.xml
cd src/tuven_ethercat && python3 -m pytest test -q           # package tests
```

Stale `ros2 launch gcan_ethercat` / `robot_state_publisher` processes publish a competing `/robot_description`:
`ros2_control_node` may pick theirs. Stop them (`pgrep -af robot_state_publisher`) or use another `ROS_DOMAIN_ID`.

## Plot a recording

```bash
ros2 run mdx_ethercat plot_csv                               # newest CSV under log/ (pp, csp or pv)
ros2 run mdx_ethercat plot_csv -t pp                         # newest pp_test recording
ros2 run mdx_ethercat plot_csv -t csp                        # newest csp_test recording
ros2 run mdx_ethercat plot_csv -t pv                         # newest pv_test recording
ros2 run mdx_ethercat plot_csv log/records/<file>.csv        # a specific file
ros2 run mdx_ethercat plot_csv -c position velocity current  # choose columns
ros2 run mdx_ethercat plot_csv -t pv -c velocity target_velocity control_word   # PV: measured vs commanded
ros2 run mdx_ethercat plot_csv -t pv --raw                   # drive units (counts, pulses/s) instead of deg and rpm
./src/mdx_ethercat/mdx_ethercat/util/plot_csv.py -t pv       # without ros2 run (from ~/glove_box_dev)
```

Velocity columns plot in rpm and position columns in deg (profile pulses_per_rev, gear_ratio, direction). A black `position_velocity` line (0.25 s window of position change) is added to the velocity panel: the drive's own `velocity` is jittery, this one shows the real speed.

Run from `~/glove_box_dev`: it searches `log/` relative to the current directory.

## Bus tool (stop the launch before set-alias)

```bash
ros2 run mdx_ethercat ec_tool scan                           # slaves, drive model, matching profile
ros2 run mdx_ethercat ec_tool set-alias <ring_pos> <alias>   # 0 removes the alias
ros2 run mdx_ethercat ec_tool set-zero [motor]               # store current position as zero (launch running)
ros2 run mdx_ethercat ec_tool read-params [motor]            # drive's current SDO values vs defaults vs profile
```

## Servo tuning (live, launch running; persistence across relaunch not confirmed, check with read-params)

```bash
ethercat upload -p 0 0x2a53 0 --type uint32                  # stiffness level (default 5)
ethercat download -p 0 0x2a53 0 --type uint32 8              # raise stiffness
ethercat upload -p 0 0x2a55 0 --type uint32                  # position loop gain, 0.1 Hz (default 52)
ethercat upload -p 0 0x2a59 0 --type int32                   # velocity feed-forward, 0.01 % (default 3000)
ethercat download -p 0 0x2a59 0 --type int32 10000           # 100 % feed-forward
ethercat upload -p 0 -t uint32 0x2a5a 0                      # velocity feed-forward filter, 0.1 Hz (default 20000)
ethercat download -p 0 -t uint32 0x2a5a 0 5000               # lower cutoff to reduce overshoot
ethercat upload -p 0 -t uint32 0x2a16 1                      # zero-speed threshold P5-42, pulses/s (default 5000)
ethercat upload -p 0 -t uint32 0x2a33 0                      # P1-03 velocity clamp mode (not P5-51)
```

Profile `parameters:` (motor1.yaml) are written at launch and override manual SDO changes; edit the
profile for persistent values. P5-51 (speed-mode zero clamp) has no SDO: set it in Luna.

## Profile Velocity debugging (drive checks, launch running)

```bash
ethercat upload -p 0 -t int32 0x60ff 0                       # target velocity the drive received
ethercat upload -p 0 -t uint16 0x6040 0                      # controlword the drive received
ethercat upload -p 0 -t uint16 0x6041 0                      # statusword (0x0637 = enabled)
ethercat upload -p 0 -t uint16 0x603f 0                      # error code (0x7500 = EtherCAT comm fault)
ethercat upload -p 0 -t int32 0x6064 0                       # actual position
ethercat download -p 0 -t uint16 0x6040 0 0x80               # fault reset: hold ~0.2 s, then write 0x06
ros2 control list_hardware_interfaces --verbose              # is motor1/target_velocity exported?
ros2 topic echo --once /mdx_gpio/gpio_states                 # state snapshot (velocity, status, error)
```

Hold the controlword Halt bit (`0x10F`) while staging a PV target, then release (`0x0F`) to run;
`pv_test` does this. Velocity/pulse units: 10000 pulses/rev. Check `P0-xx`/`P5-xx` in Luna vs SDO
index in the manual appendix.

## EtherCAT master (IgH)

```bash
ethercat slaves                    # ring pos, alias:rel, state, name
ethercat slaves -v                 # identity (vendor, product, serial), ports, mailbox
ethercat master                    # master phase, link, frame rates
ethercat pdos                      # sync managers, PDO mapping
ethercat sdos                      # object dictionary
ethercat upload -p 0 0x6081 0      # read an SDO  (-p slave position)
ethercat download -p 0 0x6081 0 10000   # write an SDO
ethercat sii_read -p 0 -v          # EEPROM contents
ethercat alias -p 0 101            # write alias
ethercat rescan                    # rescan the bus
ethercat states -p 0 PREOP         # request an AL state (INIT, PREOP, SAFEOP, OP)
sudo systemctl restart ethercat    # restart the master service
ls -l /dev/EtherCAT0               # master device exists?
```

## ros2_control and topics

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list
ros2 topic echo /mdx_gpio/gpio_states --once      # latest states of all motors
ros2 topic echo /mdx_gpio/commands --once
ros2 topic hz /mdx_gpio/gpio_states
ros2 node list
ros2 param list /controller_manager
```

## Inspect the install

```bash
ros2 pkg prefix mdx_ethercat
ros2 pkg executables mdx_ethercat
echo $PYTHONPATH | tr ':' '\n'
python3 -c "import mdx_ethercat; print(mdx_ethercat.__file__)"
diff ~/glove_box_dev/src/mdx_ethercat/config/mdx_params.yaml ~/glove_box_dev/install/mdx_ethercat/share/mdx_ethercat/config/mdx_params.yaml
```

## Logs

```bash
ls ~/glove_box_dev/log/latest_build                   # colcon build logs
ls -t ~/.ros/log | head                           # runtime logs, newest first
ros2 launch mdx_ethercat mdx.launch.py > /tmp/launch.log 2>&1   # capture launch output
```

## Config files to edit (in src/mdx_ethercat/config, same layout in src/eyou_ethercat/config)

| File | Edit for |
|---|---|
| `mdx_params.yaml` / `eyou_params.yaml` | cycle frequency, master id, list of motors |
| `drives/<type>.yaml` | drive model: identity, PDO map |
| `profiles/<motor>.yaml` | bus address, gearing, calibration, limits, test values |
| `src/glove_box_bringup/config/bringup.yaml` | combined launch: which motors, which package, ring position, cycle rate |

## CiA402 quick reference

| Control word | Meaning |
|---|---|
| `0x06` | shutdown |
| `0x07` | switch on |
| `0x0F` | enable operation |
| `0x1F` | enable + new set-point (PP) |
| `0x10F` | enable + Halt (bit 8): PV stage target / stop |
| `0x80` | fault reset |

| Status word `& 0x6F` | State |
|---|---|
| `0x40` | switch on disabled |
| `0x21` | ready to switch on |
| `0x23` | switched on |
| `0x27` | operation enabled |
| bit 3 | fault |
| bit 10 | target reached |
| bit 12 | set-point acknowledge (PP) |

Modes (`0x6060`): `1` PP, `3` PV, `4` torque, `6` homing, `8` CSP, `9` CSV.
