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
ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor2    # pick a motor profile
ros2 run mdx_ethercat pp_test --ros-args -p motor:=motor1 -p record:=true
ros2 run mdx_ethercat csp_test --ros-args -p record:=true    # record states to log/records/*.csv
pkill -f mdx.launch.py; pkill -f ros2_control_node           # stop a stale launch
pgrep -af "ros2_control_node|robot_state_publisher"          # must be empty before a new launch (stale ones break it)
```

## Plot a recording

```bash
ros2 run mdx_ethercat plot_csv                               # newest CSV under log/ (pp or csp)
ros2 run mdx_ethercat plot_csv -t pp                         # newest pp_test recording
ros2 run mdx_ethercat plot_csv -t csp                        # newest csp_test recording
ros2 run mdx_ethercat plot_csv log/records/<file>.csv        # a specific file
ros2 run mdx_ethercat plot_csv -c position velocity current  # choose columns
```

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
```

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

## Config files to edit (in src/mdx_ethercat/config)

| File | Edit for |
|---|---|
| `mdx_params.yaml` | cycle frequency, master id, list of motors |
| `drives/<type>.yaml` | drive model: identity, PDO map |
| `profiles/<motor>.yaml` | bus address, gearing, calibration, limits, test values |

## CiA402 quick reference

| Control word | Meaning |
|---|---|
| `0x06` | shutdown |
| `0x07` | switch on |
| `0x0F` | enable operation |
| `0x1F` | enable + new set-point (PP) |
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
