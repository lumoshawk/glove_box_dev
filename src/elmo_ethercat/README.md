# elmo_ethercat

Elmo Gold EtherCAT (GCON) servo drive, a standard CoE / CiA402 slave, driven through the IgH master and
`ethercat_generic_plugins/GenericEcSlave`. Addressed by ring position 5 (alias 54 is not possible, see section 2). Structure follows `mdx_ethercat`.

```
config/elmo_params.yaml         rates, master id, which motors to load
config/drives/elmo_gold.yaml    identity + PDO map (from ref/Elmo ECAT 00010420 V11.xml)
config/profiles/elmo_m1.yaml    the drive on the bus (ring position 5), mechanics, startup SDOs, test parameters
launch/elmo.launch.py           standalone launch (500 Hz, /elmo_gpio)
elmo_ethercat/                  system.py (config -> ros2_control), elmo_client.py, read_test.py, enable_test.py, pp_test.py, pv_test.py
test/                           pytest: drive file vs ESI, PDO sizes, config generation, CiA402 helpers
```

## 1. Identity and ESI

`ethercat slaves` shows the drive as `0x0000009a:0x00030924` without a name: its SII has no strings, so IgH can only
print vendor:product. It is recognized (vendor `0x9a` Elmo, product `0x00030924`, revision `0x00010420`,
mailbox CoE + EoE + FoE) and is the ESI device `Gold EtherCAT GCON Drive Rev:0x00010420` in
`ref/Elmo ECAT 00010420 V11.xml`. Supported modes (`0x6502` = `0x3ED`): PP, PV, TQ, HM, IP, CSP, CSV, CST.

PDOs: the ESI's fixed PDOs `0x1600..0x1606` / `0x1A00..0x1A04` exclude each other and split mode of operation from
velocity. `elmo_gold.yaml` therefore uses the two freely mappable PDOs `0x1607` (RxPDO, 27 bytes) and `0x1A07`
(TxPDO, 23 bytes; 8 entries max, SM size 32 bytes): control word, mode, target position / velocity, profile
velocity / accel / decel, velocity offset out; error code, status word, mode display, position, velocity, current,
following error, digital inputs in. DC sync `0x0300`; startup SDO `0x60C2:1 = 2` (2 ms, matches 500 Hz).

## 2. Alias 54 is not possible; the drive is addressed by ring position

The drive's SII (EEPROM) is emulated by the firmware and is read-only (Elmo EtherCAT Application Manual §2.6:
"Write Access is not supported"). IgH reads the alias from the SII, so `54` can never be stored:

- `ethercat alias -p 5 54` returns 0 but only changes the master's in-memory table. `ethercat slaves` shows `54:0`
  until the next rescan, master restart or power cycle, then it is `52:3` again. This is what made the first
  launch work and a later one fail with all states NaN.
- The vendor objects `0x20E0` ("Ecat alias object", set to 54 and stored with `0x1010`) and `0x10E0:1` only
  set the ESC register `0x0012`, which IgH does not use for addressing. They are harmless and can stay.

So `elmo_m1` uses `bus: {alias: 0, position: 5}` (absolute ring position; `ring: 5` in `bringup.yaml`). Check
`ethercat slaves` after any re-wiring: the drive must still be at ring position 5 (`52:3` is the same slave, counted
from the D1616BP's alias 52). `ref/alias_def.md` still lists 54 as the intended label.

## 3. EoE interface must not be managed by NetworkManager

The master creates a network interface `eoe0s5` for the drive's EoE mailbox, and re-creates it on every master start
and rescan. If NetworkManager brings it up and runs DHCP on it, the EoE traffic fills the drive's mailbox: SDO writes
time out (`Reception of CoE download response failed: No response`, PDO mapping fails, slave stuck in
`SAFEOP + ERROR`).

One-time fix, installed on this PC (`config/networkmanager/99-ethercat-eoe.conf`, makes NetworkManager ignore `eoe*`):

```bash
sudo cp src/elmo_ethercat/config/networkmanager/99-ethercat-eoe.conf /etc/NetworkManager/conf.d/
sudo systemctl reload NetworkManager
```

On another PC, repeat this. Check: `nmcli -t -f DEVICE,STATE device | grep eoe` must say `unmanaged`.

## 4. Run and test

```bash
ros2 launch elmo_ethercat elmo.launch.py            # stop any other launch first (one master user)
ros2 run elmo_ethercat read_test                    # prints all states, checks: readable, no fault, error code 0
ros2 run elmo_ethercat enable_test                  # enable (PP, target = measured position), hold hold_s, disable
ros2 run elmo_ethercat pp_test                      # MOVES: PP +100000 counts from the current position, then back
ros2 run elmo_ethercat pv_test                      # MOVES: PV forward then back, see below
cd src/elmo_ethercat && python3 -m pytest test -q   # offline tests
```

All tests exit 0 on pass. `enable_test` commands no motion; it aborts the hold when the position drifts more than
`tests.enable_test.max_drift_counts` or the drive leaves Operation enabled, and always switches the drive off again.
Reaching OP takes several seconds (`Slave did not sync after 5000 ms` is logged once, harmless).

`pp_test` moves the axis (clear it first): Profile Position (mode 1), target = start + `move_counts` (100000), then back to
the start, at `velocity` (10000 counts/s) with `motion.accel` / `motion.decel`. Each move uses the new set-point handshake
(bit 4 up, acknowledge in bit 12, bit 4 down, target reached in bit 10). Checks: acknowledged, reached in time
(distance / velocity plus ramps plus 5 s), within 500 counts of the target, and net displacement within 500 counts. On a
timeout or fault it halts the axis, and it always disables the drive. Parameters in `tests.pp_test` of the profile.

`pv_test` moves the axis (clear it first): Profile Velocity (mode 3) at `speed_percent` of
`motion.max_speed` for `duration_s` forward, then the same backward, `cycles` times, ramping with
`motion.accel` / `motion.decel`. The speed is measured from the position change over 0.5 s, not from `0x606C`, which is
noisy on this drive. Checks per leg: no fault / overspeed (> 1.5 x target aborts), speed within 20 % of the target, travel
at least half of speed x duration; then it stops at zero and must end within 20 % of one leg of the start (the reversal
ramp makes the way back a little shorter, about 1500 counts at 5000 counts/s). Parameters in `tests.pv_test` of the profile.
`motion.max_speed` (10000 counts/s) is a placeholder until the real motor speed and encoder resolution are known.

`mechanics.pulses_per_rev` in the profile is a placeholder: the drive's `0x608F` / `0x6091` are at their default 1, so
set the real encoder resolution before using degrees. The drive is in `glove_box_bringup` (`bringup.yaml`, `ring: 5`).
