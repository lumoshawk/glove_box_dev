"""Read/write stress test: PDO read rate and jitter, write->read round trip, optional SDO load.

The drive is held in shutdown (never Operation enabled), so nothing moves. Phases:
  read       listen to /mdx_gpio/gpio_states, log arrival and header-stamp jitter, missed cycles
  roundtrip  toggle the mode of operation (0x6060) and time how long until mode_display (0x6061)
             reports it: a real command -> PDO -> drive -> PDO -> state round trip
  sdo        (sdo:=true) `ethercat upload` loop in a thread while the PDO stream is watched, since
             acyclic mailbox traffic is a known source of cyclic jitter; sdo_write:=true also writes
             0x607F back with its own value and reads it again to verify

Usage: ros2 run mdx_ethercat stress_test --ros-args -p motor:=motor1 [-p duration_s:=60.0]
         [-p roundtrips:=500] [-p sdo:=true] [-p sdo_count:=200] [-p sdo_write:=true]
Results go to log/stress/<motor>_<time>/ (summary.txt, summary.json, samples.csv, roundtrip.csv, sdo.csv).
"""
import csv
import json
import math
import os
import statistics
import subprocess
import threading
import time

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues

from mdx_ethercat import system
from mdx_ethercat.mdx_client import CW_SHUTDOWN, SW_FAULT, MdxClient, drive_state
from mdx_ethercat.util.ec_tool import read_slaves

LOG_DIR = os.path.join('log', 'stress')  # relative to the current directory
MODES = (1, 3, 8)  # PP, PV, CSP; changed only while the drive is not enabled
ROUNDTRIP_TIMEOUT_S = 0.5
STALL_CYCLES = 5  # an interval longer than this many cycles fails the test
LATE_CYCLES = 1.5  # an interval longer than this many cycles counts as late
RATE_TOLERANCE = 0.01  # accepted deviation of the measured state rate from cycle_frequency
SDO_READS = ((0x6041, 0, 'uint16'), (0x6064, 0, 'int32'), (0x603F, 0, 'uint16'))
SDO_WRITE = (0x607F, 0, 'uint32')  # max_profile_velocity, written back with the value read


def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values) - 1, round(q / 100 * (len(sorted_values) - 1)))]


def describe(values):
    """n, mean, std, min, p50, p99, p99.9, max of a list of numbers."""
    if not values:
        return {'n': 0}
    v = sorted(values)
    return {'n': len(v), 'mean': statistics.fmean(v), 'std': statistics.pstdev(v), 'min': v[0],
            'p50': percentile(v, 50), 'p99': percentile(v, 99), 'p99.9': percentile(v, 99.9),
            'max': v[-1]}


def fmt(stats, unit='ms'):
    if not stats['n']:
        return 'no data'
    return (f'n={stats["n"]} mean={stats["mean"]:.3f} std={stats["std"]:.3f} min={stats["min"]:.3f} '
            f'p50={stats["p50"]:.3f} p99={stats["p99"]:.3f} p99.9={stats["p99.9"]:.3f} '
            f'max={stats["max"]:.3f} {unit}')


def ec(*args):
    return subprocess.run(['ethercat', *args], capture_output=True, text=True, check=True,
                          timeout=5).stdout.split()[-1]


def sdo_worker(ring, count, write, rows):
    """Append (op, object, latency_ms, ok) rows; runs in a thread so the main thread keeps spinning."""
    def timed(op, index, call):
        t0 = time.perf_counter()
        try:
            value, ok = call(), True
        except (subprocess.SubprocessError, OSError):
            value, ok = None, False
        rows.append((op, f'{index:#06x}', (time.perf_counter() - t0) * 1e3, ok))
        return value

    for _ in range(count):
        for index, sub, type_ in SDO_READS:
            timed('read', index, lambda: ec('upload', '-p', str(ring), hex(index), str(sub), '--type', type_))
        if write:
            index, sub, type_ = SDO_WRITE
            value = timed('read', index, lambda: ec('upload', '-p', str(ring), hex(index), str(sub), '--type', type_))
            if value is None:
                continue
            timed('write', index, lambda: ec('download', '-p', str(ring), hex(index), str(sub), '--type', type_, value))
            back = timed('read', index, lambda: ec('upload', '-p', str(ring), hex(index), str(sub), '--type', type_))
            rows.append(('verify', f'{index:#06x}', 0.0, back == value))


class Sampler:
    """One row per /mdx_gpio/gpio_states message of this motor, while a phase is collecting."""

    def __init__(self, node):
        self.node = node
        self.rows = None
        node.create_subscription(DynamicInterfaceGroupValues, '/mdx_gpio/gpio_states', self._on_msg, 2000)

    def _on_msg(self, msg):
        if self.rows is None:
            return
        now = time.monotonic_ns()
        for name, group in zip(msg.interface_groups, msg.interface_values):
            if name == self.node.motor:
                v = dict(zip(group.interface_names, group.values))
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                self.rows.append((now, stamp, v['status_word'], v['error_code'], v['position'],
                                  v['mode_display']))

    def collect(self, until, timeout):
        self.rows = []
        self.node.spin_until(until, timeout)
        rows, self.rows = self.rows, None
        return rows


def analyze_stream(rows, period_ms):
    """Rate, interval jitter, missed cycles and drive-state sanity of a list of Sampler rows."""
    out = {'samples': len(rows)}
    if len(rows) < 3:
        return out
    arrival = [(b[0] - a[0]) / 1e6 for a, b in zip(rows, rows[1:])]
    has_stamp = rows[-1][1] > rows[0][1]
    stamped = [(b[1] - a[1]) / 1e6 for a, b in zip(rows, rows[1:])] if has_stamp else None
    intervals = stamped or arrival  # the header stamp is set by the controller, not by python
    span_s = (rows[-1][1] - rows[0][1] if has_stamp else rows[-1][0] - rows[0][0]) / 1e9
    out.update({
        'interval_source': 'header stamp' if has_stamp else 'arrival time',
        'rate_hz': (len(rows) - 1) / span_s,
        'arrival_interval_ms': describe(arrival),
        'stamp_interval_ms': describe(stamped) if has_stamp else None,
        'late': sum(d > LATE_CYCLES * period_ms for d in intervals),
        'stalls': sum(d > STALL_CYCLES * period_ms for d in intervals),
        'missed_cycles': sum(max(0, round(d / period_ms) - 1) for d in intervals),
        'status_changes': sum(a[2] != b[2] for a, b in zip(rows, rows[1:])),
        'fault_samples': sum(int(r[2]) & SW_FAULT != 0 for r in rows),
        'error_code_samples': sum(r[3] != 0 for r in rows),
        'nan_samples': sum(any(math.isnan(x) for x in r[2:]) for r in rows),
        'position_span_counts': max(r[4] for r in rows) - min(r[4] for r in rows),
    })
    return out


def roundtrip(node, count):
    """Command a different mode of operation each time and time until mode_display shows it.

    Includes up to one client publish period (command timer) on top of the EtherCAT round trip.
    Returns rows of (index, mode, latency_ms, ok).
    """
    rows = []
    for i in range(count):
        current = node.state.get('mode_display')
        mode = MODES[(MODES.index(current) + 1) % len(MODES)] if current in MODES else MODES[0]
        t0 = time.perf_counter()
        node.command['mode'] = mode
        ok = node.spin_until(lambda: node.state.get('mode_display') == mode, ROUNDTRIP_TIMEOUT_S)
        rows.append((i, mode, (time.perf_counter() - t0) * 1e3, ok))
    return rows


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def stream_lines(label, s):
    if s['samples'] < 3:
        return [f'{label}: only {s["samples"]} samples']
    lines = [f'{label}: {s["samples"]} samples, {s["rate_hz"]:.2f} Hz (interval from {s["interval_source"]})',
             f'  arrival interval  {fmt(s["arrival_interval_ms"])}']
    if s['stamp_interval_ms']:
        lines.append(f'  stamp interval    {fmt(s["stamp_interval_ms"])}')
    lines.append(f'  late={s["late"]} (>{LATE_CYCLES} cycles)  stalls={s["stalls"]} (>{STALL_CYCLES} cycles)  '
                 f'missed cycles={s["missed_cycles"]}')
    lines.append(f'  fault samples={s["fault_samples"]}  error_code!=0 samples={s["error_code_samples"]}  '
                 f'NaN samples={s["nan_samples"]}  status word changes={s["status_changes"]}  '
                 f'position span={s["position_span_counts"]:.0f} counts')
    return lines


def main():
    rclpy.init()
    node = MdxClient('mdx_stress_test')
    duration = node.declare_parameter('duration_s', 60.0).value
    count = node.declare_parameter('roundtrips', 500).value
    use_sdo = node.declare_parameter('sdo', False).value
    sdo_count = node.declare_parameter('sdo_count', 200).value
    sdo_write = node.declare_parameter('sdo_write', False).value
    params = system.load_params()
    period_ms = 1000 / params['cycle_frequency']
    cmd = node.command
    result = {'motor': node.motor, 'cycle_frequency_hz': params['cycle_frequency'],
              'client_rate_hz': params['client_rate_hz'], 'started': time.strftime('%Y-%m-%d %H:%M:%S')}
    checks = {}
    try:
        if not node.spin_until(node.has_state, 10):
            raise SystemExit('no valid state received for this motor on /mdx_gpio/gpio_states')
        # never run with the drive enabled: only mode of operation is toggled
        cmd['control_word'] = CW_SHUTDOWN
        if not node.spin_until(lambda: node.status_word() & 0x6F != 0x27, 5):
            raise SystemExit(f'drive stays Operation enabled (status 0x{node.status_word():04X})')
        if node.status_word() & SW_FAULT:
            raise SystemExit(f'drive is in fault (0x{node.status_word():04X}); clear it first')
        print(f'{node.motor}: {drive_state(node.status_word())}, cycle {period_ms:.2f} ms')

        out_dir = os.path.join(LOG_DIR, f'{node.motor}_{time.strftime("%Y-%m-%d_%H-%M-%S")}')
        os.makedirs(out_dir, exist_ok=True)
        sampler = Sampler(node)
        sample_header = ['monotonic_ns', 'stamp_ns', 'status_word', 'error_code', 'position', 'mode_display']

        print(f'Phase 1/3: PDO read for {duration:.0f} s')
        rows = sampler.collect(lambda: False, duration)
        write_csv(os.path.join(out_dir, 'samples.csv'), sample_header, rows)
        result['read'] = analyze_stream(rows, period_ms)
        read = result['read']
        checks['state stream received'] = read['samples'] >= 3
        if checks['state stream received']:
            checks[f'state rate within {RATE_TOLERANCE:.0%} of {params["cycle_frequency"]} Hz'] = (
                abs(read['rate_hz'] / params['cycle_frequency'] - 1) <= RATE_TOLERANCE)
            checks['no stalled intervals'] = read['stalls'] == 0
            checks['no fault / error code / NaN'] = not (
                read['fault_samples'] or read['error_code_samples'] or read['nan_samples'])

        print(f'Phase 2/3: {count} write -> read round trips (mode of operation)')
        rt = roundtrip(node, count)
        write_csv(os.path.join(out_dir, 'roundtrip.csv'), ['index', 'mode', 'latency_ms', 'ok'], rt)
        good = [r[2] for r in rt if r[3]]
        result['roundtrip_ms'] = describe(good)
        result['roundtrip_timeouts'] = sum(not r[3] for r in rt)
        checks['no round trip timeouts'] = result['roundtrip_timeouts'] == 0

        if use_sdo:
            print(f'Phase 3/3: {sdo_count} SDO loops{" with write-back" if sdo_write else ""} under PDO stream')
            bus = node.profile['bus']
            ring = next((s['ring'] for s in read_slaves()
                         if (s['alias'], s['rel']) == (bus['alias'], bus['position'])), None)
            if ring is None:
                raise SystemExit(f'{node.motor}: no slave at alias {bus["alias"]} position {bus["position"]}')
            sdo_rows = []
            thread = threading.Thread(target=sdo_worker, args=(ring, sdo_count, sdo_write, sdo_rows))
            thread.start()
            rows = sampler.collect(lambda: not thread.is_alive(), 3600)
            thread.join()
            write_csv(os.path.join(out_dir, 'sdo.csv'), ['op', 'object', 'latency_ms', 'ok'], sdo_rows)
            result['pdo_during_sdo'] = analyze_stream(rows, period_ms)
            result['sdo_ms'] = {op: describe([r[2] for r in sdo_rows if r[0] == op and r[3]])
                                for op in ('read', 'write')}
            result['sdo_failures'] = sum(not r[3] for r in sdo_rows)
            checks['no SDO failures / write-back mismatch'] = result['sdo_failures'] == 0
            if result['pdo_during_sdo']['samples'] >= 3:
                checks['no stalled intervals during SDO'] = result['pdo_during_sdo']['stalls'] == 0
        else:
            print('Phase 3/3: skipped (sdo:=true to enable)')
    finally:
        cmd['control_word'] = CW_SHUTDOWN
        node.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()

    result['checks'] = checks
    lines = [f'stress_test {node.motor}  {result["started"]}  cycle={period_ms:.2f} ms',
             f'duration={duration:.0f} s  roundtrips={count}  sdo={use_sdo}  sdo_write={sdo_write}', '']
    lines += stream_lines('PDO read', result['read'])
    lines += ['', f'Round trip (mode write -> mode_display): {fmt(result["roundtrip_ms"])}',
              f'  timeouts={result["roundtrip_timeouts"]} (>{ROUNDTRIP_TIMEOUT_S * 1e3:.0f} ms)']
    if use_sdo:
        lines += ['', *stream_lines('PDO read during SDO load', result['pdo_during_sdo'])]
        for op, stats in result['sdo_ms'].items():
            lines.append(f'SDO {op}: {fmt(stats)}')
        lines.append(f'  failures={result["sdo_failures"]}')
    lines.append('')
    lines += [f'[{"PASS" if ok else "FAIL"}] {name}' for name, ok in checks.items()]
    report = '\n'.join(lines)
    print('\n' + report)
    with open(os.path.join(out_dir, 'summary.txt'), 'w') as f:
        f.write(report + '\n')
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\nLogs in {out_dir}')
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
