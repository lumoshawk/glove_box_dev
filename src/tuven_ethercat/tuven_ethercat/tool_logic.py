"""The tool changer sequences, written against a small IO object so they run on the robot and in unit tests.

IO object (see IoClient in io_client.py for the real one):
  io.lock(actuator) / io.unlock(actuator)   write the actuator's lock / unlock level
  io.sensor_stable(sensor)                  True while the proximity sensor reads its stable level (high voltage)
  io.now()                                  seconds, monotonic
  io.step(seconds)                          let time pass (the real one keeps spinning ROS)
"""
from dataclasses import dataclass

POLL_S = 0.02


@dataclass
class Result:
    ok: bool
    message: str
    stable_for_s: float = 0.0
    elapsed_s: float = 0.0


def run_sequence(io, actuator, sensor, delay_s, stable_s, timeout_s, log=print):
    """lock -> delay -> unlock -> wait until the sensor was stable for stable_s without a break -> lock.

    The first lock assumes the changer is in its rest state. The actuator is locked again on every way out
    (success, timeout, Ctrl-C or any error): a tool must never be left unlocked by a failed run.
    """
    start = io.now()
    try:
        log(f'lock {actuator}')
        io.lock(actuator)
        log(f'delay {delay_s:g} s')
        io.step(delay_s)
        log(f'unlock {actuator}: attach the tool, {sensor} must be stable for {stable_s:g} s')
        io.unlock(actuator)

        deadline = io.now() + timeout_s
        stable_since = None
        while True:
            now = io.now()
            if io.sensor_stable(sensor):
                if stable_since is None:
                    stable_since = now
                    log(f'{sensor} stable, counting')
                if now - stable_since >= stable_s:
                    return Result(True, f'{sensor} stable for {stable_s:g} s', now - stable_since, now - start)
            elif stable_since is not None:
                log(f'{sensor} lost, restart count')
                stable_since = None
            if now >= deadline:
                return Result(False, f'{sensor} not stable for {stable_s:g} s within {timeout_s:g} s',
                              0.0 if stable_since is None else now - stable_since, now - start)
            io.step(POLL_S)
    finally:
        log(f'lock {actuator}')
        io.lock(actuator)


def get_tool(io, actuator, sensor, delay_s, stable_s, timeout_s, log=print):
    """The slot is empty: lock, delay, unlock to receive the tool, sensor stable (default 3 s), lock."""
    return run_sequence(io, actuator, sensor, delay_s, stable_s, timeout_s, log)


def switch_tool(io, actuator, sensor, delay_s, stable_s, timeout_s, log=print):
    """A tool is attached: lock, delay, unlock to swap it, sensor stable (default 5 s), lock."""
    return run_sequence(io, actuator, sensor, delay_s, stable_s, timeout_s, log)
