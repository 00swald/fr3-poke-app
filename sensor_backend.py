"""sensor_backend.py -- common sensor interface + real/simulated backends.

move_to_spheres.force_limited_approach() (reused verbatim by this app, see
probe.py and hardware.py) only ever touches a sensor object through:
  - sensor.fz_vals: list-like, len() must keep growing while "live" (the
    staleness watchdog relies on this), and sensor.fz_vals[-1] must be a
    signed float force in N.
That's the whole contract. Both backends below satisfy it, plus a few extra
list attributes / start()/stop() for lifecycle and CSV-logging parity with
move_to_spheres.execute().

RealSensorBackend wraps Bota_sys.BotaSerialSensor. Its background read
thread only updates private _fx.._mz scalars -- a SEPARATE method,
update_plot(), must be run as its own thread to actually append to the
public fz_vals/fx_vals/etc lists that the rest of this app reads.
RealSensorBackend.start() starts it explicitly.

`Bota_sys` (and the plotly/IPython/pyserial/crc it drags in at module
scope) is imported lazily, inside RealSensorBackend.__init__, so that
NullSensorBackend-only dev/testing never needs any of that installed --
matching this repo's existing convention of only importing `fairino` where
a function actually needs it.

NullSensorBackend (no hardware attached -- the current state of the lab):
design is deliberately a hybrid, not either extreme alone:
  - fz_vals stays genuinely EMPTY by default. This is the honest, fail-safe
    behavior: a force-limited approach with no sensor attached should trip
    the staleness watchdog and stop itself, not silently report success.
    Faking "it just works" data would validate a false sense of confidence
    in a hardware path nobody has tested.
  - BUT a background thread appends tiny (+/-0.1N) liveness-only noise
    samples at roughly the real sensor's cadence, SOLELY so len(fz_vals)
    keeps growing and the staleness watchdog itself is exercisable (does it
    correctly NOT trip while samples keep flowing?) independent of the
    force-threshold logic. This is fake LIVENESS only, never fake FORCE --
    it will never on its own cross a force threshold.
  - .simulate_collision(force_n, hold_s) lets a developer/operator hold an
    elevated reading above threshold for hold_s seconds (default 1s), to
    exercise the success path (reached=True) for testing orchestration/GUI
    code end-to-end without any hardware. Wired to POST
    /api/simulate-collision + a GUI button, enabled only when
    config.SIMULATE_SENSOR is true. Deliberately a sustained hold, not a
    single injected sample: the noise loop overwrites fz_vals[-1] every
    ~33ms, a far narrower window than any real HTTP round-trip or human
    click can reliably land in -- confirmed by hand while building this,
    a single-sample version silently missed nearly every external trigger.
"""

import threading
import time


class NullSensorBackend:
    NOISE_AMPLITUDE_N = 0.1
    NOISE_PERIOD_S = 1.0 / 30.0  # roughly matches BotaSerialSensor.update_plot's ~1/300s cadence order of magnitude, relaxed since it's just liveness
    DEFAULT_COLLISION_HOLD_S = 1.0

    def __init__(self):
        self._stop_event = threading.Event()
        self._thread = None
        self._start_time = None
        self._lock = threading.Lock()

        # A manually-triggered "collision" is held for this many seconds
        # (see simulate_collision()) rather than injected as a single
        # sample -- a single sample is invisible in practice: the noise
        # loop below overwrites fz_vals[-1] every ~33ms, which is far
        # narrower than any real HTTP round-trip or human click can
        # reliably land in. Holding it also happens to be more physically
        # honest: a real collision reads high force continuously for as
        # long as contact persists, not as one instantaneous spike.
        self._collision_until = 0.0
        self._collision_force_n = 0.0

        self.times = []
        self.real_times = []
        self.fx_vals = []
        self.fy_vals = []
        self.fz_vals = []
        self.mx_vals = []
        self.my_vals = []
        self.mz_vals = []

    def start(self):
        self._stop_event.clear()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._noise_loop, daemon=True)
        self._thread.start()

    def _noise_loop(self):
        import random

        while not self._stop_event.is_set():
            now = time.time()
            with self._lock:
                if now < self._collision_until:
                    fz = self._collision_force_n + random.uniform(-self.NOISE_AMPLITUDE_N, self.NOISE_AMPLITUDE_N)
                else:
                    fz = random.uniform(-self.NOISE_AMPLITUDE_N, self.NOISE_AMPLITUDE_N)
                self.times.append(now - self._start_time)
                self.real_times.append(now)
                self.fx_vals.append(0.0)
                self.fy_vals.append(0.0)
                self.fz_vals.append(fz)
                self.mx_vals.append(0.0)
                self.my_vals.append(0.0)
                self.mz_vals.append(0.0)
            time.sleep(self.NOISE_PERIOD_S)

    def simulate_collision(self, force_n=15.0, hold_s=None):
        """Hold an elevated force reading for `hold_s` seconds (default
        DEFAULT_COLLISION_HOLD_S), so a developer/operator can exercise the
        "contact reached" success path without real hardware -- long enough
        for a browser click or an HTTP-driven test to reliably land inside
        the window, unlike a single injected sample. Not fake sensor
        emulation of a real collision profile -- a deliberate, explicit
        test hook."""
        with self._lock:
            self._collision_force_n = float(force_n)
            self._collision_until = time.time() + (hold_s if hold_s is not None else self.DEFAULT_COLLISION_HOLD_S)

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class RealSensorBackend:
    def __init__(self, port):
        from Bota_sys import BotaSerialSensor  # lazy: see module docstring

        self._sensor = BotaSerialSensor(port)
        self._update_thread = None

    def start(self):
        self._sensor.start()
        # BotaSerialSensor's own read thread never populates the public
        # *_vals lists -- only update_plot() does, and it must be run as
        # its own thread.
        self._update_thread = threading.Thread(target=self._sensor.update_plot, daemon=True)
        self._update_thread.start()

    def stop(self):
        self._sensor.stop()

    @property
    def fz_vals(self):
        return self._sensor.fz_vals

    @property
    def fx_vals(self):
        return self._sensor.fx_vals

    @property
    def fy_vals(self):
        return self._sensor.fy_vals

    @property
    def mx_vals(self):
        return self._sensor.mx_vals

    @property
    def my_vals(self):
        return self._sensor.my_vals

    @property
    def mz_vals(self):
        return self._sensor.mz_vals

    @property
    def times(self):
        return self._sensor.times

    @property
    def real_times(self):
        return self._sensor.real_times

    def simulate_collision(self, force_n=15.0, hold_s=None):
        # No-op on real hardware -- surfaced as a 400 by the route, not here.
        raise NotImplementedError("simulate_collision is only available on the simulated sensor backend")


def get_backend():
    import config

    if config.SIMULATE_SENSOR:
        return NullSensorBackend()
    return RealSensorBackend(config.SENSOR_PORT)
