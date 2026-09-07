"""state.py -- thread-safe shared run status + web-driven confirmation.

Exactly one run (mode1/mode2/mode3) can be active at a time, so a single
process-wide RunState instance is enough -- no per-run IDs needed.

The interesting part is request_confirm()/respond_confirm(): the background
hardware thread needs to pause and wait for an operator's click in the
browser, which is a fundamentally different shape than the old CLI's
blocking input(). The mechanism: the hardware thread publishes
`pending_confirm` (so /api/status pollers can see it and render buttons)
and then blocks on a plain threading.Event -- no busy-waiting. The Flask
thread handling the operator's button click sets the event, which
unblocks the hardware thread with the operator's choice.
"""

import threading
import time
from collections import deque


class RunState:
    FORCE_HISTORY_MAXLEN = 5000

    def __init__(self):
        self._lock = threading.Lock()
        self.mode = None  # "mode1" | "mode2" | "mode3" | None
        self.status_text = "idle"
        self.running = False
        self.stop_requested = False
        self.records = []
        self.force_history = {
            "t": deque(maxlen=self.FORCE_HISTORY_MAXLEN),
            "f": deque(maxlen=self.FORCE_HISTORY_MAXLEN),
        }
        self.pending_confirm = None
        self.error = None
        self.registration_result = None
        self.heightmap = None
        self.run_dir = None
        self.run_id = None

        self._confirm_event = threading.Event()
        self._confirm_response = None

        # Only the Null sensor backend needs to be reachable from a route
        # (POST /api/simulate-collision) while a run is active.
        self.active_sensor = None

    # -- lifecycle ---------------------------------------------------

    def start_run(self, mode, run_id, run_dir):
        with self._lock:
            self.mode = mode
            self.run_id = run_id
            self.run_dir = run_dir
            self.status_text = "starting..."
            self.running = True
            self.stop_requested = False
            self.records = []
            self.force_history["t"].clear()
            self.force_history["f"].clear()
            self.pending_confirm = None
            self.error = None
            self.registration_result = None
            self.heightmap = None
            self._confirm_response = None
            self._confirm_event.clear()

    def finish_run(self, error=None):
        with self._lock:
            self.running = False
            self.pending_confirm = None
            self.active_sensor = None
            if error:
                self.error = str(error)
                self.status_text = f"error: {error}"
            self._confirm_event.set()  # release anyone still waiting

    # -- simple setters (all lock-guarded) ----------------------------

    def set_status(self, text):
        with self._lock:
            self.status_text = text

    def append_record(self, record):
        with self._lock:
            self.records.append(record)

    def append_force_sample(self, t, f):
        with self._lock:
            self.force_history["t"].append(t)
            self.force_history["f"].append(f)

    def set_registration_result(self, result):
        with self._lock:
            self.registration_result = result

    def set_heightmap(self, heightmap):
        with self._lock:
            self.heightmap = heightmap

    def request_stop(self):
        with self._lock:
            self.stop_requested = True

    def is_stop_requested(self):
        with self._lock:
            return self.stop_requested

    # -- web-driven confirmation ---------------------------------------

    def request_confirm(self, payload, timeout_s=None):
        """Called from the background hardware thread. Publishes
        `pending_confirm` then blocks (lock released) until the browser
        posts a response, or forever if timeout_s is None. Returns
        "continue" | "skip" | "abort"."""
        with self._lock:
            self.pending_confirm = payload
            self._confirm_response = None
            self._confirm_event.clear()

        self._confirm_event.wait(timeout=timeout_s)

        with self._lock:
            resp = self._confirm_response
            self.pending_confirm = None
        return resp or "abort"  # a timeout with no response fails safe

    def respond_confirm(self, response):
        """Called from the Flask request thread handling POST /api/confirm
        or POST /api/abort. Returns False if there was nothing pending."""
        with self._lock:
            if self.pending_confirm is None and response != "abort":
                return False
            self._confirm_response = response
        self._confirm_event.set()
        return True

    # -- snapshot for /api/status ---------------------------------------

    def snapshot(self):
        with self._lock:
            return {
                "mode": self.mode,
                "run_id": self.run_id,
                "status_text": self.status_text,
                "running": self.running,
                "records": list(self.records),
                "force_history": {
                    "t": list(self.force_history["t"]),
                    "f": list(self.force_history["f"]),
                },
                "pending_confirm": self.pending_confirm,
                "error": self.error,
                "registration_result": self.registration_result,
                "heightmap": self.heightmap,
            }


# module-level singleton -- see class docstring for why one is enough.
state = RunState()
