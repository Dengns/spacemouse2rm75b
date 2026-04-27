"""SpaceMouse reader using libspnav (non-blocking polling in a background thread)."""

import ctypes
import threading
import time

from config import LIBSPNAV_PATH, DEADZONE


# ---------- ctypes event structs (matching spnav.h) ----------

class MotionEvent(ctypes.Structure):
    _fields_ = [
        ("type",   ctypes.c_int),
        ("x",      ctypes.c_int),
        ("y",      ctypes.c_int),
        ("z",      ctypes.c_int),
        ("rx",     ctypes.c_int),
        ("ry",     ctypes.c_int),
        ("rz",     ctypes.c_int),
        ("period", ctypes.c_uint),
        ("data",   ctypes.c_void_p),
    ]


class ButtonEvent(ctypes.Structure):
    _fields_ = [
        ("type",  ctypes.c_int),
        ("press", ctypes.c_int),
        ("bnum",  ctypes.c_int),
    ]


class SpnavEvent(ctypes.Union):
    _fields_ = [
        ("type",   ctypes.c_int),
        ("motion", MotionEvent),
        ("button", ButtonEvent),
    ]


SPNAV_EVENT_MOTION = 1
SPNAV_EVENT_BUTTON = 2


class SpaceMouseReader:
    """Non-blocking SpaceMouse reader running in a daemon thread."""

    def __init__(self, lib_path: str = LIBSPNAV_PATH, deadzone: int = DEADZONE):
        self._lib = ctypes.CDLL(lib_path)
        self._deadzone = deadzone

        # shared state protected by lock
        self._lock = threading.Lock()
        self._axes = [0, 0, 0, 0, 0, 0]        # x, y, z, rx, ry, rz (raw, dead-zoned)
        self._buttons = {}                       # bnum -> pressed (bool)
        self._button_events = []                 # [(bnum, pressed), ...]

        self._stop_event = threading.Event()
        self._thread = None

    # ------ lifecycle ------

    def open(self):
        """Connect to spacenavd."""
        if self._lib.spnav_open() == -1:
            raise RuntimeError("Cannot connect to spacenavd. Is the daemon running?")
        buf = ctypes.create_string_buffer(256)
        self._lib.spnav_dev_name(buf, 256)
        name = buf.value.decode()
        axes = self._lib.spnav_dev_axes()
        buttons = self._lib.spnav_dev_buttons()
        print(f"SpaceMouse connected: {name}  (axes={axes}, buttons={buttons})")

    def start(self):
        """Start the background polling thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop polling and disconnect."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._lib.spnav_close()
        print("SpaceMouse disconnected.")

    # ------ public accessors (thread-safe) ------

    def get_axes(self):
        """Return latest [x, y, z, rx, ry, rz] with dead-zone applied."""
        with self._lock:
            return list(self._axes)

    def get_button(self, bnum: int) -> bool:
        """Return current pressed state of button *bnum*."""
        with self._lock:
            return self._buttons.get(bnum, False)

    def pop_button_events(self):
        """Pop and return all queued button events as [(bnum, pressed), ...]."""
        with self._lock:
            events = list(self._button_events)
            self._button_events.clear()
            return events

    # ------ internals ------

    def _apply_deadzone(self, value: int) -> int:
        if abs(value) < self._deadzone:
            return 0
        return value - self._deadzone if value > 0 else value + self._deadzone

    def _poll_loop(self):
        ev = SpnavEvent()
        while not self._stop_event.is_set():
            ret = self._lib.spnav_poll_event(ctypes.byref(ev))
            if ret == 0:
                # no event available
                time.sleep(0.001)
                continue

            if ev.type == SPNAV_EVENT_MOTION:
                m = ev.motion
                raw_axes = [m.x, m.y, m.z, m.rx, m.ry, m.rz]
                if any(v < -430 for v in raw_axes):
                    raise Exception(f"mouse get axes error {raw_axes}")
                axes = [self._apply_deadzone(v) for v in raw_axes]
                with self._lock:
                    self._axes = axes

            elif ev.type == SPNAV_EVENT_BUTTON:
                b = ev.button
                pressed = bool(b.press)
                with self._lock:
                    self._buttons[b.bnum] = pressed
                    self._button_events.append((b.bnum, pressed))


# ---------- standalone test ----------

if __name__ == "__main__":
    import signal

    reader = SpaceMouseReader()
    reader.open()
    reader.start()

    def _quit(sig, frame):
        reader.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _quit)

    print("Move SpaceMouse / press buttons... (Ctrl+C to quit)\n")
    while True:
        axes = reader.get_axes()
        btn_events = reader.pop_button_events()
        for bnum, pressed in btn_events:
            print(f"  Button {bnum} {'pressed' if pressed else 'released'}")
        print(
            f"\rT({axes[0]:5d},{axes[1]:5d},{axes[2]:5d})  "
            f"R({axes[3]:5d},{axes[4]:5d},{axes[5]:5d})",
            end="", flush=True,
        )
        time.sleep(0.02)
