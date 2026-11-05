from motion_controller import MotionController

import atexit
import csv
import random
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

import math3d as m3d


NUM_MOTORS = 3
DEFAULT_SPEED = 1000          # us per step (lower = faster)
COUNTS_PER_STEP = 5           # encoder counts per motor step
ENCODER_LOG_DELTA = 100       # log encoder changes larger than this
SETTLE_FRAMES = 2             # encoder frames a motor counts as busy after DONE/stop/null
ECHO_MODES = ("off", "external", "on")


# =====================================
# CSV LOG
# =====================================

class CsvLog:
    """
    One CSV file per log segment:  logs/<date>_<start-time>_<seq>.csv
    e.g. logs/2026-11-05_134512_001.csv

    date/start-time identify the program run, seq counts the segments
    within that run (a new segment starts after every reset). The segment
    after a reset is only created with its first event, so quitting right
    after a reset leaves no empty log behind.
    A closed segment is also copied to logs/latest.csv.

    Next to it, <name>_pretty.csv is written for Excel with Norwegian locale
    (';' separator, decimal comma): one column per motor, each row holding
    the last known value of every motor, t = seconds since the segment start.
    Copied to logs/latest_pretty.csv on close.
    """

    FIELDS = ["time", "timestamp", "type", "cmd", "channel", "value", "info"]
    PRETTY_FIELDS = (["time", "t [s]"] + [f"M{m}" for m in range(NUM_MOTORS)]
                     + ["info"])
    # row types whose value is an encoder position
    POSITION_TYPES = ("ENC", "ENC_EXT", "DONE", "READ", "RESET")

    def __init__(self, folder=Path(__file__).parent / "logs"):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.session = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.seq = 0
        self.lock = threading.Lock()
        self.closed = True
        self.pending = None   # previous path while the next segment is not yet created
        self.last = [""] * NUM_MOTORS   # last known positions, kept across segments
        with self.lock:
            self._open(previous=None)

    def _open(self, previous):
        self.seq += 1
        self.path = self.folder / f"{self.session}_{self.seq:03d}.csv"
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.FIELDS)
        self.pretty_path = self.path.with_name(self.path.stem + "_pretty.csv")
        # utf-8-sig: BOM so Excel detects the encoding
        self.pretty_file = open(self.pretty_path, "w", newline="",
                                encoding="utf-8-sig")
        self.pretty_writer = csv.writer(self.pretty_file, delimiter=";")
        self.pretty_writer.writerow(self.PRETTY_FIELDS)
        self.t0 = time.time()
        info = "log started"
        if previous:
            info += f" (continues {previous.name})"
        self.closed = False
        self._row("INFO", info)

    def _close(self, reason):
        self._row("INFO", f"log closed: {reason}")
        self.file.close()
        self.pretty_file.close()
        self.closed = True
        for src, dst in ((self.path, "latest.csv"),
                         (self.pretty_path, "latest_pretty.csv")):
            try:
                shutil.copyfile(src, self.folder / dst)
            except OSError as e:  # e.g. file open in Excel
                print(f"warning: could not update {dst}: {e}")
        return self.path

    def _row(self, type_, info, cmd="", channel: object = "", value: object = ""):
        now = time.time()
        human = datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.writer.writerow([human, f"{now:.3f}", type_, cmd, channel, value, info])
        self.file.flush()
        # pretty (Norwegian Excel) version
        if type_ in self.POSITION_TYPES and channel != "":
            self.last[int(channel)] = value
        human_no = human.replace(".", ",")   # 13:04:57,912
        t = f"{now - self.t0:.3f}".replace(".", ",")
        self.pretty_writer.writerow([human_no, t] + self.last + [info])
        self.pretty_file.flush()
        return human

    def write(self, type_, info, cmd="", channel: object = "", value: object = "",
              echo=True):
        with self.lock:
            if self.closed:
                return
            if self.pending is not None:   # first event after a reset
                self._open(previous=self.pending)
                self.pending = None
                print(f"new log: {self.path.name}")
            human = self._row(type_, info, cmd, channel, value)
        if echo:
            print(f"{human}  {info}")

    def rotate(self, reason):
        """Close the current segment; the next one starts with the next event."""
        with self.lock:
            old = self._close(reason)
            self.closed = False
            self.pending = old
        print(f"log saved: {old.name}")

    @property
    def idle(self):
        """True if nothing was logged since the last rotate."""
        return self.pending is not None

    def close(self, reason):
        with self.lock:
            if self.closed:
                return
            if self.pending is not None:   # nothing happened since the reset
                self.closed = True
                self.pending = None
                return
            old = self._close(reason)
        print(f"log saved: {old.name}")


# =====================================
# PANEL INTERFACE
# =====================================

class PanelInterface:
    def __init__(self, log=None):
        self.log = log or CsvLog()
        self._pending_done = set()
        # motors whose encoder changes are caused by us:
        #   None = commanded move running, n = settle for n more frames
        self._busy = {}
        self._busy_lock = threading.Lock()
        # console echo of encoder changes: off | external | on
        self.echo = "external"
        self._closed = False
        # also stop + save when used from a REPL / script that just exits
        atexit.register(self.close, "program exit")
        self.mc = MotionController(
            encoder_callback=self._encoder_changed,
            encoder_callback_delta=ENCODER_LOG_DELTA,
            motor_done_callback=self._motor_done,
            frame_callback=self._encoder_frame,
            homed_callback=None,
            estop_callback=None,
            debug=False)

    # ---- callbacks (run on the serial worker thread) ----

    def _encoder_changed(self, encoder, value, delta):
        with self._busy_lock:
            external = encoder not in self._busy
        if external:
            type_, info = "ENC_EXT", f"E{encoder}: {value} (delta={delta}, external)"
        else:
            type_, info = "ENC", f"E{encoder}: {value} (delta={delta})"
        echo = self.echo == "on" or (self.echo == "external" and external)
        self.log.write(type_, info, channel=encoder, value=value, echo=echo)

    def _motor_done(self, motor):
        # encoder values arrive only every 50 ms, so the value known
        # at DONE is stale -> log it with the next encoder frame
        self._pending_done.add(motor)
        self._settle(motor)

    def _encoder_frame(self, values):
        for motor in sorted(self._pending_done):
            self.log.write("DONE", f"M{motor} done at {values[motor]}",
                           channel=motor, value=values[motor])
        self._pending_done.clear()
        # count down settling motors
        with self._busy_lock:
            for m, n in list(self._busy.items()):
                if n is None:
                    continue
                if n <= 1:
                    del self._busy[m]
                else:
                    self._busy[m] = n - 1

    def _settle(self, m, keep_moving=False):
        """Treat encoder m as ours for a few more frames."""
        with self._busy_lock:
            if keep_moving and m in self._busy and self._busy[m] is None:
                return
            self._busy[m] = SETTLE_FRAMES

    # ---- helpers ----

    def wait_ready(self, timeout=15):
        """Block until the first encoder values have arrived."""
        t0 = time.time()
        while not self.mc.get_encoders():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.1)
        return True

    def wait(self, events):
        """Wait for DONE events, then for fresh encoder values."""
        # short waits so Ctrl+C stays responsive on Windows
        for ev in events:
            if ev is None:
                continue
            while not ev.wait(0.1):
                pass
        self.mc.wait_encoder_update()

    # ---- commands ----

    def get(self, m):
        return self.mc.get_encoders()[m]

    def read(self, motors: Iterable[int] = range(NUM_MOTORS)):
        values = self.mc.get_encoders()
        for m in motors:
            self.log.write("READ", f"E{m}: {values[m]}",
                           cmd=f"read {m}", channel=m, value=values[m])
        return [values[m] for m in motors]

    def move(self, m, val_delta, speed=DEFAULT_SPEED, waitFor=True, cmd=None):
        """Relative move. Returns the DONE event when waitFor=False."""
        cmd = cmd or f"move {m} {val_delta} {speed}"
        steps = int(abs(val_delta) // COUNTS_PER_STEP)
        if steps == 0:
            # firmware would still take one step for MV with 0 steps
            self.log.write("CMD", f"M{m}: nothing to move (delta={val_delta})",
                           cmd=cmd, channel=m, value=self.get(m))
            return None
        direction = 0 if val_delta > 0 else 1
        with self._busy_lock:
            self._busy[m] = None
        self.log.write("CMD", f"M{m}: move {val_delta:+} from {self.get(m)} "
                              f"({steps} steps, speed {speed})",
                       cmd=cmd, channel=m, value=val_delta)
        return self.mc.move_motor(motor=m, steps=steps, direction=direction,
                                  speed_us=speed, waitFor=waitFor)

    def set(self, m, val, speed=DEFAULT_SPEED, waitFor=True, cmd=None):
        """Absolute move to encoder value."""
        cmd = cmd or f"set {m} {val} {speed}"
        self.log.write("CMD", f"M{m}: set to {val}", cmd=cmd, channel=m, value=val)
        return self.move(m, val - self.get(m), speed, waitFor, cmd=cmd)

    def setMulti(self, v1, v2, v3):
        self.set(0, v1, waitFor=False)
        self.set(1, v2, waitFor=False)
        self.set(2, v3, waitFor=False)

    def null(self, motors: Iterable[int] = range(NUM_MOTORS)):
        for m in motors:
            self.log.write("CMD", f"E{m}: zeroed (was {self.get(m)})",
                           cmd=f"null {m}", channel=m, value=0)
            self._settle(m, keep_moving=True)
            self.mc.zero_encoder(m)

    def stop(self, motors: Iterable[int] = range(NUM_MOTORS)):
        for m in motors:
            self._settle(m)
            self.mc.stop_motor(m)
            self.log.write("CMD", f"M{m}: stop at {self.get(m)}",
                           cmd=f"stop {m}", channel=m, value=self.get(m))

    def reset(self, motors: Iterable[int] = range(NUM_MOTORS), speed=DEFAULT_SPEED):
        """Move motors to 0, wait until done, then start a new log file."""
        motors = list(motors)
        name = "all" if len(motors) == NUM_MOTORS else " ".join(map(str, motors))
        self.log.write("CMD", f"reset {name}: moving to 0", cmd=f"reset {name}")
        events = [self.set(m, 0, speed, waitFor=False, cmd=f"reset {name}")
                  for m in motors]
        try:
            self.wait(events)
        except KeyboardInterrupt:
            self.stop(motors)
            self.log.write("INFO", f"reset {name} aborted by user")
            return False
        for m in motors:
            self.log.write("RESET", f"M{m}: reset done at {self.get(m)}",
                           cmd=f"reset {name}", channel=m, value=self.get(m))
        self.log.rotate(f"reset {name}")
        return True

    def close(self, reason="program closed"):
        """Stop all motors, save the log, disconnect. Safe to call twice."""
        if self._closed:
            return
        self._closed = True
        if self.log.idle:
            # quit right after a reset: don't start a new log just for this
            self.log.close(reason)
        self.stop()
        self.log.write("INFO", reason)
        self.log.close(reason)
        self.mc.close()


def get_interface():
    pi = PanelInterface()
    pi.wait_ready()
    return pi


DEMO_M2_HIGH = 15000   # motor 2 alternates between this and 0 (never below 0)


def demo(pi, rounds=5):
    """
    Like the old demo, but with absolute targets and without zeroing the
    encoders: M0/M1 go to random positions in [-10000, 10000] (M1 slow),
    M2 alternates between DEMO_M2_HIGH and 0. Each round waits for M2.
    Ends with 'reset all', which also starts a new log. Ctrl+C stops.
    """
    cmd = f"demo {rounds}"
    pi.log.write("CMD", f"demo: {rounds} rounds", cmd=cmd)
    try:
        for i in range(rounds):
            m2_target = DEMO_M2_HIGH if i % 2 == 0 else 0
            pi.log.write("INFO", f"demo round {i + 1}/{rounds}", cmd=cmd)
            e0 = pi.set(0, random.randint(-10000, 10000), waitFor=False, cmd=cmd)
            e1 = pi.set(1, random.randint(-10000, 10000), speed=8000,
                        waitFor=False, cmd=cmd)
            e2 = pi.set(2, m2_target, waitFor=False, cmd=cmd)
            # pace by M2 like before; if M2 had nothing to do, wait for the others
            pi.wait([e2] if e2 else [e0, e1])
    except KeyboardInterrupt:
        pi.stop()
        pi.log.write("INFO", "demo aborted by user", cmd=cmd)
        return False
    pi.log.write("INFO", "demo finished, resetting", cmd=cmd)
    return pi.reset()


# =====================================
# COMMAND LINE
# =====================================

HELP = f"""\
commands  (m = motor 0..{NUM_MOTORS - 1}, or 'all'; omitted = all)
  read  [m]                  show encoder value(s)
  move  <m> <delta> [speed]  relative move (encoder counts)
  set   <m> <value> [speed]  absolute move to encoder value
  null  [m]                  zero encoder(s) at current position
  reset [m]                  move to 0, wait, save log and start a new one
  stop  [m]                  stop motor(s)
  demo  [rounds]             random demo motion (default 5 rounds), ends with reset
  echo  [off|external|on]    encoder changes on the console: none, only
                             ones not caused by a command (default), all
  help                       this text
  quit                       save log and exit
speed = microseconds per step (default {DEFAULT_SPEED}, lower is faster)
"""


def _motors(arg):
    if arg is None or arg == "all":
        return list(range(NUM_MOTORS))
    m = int(arg)
    if not 0 <= m < NUM_MOTORS:
        raise ValueError(f"motor must be 0..{NUM_MOTORS - 1}")
    return [m]


def _motor(arg):
    m = int(arg)
    if not 0 <= m < NUM_MOTORS:
        raise ValueError(f"motor must be 0..{NUM_MOTORS - 1}")
    return m


def run_cli():
    pi = PanelInterface()
    print(f"logging to {pi.log.path}")
    print("waiting for controller ...")
    if not pi.wait_ready():
        pi.close("no encoder data from controller")
        print("no encoder data from controller, giving up")
        return
    pi.read()
    print(HELP)

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *args = line.split()
            cmd = cmd.lower()
            arg = args[0] if args else None
            try:
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd in ("h", "help", "?"):
                    print(HELP)
                elif cmd in ("r", "read"):
                    pi.read(_motors(arg))
                elif cmd in ("m", "move"):
                    speed = int(args[2]) if len(args) > 2 else DEFAULT_SPEED
                    pi.move(_motor(args[0]), int(args[1]), speed,
                            waitFor=False, cmd=line)
                elif cmd in ("s", "set"):
                    speed = int(args[2]) if len(args) > 2 else DEFAULT_SPEED
                    pi.set(_motor(args[0]), int(args[1]), speed,
                           waitFor=False, cmd=line)
                elif cmd in ("n", "null"):
                    pi.null(_motors(arg))
                elif cmd == "reset":
                    pi.reset(_motors(arg))
                elif cmd == "stop":
                    pi.stop(_motors(arg))
                elif cmd == "demo":
                    demo(pi, int(arg) if arg else 5)
                elif cmd == "echo":
                    if arg is not None:
                        if arg not in ECHO_MODES:
                            raise ValueError(f"echo mode must be {'|'.join(ECHO_MODES)}")
                        pi.echo = arg
                        pi.log.write("INFO", f"encoder echo: {arg}", cmd=line,
                                     echo=False)
                    print(f"encoder echo: {pi.echo}")
                else:
                    print(f"unknown command '{cmd}', type 'help'")
            except (IndexError, ValueError) as e:
                print(f"bad arguments: {e or 'missing value'}  (type 'help')")
    except KeyboardInterrupt:
        print()
    finally:
        pi.close()


if __name__ == "__main__":
    run_cli()
