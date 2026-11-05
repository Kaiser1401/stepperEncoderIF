import serial
import serial.tools.list_ports

import threading
import queue
import time


class MotionController:

    # =====================================
    # INIT
    # =====================================

    def __init__(
        self,
        baudrate=115200,
        encoder_callback=None,
        encoder_callback_delta=10,
        motor_done_callback=None,
        homed_callback=None,
        estop_callback=None,
        frame_callback=None,
        debug=False
    ):

        self.baudrate = baudrate

        self.debug = debug

        # callbacks

        self.encoder_callback = (
            encoder_callback
        )

        self.motor_done_callback = (
            motor_done_callback
        )

        self.homed_callback = (
            homed_callback
        )

        self.estop_callback = (
            estop_callback
        )

        self.encoder_callback_delta = (
            encoder_callback_delta
        )

        # called with the values of every
        # ENC frame (on the worker thread)

        self.frame_callback = (
            frame_callback
        )

        # serial

        self.serial_port = None

        self.connected = False

        self.running = True

        # state

        self.encoder_values = []

        self.last_callback_values = []

        self.lock = threading.Lock()

        # counts received ENC frames

        self.enc_frame = 0

        self.enc_cond = threading.Condition()

        # synchronization

        self.wait_events = {}

        self.wait_lock = threading.Lock()

        # tx

        self.tx_queue = queue.Queue()

        # worker

        self.thread = threading.Thread(
            target=self._worker,
            daemon=True
        )

        self.thread.start()

    # =====================================
    # DEBUG
    # =====================================

    def _debug(self, *args):

        if self.debug:
            print(*args)

    # =====================================
    # PORT DETECTION
    # =====================================

    def _find_arduino(self):

        ports = (
            serial.tools
            .list_ports
            .comports()
        )

        for port in ports:

            desc = (
                port.description
                .lower()
            )

            if (
                "arduino" in desc
                or "usb serial" in desc
            ):
                return port.device

        return None

    # =====================================
    # CONNECT
    # =====================================

    def _connect(self):

        while self.running:

            try:

                port = self._find_arduino()

                if port is None:

                    self._debug(
                        "Arduino not found"
                    )

                    time.sleep(2)

                    continue

                self._debug(
                    f"Connecting to {port}"
                )

                # open with DTR/RTS low: raising DTR
                # on open triggers the Arduino
                # auto-reset, which would restart
                # the firmware and zero the encoders

                sp = serial.Serial()

                sp.port = port

                sp.baudrate = self.baudrate

                sp.timeout = 0.05

                sp.dtr = False

                sp.rts = False

                sp.open()

                self.serial_port = sp

                # only needed if the board did
                # reset anyway (bootloader delay)

                time.sleep(2)

                self.connected = True

                self._debug("Connected")

                return

            except Exception as e:

                self._debug(
                    "Connection failed:",
                    e
                )

                time.sleep(2)

    # =====================================
    # THREAD
    # =====================================

    def _worker(self):

        self._connect()

        while self.running:

            try:

                if not self.connected:

                    self._connect()

                while not self.tx_queue.empty():

                    cmd = self.tx_queue.get()

                    self.serial_port.write(
                        (cmd + "\n").encode()
                    )

                line = (
                    self.serial_port
                    .readline()
                    .decode(errors="ignore")
                    .strip()
                )

                if line:

                    self._handle_line(line)

            except Exception as e:

                self._debug(
                    "Disconnected:",
                    e
                )

                self.connected = False

                try:
                    self.serial_port.close()
                except:
                    pass

                time.sleep(1)

    # =====================================
    # RX
    # =====================================

    def _handle_line(self, line):

        self._debug("RX:", line)

        # encoder stream

        if line.startswith("ENC"):

            parts = line.split(",")

            values = [
                int(v)
                for v in parts[1:]
            ]

            with self.lock:

                if not self.last_callback_values:

                    self.last_callback_values = (
                        values.copy()
                    )

                self.encoder_values = values

            self._process_encoder_callbacks()

            if self.frame_callback:

                self.frame_callback(
                    values.copy()
                )

            with self.enc_cond:

                self.enc_frame += 1

                self.enc_cond.notify_all()

        # move complete

        elif line.startswith("DONE"):

            motor = int(
                line.split(",")[1]
            )

            self._signal_wait(
                f"DONE_{motor}"
            )

            if self.motor_done_callback:

                self.motor_done_callback(
                    motor
                )

        # homed

        elif line.startswith("HOMED"):

            motor = int(
                line.split(",")[1]
            )

            self._signal_wait(
                f"HOMED_{motor}"
            )

            if self.homed_callback:

                self.homed_callback(
                    motor
                )

        # estop

        elif line.startswith("ESTOP"):

            self._signal_wait(
                "ESTOP"
            )

            if self.estop_callback:

                self.estop_callback()

    # =====================================
    # CALLBACKS
    # =====================================

    def _process_encoder_callbacks(
        self
    ):

        if self.encoder_callback is None:
            return

        with self.lock:

            current = (
                self.encoder_values
                .copy()
            )

            previous = (
                self.last_callback_values
                .copy()
            )

        for i in range(len(current)):

            delta = abs(
                current[i]
                - previous[i]
            )

            if (
                delta >=
                self.encoder_callback_delta
            ):

                self.encoder_callback(
                    i,
                    current[i],
                    delta
                )

                with self.lock:

                    self.last_callback_values[i] = (
                        current[i]
                    )

    # =====================================
    # WAIT HELPERS
    # =====================================

    def _create_wait_event(
        self,
        key
    ):

        event = threading.Event()

        with self.wait_lock:

            self.wait_events[key] = event

        return event

    def _signal_wait(
        self,
        key
    ):

        with self.wait_lock:

            if key in self.wait_events:

                self.wait_events[key].set()

    # =====================================
    # SEND
    # =====================================

    def _send(
        self,
        cmd
    ):

        self.tx_queue.put(cmd)

    # =====================================
    # API
    # =====================================

    def enable_motor(
        self,
        motor,
        enable=True
    ):

        self._send(
            f"EN,{motor},{1 if enable else 0}"
        )

    def move_motor(
        self,
        motor,
        steps,
        direction,
        speed_us,
        waitFor=True,
        timeout=None
    ):

        # always create the event so non-blocking
        # callers can wait on it later

        event = self._create_wait_event(
            f"DONE_{motor}"
        )

        self._send(
            f"MV,{motor},{steps},"
            f"{direction},{speed_us}"
        )

        if waitFor:

            return event.wait(timeout)

        return event

    def home_motor(
        self,
        motor,
        encoder,
        direction,
        speed_us=2000,
        stall_ms=300,
        stall_threshold=3,
        waitFor=True,
        timeout=None
    ):

        if waitFor:

            event = self._create_wait_event(
                f"HOMED_{motor}"
            )

        self._send(
            f"HOME,{motor},{encoder},"
            f"{direction},{speed_us},"
            f"{stall_ms},{stall_threshold}"
        )

        if waitFor:

            return event.wait(timeout)

    def stop_motor(
        self,
        motor
    ):

        self._send(
            f"ST,{motor}"
        )

    def zero_encoder(
        self,
        encoder
    ):

        self._send(
            f"ZEROENC,{encoder}"
        )

    def estop(
        self,
        waitFor=True,
        timeout=None
    ):

        if waitFor:

            event = self._create_wait_event(
                "ESTOP"
            )

        self._send("ESTOP")

        if waitFor:

            return event.wait(timeout)

    def reset_estop(self):

        self._send("RESET")

    def wait_encoder_update(
        self,
        timeout=1.0
    ):
        # wait for the next ENC frame, i.e.
        # values sampled after everything
        # received so far (e.g. a DONE).
        # do NOT call from a callback.

        with self.enc_cond:

            start = self.enc_frame

            return self.enc_cond.wait_for(
                lambda: self.enc_frame > start,
                timeout
            )

    def get_encoders(self):

        with self.lock:

            return (
                self.encoder_values
                .copy()
            )

    def close(self):

        # stop the worker, then send whatever
        # is still queued (e.g. stop commands)
        # before closing the port

        self.running = False

        self.thread.join(timeout=2)

        if self.serial_port and self.connected:

            try:

                while not self.tx_queue.empty():

                    cmd = self.tx_queue.get()

                    self.serial_port.write(
                        (cmd + "\n").encode()
                    )

                self.serial_port.flush()

            except Exception as e:

                self._debug(
                    "Flush on close failed:",
                    e
                )

        if self.serial_port:

            self.serial_port.close()