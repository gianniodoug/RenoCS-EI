"""Prepared A22 motor backend; DISABLED unless explicitly authorized.

No port/process/thread is opened on import or construction. Never calls the
unbounded legacy connect/WaitACK. Protocol mirrors basic_motion/arduino.py:
115200, A22, C + <4h FR/FL/RR/RL>, B/OK; SetMotors truncates left/right to int.
T.9's BoundedBlunoStop only handles an already-open legacy object, so this full
transport owns its own bounded handshake, movement and stop transactions.

The sole serial owner is an isolated spawned worker. The bound controller is
its sole command producer. Parent waits have deadlines including serial open /
boot delay; a failed request poisons the session and terminates the worker.
No queued retries/reconnects. Port ownership is retained if the worker cannot
be reaped. OS scheduling/process creation cannot have hard real-time guarantees;
an independent physical stop remains mandatory. A lost link cannot deliver Stop.

HardwareAuthorization records EXTERNAL evidence, not proof manufactured here.
The quadrillage CLI uses the team's confirmed professor_A22 profile and requires
an explicit operator confirmation of a tested independent motor-power cutoff.
Alternatively, callers can retain source path/hash evidence as follows:
resolve arduino with find_spec using the SAME Pi interpreter/CWD/PYTHONPATH as
huit.py (do not import/run huit), hash/review that module and identify the actual
flashed firmware. Both original and watchdog candidates say version 1.0: the
handshake cannot distinguish them. Verify watchdog/cutoff physically and record
the procedure/result before setting independent_stop_verified. The supplied
source has an empty watchdog; the 350 ms candidate is not confirmed installed.

Default PWM limit is ZERO. Reject above-limit proposals rather than silently
clipping proposals inside this transport. The controller explicitly configures
the follower's base PWM/cap without changing its gains. Other programs
(huit, Arduino IDE, serial monitors) must be stopped: POSIX exclusive is advisory.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import math
import multiprocessing
from numbers import Real
import os
import signal
from pathlib import Path
import struct
import threading
import time

from intersections.bounded_bluno_stop import StopProtocolError
from intersections.fault_manager import SerialStatus


@dataclass(frozen=True)
class HardwareAuthorization:
    arduino_module_path: str
    arduino_sha256: str
    protocol_confirmed: bool = False
    independent_stop_verified: bool = False
    evidence_reference: str = ""  # Recorded Pi module/firmware + physical stop test.
    protocol_profile: str = ""  # Explicit team-confirmed professor A22 alternative.


@dataclass(frozen=True)
class BlunoConfig:
    port: str = "/dev/ttyACM0"  # Must be identified on the actual Pi.
    pwm_limit: int = 0
    connect_timeout_s: float = 4.0
    transaction_timeout_s: float = .1
    boot_wait_s: float = 2.0

    def __post_init__(self):
        if not isinstance(self.port, str) or not self.port or len(self.port) > 256:
            raise ValueError("identify one serial device")
        if type(self.pwm_limit) is not int or not 0 <= self.pwm_limit <= 255:
            raise ValueError("PWM limit must be an explicitly approved integer in [0,255]")
        for value in (self.connect_timeout_s, self.transaction_timeout_s):
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or not 0 < value <= 10:
                raise ValueError("finite positive time budgets <=10 s required")
        if (isinstance(self.boot_wait_s, bool) or not isinstance(self.boot_wait_s, Real)
                or not math.isfinite(self.boot_wait_s) or not 0 <= self.boot_wait_s < self.connect_timeout_s):
            raise ValueError("boot wait must fit inside the connection budget")


class _Wire:
    """Worker-only finite I/O. Each read/write uses the remaining total budget."""
    def __init__(self, port, deadline_ns, now_ns):
        self.port, self.deadline, self.now_ns = port, deadline_ns, now_ns
        self.last_ns = now_ns()

    def remaining(self):
        now = self.now_ns()
        if type(now) is not int or now < self.last_ns:
            raise StopProtocolError("invalid I/O clock")
        self.last_ns = now
        remaining = (self.deadline-now)/1e9
        if remaining <= 0:
            raise TimeoutError("A22 transaction deadline exceeded")
        return remaining

    def exchange(self, payload, expected):
        self.remaining()
        if self.port.in_waiting:
            raise StopProtocolError("unconsumed input; explicit resynchronization required")
        self.port.write_timeout = self.remaining()
        count = self.port.write(payload)
        self.remaining()
        if count != len(payload):
            raise OSError("partial command; session poisoned")
        line = bytearray()
        while len(line) < 128:
            self.port.timeout = self.remaining()
            byte = self.port.read(1)
            self.remaining()
            if byte:
                line.extend(byte)
                if byte == b"\n":
                    if bytes(line).strip() != expected:
                        raise StopProtocolError("unexpected A22 reply")
                    return
        raise StopProtocolError("overlong A22 reply")


def _error_state(error):
    return ("TIMEOUT" if isinstance(error, TimeoutError) or type(error).__name__ == "SerialTimeoutException"
            else "PROTOCOL_ERROR" if isinstance(error, StopProtocolError) else "LOST")


def _serial_worker(channel, config, serial_factory=None, now_ns=time.monotonic_ns, sleep=time.sleep):
    """Only this process owns serial. Test injection never constructs pySerial."""
    if multiprocessing.current_process().name != "MainProcess":
        # Terminal Ctrl+C must unwind the controller, which sends Stop; do not
        # kill its serial owner first. SIGTERM/kill remain available for abort.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    port = None
    connected = False
    try:
        while True:
            request = channel.recv()
            seq, operation, deadline, targets = request
            try:
                if operation == "connect":
                    if port is not None:
                        raise StopProtocolError("duplicate connection")
                    if now_ns() >= deadline:
                        raise TimeoutError("connection expired before open")
                    if serial_factory is None:
                        import serial  # Only after parent authorization + explicit connect.
                        serial_factory = serial.Serial
                    remaining = (deadline-now_ns())/1e9
                    port = serial_factory(port=config.port, baudrate=115200, timeout=remaining,
                                          write_timeout=remaining, exclusive=True)
                    wire = _Wire(port, deadline, now_ns)
                    ready = now_ns()+round(config.boot_wait_s*1e9)
                    while now_ns() < ready:
                        sleep(max(0., min(.01, wire.remaining(), (ready-now_ns())/1e9)))
                    wire.remaining()
                    port.reset_input_buffer()  # Startup only, never after a failed binary command.
                    wire.remaining()
                    wire.exchange(b"A22", b"OK Arduino connecte version 1.0 en mode 2")
                    # Do not send I0: supplied PROTECT_IR_code is an unacknowledged stub.
                    wire.exchange(b"C"+bytes(8), b"OK Moteurs mis aux consignes : 0 0 0 0")
                    connected = True
                else:
                    if not connected:
                        raise StopProtocolError("link is not connected")
                    wire = _Wire(port, deadline, now_ns)
                    if operation == "probe":
                        wire.exchange(b"B", b"OK")
                    elif operation in ("Stop", "SetMotors"):
                        if (not isinstance(targets, tuple) or len(targets) != 4
                                or any(type(x) is not int or abs(x) > config.pwm_limit for x in targets)
                                or (operation == "Stop" and any(targets))):
                            raise StopProtocolError("invalid PWM request")
                        expected = ("OK Moteurs mis aux consignes : " + " ".join(map(str, targets))).encode("ascii")
                        wire.exchange(b"C"+struct.pack("<4h", *targets), expected)
                    else:
                        raise StopProtocolError("unknown motor operation")
                channel.send((seq, True, "OK", ""))
            except Exception as error:
                channel.send((seq, False, _error_state(error), str(error)[:128]))
                break  # Never send another command on an uncertain/torn packet.
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        if port is not None:
            port.close()
        channel.close()


class A22MotorBackend:
    _ports = set()
    _ports_lock = threading.Lock()

    def __init__(self, *, clock_id, now_ns, config=None, authorization=None,
                 allow_hardware=False, _context=None, _deadline_ns=time.monotonic_ns):
        self.config = config or BlunoConfig()
        if not isinstance(clock_id, str) or not clock_id or not callable(now_ns):
            raise ValueError("declare the camera's clock domain and callable")
        self.clock_id, self.now_ns = clock_id, now_ns
        self.authorization, self.allow_hardware = authorization, allow_hardware
        self._context, self._deadline_ns = _context, _deadline_ns
        self.status = SerialStatus(clock_id, now_ns(), "UNVERIFIED")
        self.last_acknowledged_targets = None
        self.stop_acknowledged = False
        self.physical_stop_verified = False
        self.motor_commands_sent = False  # Conservative: a write may have occurred on timeout.
        self._owner = self._process = self._channel = None
        self._failed = self._connected = self._closed = False
        self._port_key = os.path.realpath(self.config.port)
        self._claimed = False
        self._seq = 0
        self._commands = deque(maxlen=128)
        self._lock = threading.Lock()

    @property
    def commands(self):
        return deepcopy(list(self._commands))

    @property
    def serial_connection_opened(self):
        # Conservative while open/handshake state is uncertain.
        return self._process is not None and self._process.is_alive()

    def assert_authorized(self, *, movement=False):
        auth = self.authorization
        if (self.allow_hardware is not True or not isinstance(auth, HardwareAuthorization)
                or auth.protocol_confirmed is not True or auth.independent_stop_verified is not True
                or not isinstance(auth.evidence_reference, str) or not auth.evidence_reference.strip()):
            raise PermissionError("hardware locked: Pi protocol and independent stop evidence required")
        path = Path(auth.arduino_module_path)
        confirmed_profile = (auth.protocol_profile == "professor_A22"
                             and auth.arduino_module_path == "" and auth.arduino_sha256 == "")
        if not confirmed_profile and (not path.is_absolute() or not path.is_file()
                or hashlib.sha256(path.read_bytes()).hexdigest() != auth.arduino_sha256):
            raise PermissionError("verified Pi Arduino module path/hash changed or unavailable")
        if movement and self.config.pwm_limit == 0:
            raise PermissionError("zero-only default: approve a tested PWM limit before movement")

    def bind(self, owner):
        if self._owner is not None:
            raise RuntimeError("one motor-command owner only")
        self._owner = owner

    def _check_owner(self, owner):
        if owner is not self._owner or owner is None:
            raise PermissionError("only the bound controller owns this motor connection")

    def connect(self, *, owner):
        self._check_owner(owner)
        self.assert_authorized()
        if self._process is not None or self._connected:
            raise RuntimeError("backend sessions cannot reconnect or connect twice")
        return self._request("connect", None)

    def probe(self, *, owner):
        self._check_owner(owner)
        return self._request("probe", None)

    def SetMotors(self, left, right, *, owner):
        self._check_owner(owner)
        if any(isinstance(x, bool) or not isinstance(x, Real) or not math.isfinite(x) for x in (left, right)):
            raise ValueError("finite signed PWM required")
        left, right = int(left), int(right)
        if max(abs(left), abs(right)) > self.config.pwm_limit:
            raise ValueError("proposal exceeds approved PWM limit; no clipping")
        return self._request("SetMotors", (right, left, right, left))

    def Stop(self, *, owner):
        self._check_owner(owner)
        return self._request("Stop", (0, 0, 0, 0))

    def _request(self, operation, targets):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("concurrent serial command forbidden")
        try:
            if self._closed or self._failed or (operation != "connect" and not self._connected):
                raise StopProtocolError("inactive/failed link: explicit safe reinitialization required")
            budget = self.config.connect_timeout_s if operation == "connect" else self.config.transaction_timeout_s
            deadline = self._deadline_ns()+round(budget*1e9)
            self.stop_acknowledged = False
            if operation == "connect":
                if self._process is not None:
                    raise StopProtocolError("duplicate connection")
                with self._ports_lock:
                    if self._port_key in self._ports:
                        raise OSError("this serial device already has an owner")
                    self._ports.add(self._port_key)
                    self._claimed = True
                context = self._context or multiprocessing.get_context("spawn")
                self._channel, child = context.Pipe()
                self._process = context.Process(target=_serial_worker, args=(child, self.config), daemon=True)
                try:
                    self._process.start()
                finally:
                    child.close()
            self._seq += 1
            self._commands.append(dict(operation=operation, t_ns=self.now_ns(), targets=targets, acknowledged=False))
            if operation in ("connect", "SetMotors", "Stop"):
                self.motor_commands_sent = True  # Worker might write before a response is lost.
            if self._deadline_ns() >= deadline:
                raise TimeoutError("motor request expired before dispatch")
            # One small request/response in flight; never enqueue camera images.
            self._channel.send((self._seq, operation, deadline, targets))
            remaining = max(0., (deadline-self._deadline_ns())/1e9)
            if not self._channel.poll(remaining):
                raise TimeoutError("motor worker/open/ACK deadline exceeded")
            reply = self._channel.recv()
            if self._deadline_ns() >= deadline:
                raise TimeoutError("late motor reply rejected")
            if not isinstance(reply, tuple) or len(reply) != 4 or reply[0] != self._seq:
                raise StopProtocolError("mismatched motor response")
            if reply[1] is not True:
                if reply[2] == "TIMEOUT":
                    raise TimeoutError(reply[3])
                if reply[2] == "PROTOCOL_ERROR":
                    raise StopProtocolError(reply[3])
                raise OSError(reply[3])
            if reply[2] != "OK" or reply[3] != "":
                raise StopProtocolError("inconsistent successful motor response")
            self._connected = True
            self.status = SerialStatus(self.clock_id, self.now_ns(), "OK")
            self._commands[-1]["acknowledged"] = True
            if targets is not None or operation == "connect":
                self.last_acknowledged_targets = targets or (0, 0, 0, 0)
            self.stop_acknowledged = operation in ("connect", "Stop")
            return self.status
        except (Exception, KeyboardInterrupt) as error:
            self._failed = True
            self.stop_acknowledged = False
            self.status = SerialStatus(self.clock_id, self.now_ns(), _error_state(error))
            self._abort()
            raise
        finally:
            self._lock.release()

    def _abort(self):
        self._connected = False
        if self._process is not None and self._process.pid is not None:
            if self._process.is_alive():
                self._process.terminate()
            self._process.join(timeout=.05)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=.05)
        if self._channel is not None:
            self._channel.close()
        dead = self._process is None or not self._process.is_alive()
        if dead and self._claimed:
            with self._ports_lock:
                self._ports.discard(self._port_key)
                self._claimed = False

    def close(self, *, owner):
        self._check_owner(owner)
        if self._closed:
            return
        try:
            if self._connected and not self._failed:
                self.Stop(owner=owner)
        finally:
            self._closed = True
            self._abort()
            self.status = SerialStatus(self.clock_id, self.now_ns(), "LOST")
            if self.serial_connection_opened:
                raise OSError("serial worker could not be reaped; ownership retained, use independent cutoff")
