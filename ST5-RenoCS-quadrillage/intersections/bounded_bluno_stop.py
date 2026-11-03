"""Optional bounded bridge to the UNCHANGED professor arduino.Stop() API.

No serial imports, connection, handshake or calls on construction/import. Only
an explicit Stop()/probe() touches an ALREADY connected car (115200, A22).
probe() uses the firmware's existing B/empty command and OK reply, no motor data.
The single serial owner must gate all other writers; this temporarily substitutes
car.serialport. Other legacy calls/connect remain unbounded and must not be used
in a moving robot's control loop without their own bounded integration.

A total monotonic budget bounds repeated ACK waits and each pySerial read/write.
This is not a hard real-time bound on OS/USB drivers or a hardware watchdog.
After any failed transaction the wrapper refuses further commands: reconnect /
reinitialize the link explicitly before constructing a replacement. A torn binary
command must not be followed blindly by another command.
"""
import math
import threading
import time

from .fault_manager import SerialStatus


class StopProtocolError(OSError):
    pass


class _DeadlinePort:
    EXPECTED = b"C" + bytes(8)
    ACK = b"OK Moteurs mis aux consignes : 0 0 0 0"

    def __init__(self, port, now_ns, deadline, *, probe=False):
        self.port, self.now_ns, self.deadline = port, now_ns, deadline
        self.expected = b"B" if probe else self.EXPECTED
        self.ack = b"OK" if probe else self.ACK
        self.last_ns = now_ns()
        self.sent = 0
        self.acknowledged = False

    def remaining(self):
        now = self.now_ns()
        if type(now) is not int or now < self.last_ns:
            raise StopProtocolError("invalid serial deadline clock")
        self.last_ns = now
        remaining = (self.deadline-now)/1e9
        if remaining <= 0:
            raise TimeoutError("Bluno stop transaction deadline exceeded")
        return remaining

    def write(self, data):
        data = bytes(data)
        if not data or self.expected[self.sent:self.sent+len(data)] != data:
            raise StopProtocolError("legacy Stop produced an unexpected command")
        self.port.write_timeout = self.remaining()
        count = self.port.write(data)
        self.remaining()
        if count != len(data):
            raise OSError("partial Bluno write: link requires reinitialization")
        self.sent += count
        return count

    def readline(self):
        if self.sent != len(self.expected):
            raise StopProtocolError("ACK requested before complete zero command")
        line = bytearray()
        while len(line) < 128:
            # Read ONE byte per budgeted call: readline() can repeatedly apply
            # a timeout per byte and overrun a total transaction budget.
            self.port.timeout = self.remaining()
            value = self.port.read(1)
            self.remaining()
            if value:
                line.extend(value)
                if value == b"\n":
                    if bytes(line).strip() != self.ack:
                        raise StopProtocolError("unexpected or negative Bluno stop ACK")
                    self.acknowledged = True
                    return bytes(line)
        raise StopProtocolError("overlong Bluno ACK")


class BoundedBlunoStop:
    """Pass to adapter.apply_stop_request only in an authorized hardware test.

    status can feed FaultManager (same clock_id/now_ns as the camera adapter).
    stop_acknowledged means the expected text reply arrived, NOT physical rest.
    Firmware has no transaction IDs: even matching ACKs cannot prove freshness
    against arbitrarily delayed previous replies. Single-owner ordering required.
    """
    def __init__(self, connected_car, *, clock_id, now_ns=None, timeout_s=.3):
        if not isinstance(clock_id, str) or not clock_id:
            raise ValueError("declare a monotonic physical clock")
        if clock_id != "HOST_MONOTONIC" and now_ns is None:
            raise ValueError("supply the clock matching clock_id")
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or not 0 < timeout_s <= 5):
            raise ValueError("timeout_s must be finite and in (0,5]")
        self.car = connected_car
        self.clock_id = clock_id
        self.now_ns = now_ns or time.monotonic_ns
        self.timeout_ns = round(timeout_s*1e9)
        initial = self.now_ns()
        if type(initial) is not int or initial < 0:
            raise ValueError("clock must supply nonnegative integer nanoseconds")
        self.status = SerialStatus(clock_id, initial, "UNVERIFIED")
        self.stop_acknowledged = False
        self.physical_stop_verified = False
        self._failed = False
        self._lock = threading.Lock()

    def Stop(self):
        return self._transaction(probe=False)

    def probe(self):
        """Explicit bounded B/OK liveness transaction; opens no port, no motion.

        Future main controller must schedule probes at an appropriate cadence;
        this class starts no background thread. is_open alone proves no liveness.
        """
        return self._transaction(probe=True)

    def _transaction(self, *, probe):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("concurrent serial operation forbidden")
        original = None
        saved = None
        self.stop_acknowledged = False
        try:
            if self._failed:
                raise StopProtocolError("failed link requires explicit reinitialization")
            original = self.car.serialport
            proxy = _DeadlinePort(original, self.now_ns, self.now_ns()+self.timeout_ns, probe=probe)
            saved = (original.timeout, original.write_timeout)
            if not original.is_open:
                raise OSError("Bluno serial port is closed")
            # Do not flush/discard unread replies or perform an unbounded drain.
            # Unexpected pending input requires owner-led resynchronization.
            if original.in_waiting:
                raise StopProtocolError("unconsumed serial input; resynchronize before stop test")
            proxy.remaining()
            self.car.serialport = proxy
            if probe:
                # Commandes.cpp empty()/B, already implemented by the professors.
                proxy.write(b"B")
                proxy.readline()
            else:
                self.car.Stop()  # Exact legacy API/protocol, now with bounded I/O.
            if not proxy.acknowledged:
                raise StopProtocolError("legacy Stop returned without a validated ACK")
            self.status = SerialStatus(self.clock_id, self.now_ns(), "OK")
            self.stop_acknowledged = not probe
            return self.status
        except Exception as error:
            self._record_failure(error)
            raise
        finally:
            try:
                if original is not None:
                    self.car.serialport = original
                if saved is not None:
                    original.timeout, original.write_timeout = saved
            except Exception as error:
                self._record_failure(error)
                raise
            finally:
                self._lock.release()

    def _record_failure(self, error):
        self._failed = True
        self.stop_acknowledged = False
        kind = ("TIMEOUT" if isinstance(error, TimeoutError)
                or type(error).__name__ == "SerialTimeoutException" else
                "PROTOCOL_ERROR" if isinstance(error, StopProtocolError) else "LOST")
        self.status = SerialStatus(self.clock_id, self.now_ns(), kind)
