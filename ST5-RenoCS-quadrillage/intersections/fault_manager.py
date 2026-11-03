"""T.9 decision policy, no hardware, OpenCV, timers or motor I/O on import.

One owner calls update every control tick, including camera/serial failures.
Feed PhysicalIntersectionAdapter.process_frame/check_frame_timeout results:
the adapter remains responsible for UNKNOWN persistence and camera processing.
This module checks envelope freshness, combines faults and gates navigation.
All timestamps must use the SAME declared monotonic physical clock.

NORMAL permits movement only without a navigation stop. DEGRADED pauses movement
but recovers on healthy inputs. SAFE_STOP latches until recover succeeds after
fault correction, fresh healthy inputs, operator authorization and independently
verified rest. A latched adapter fault additionally requires an explicit repaired
adapter session; this module never resets it. No automatic restart/rearming.

Integration (single camera/serial/motor owner):
  decision = faults.update(adapter_result, serial_status=link_status,
                           now_ns=clock(), clock_id=adapter.clock_id)
  # Gate EVERY navigation/motor writer with decision['motion_allowed'].
  # A stop_requested decision is only a request, never evidence of actual rest.
  # The owner must explicitly attempt bounded Stop on a T.9 stop transition:
  # adapter.apply_stop_request knows only the adapter's own T.2/T.5 stop latch.
  # Merely suppressing new PWM commands does not cancel the previous targets.
SerialStatus.OK must come from a completed, bounded real transaction, not from
Serial.is_open. MotorFeedback.reliable requires a separately verified feedback
source; the supplied GetSpeed implementation does not establish that reliability.
"""
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum


class FaultState(str, Enum):
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    SAFE_STOP = "SAFE_STOP"


@dataclass(frozen=True)
class SerialStatus:
    clock_id: str
    checked_ns: int
    state: str  # OK, UNVERIFIED, LOST, TIMEOUT or PROTOCOL_ERROR


@dataclass(frozen=True)
class MotorFeedback:
    clock_id: str
    checked_ns: int
    reliable: bool
    fault: bool  # A reported fault, not an estimate from requested PWM.


def _ns(value):
    return type(value) is int and value >= 0


class FaultManager:
    def __init__(self, *, clock_id, camera_timeout_ns=250_000_000,
                 serial_timeout_ns=500_000_000, motor_timeout_ns=500_000_000,
                 require_motor_feedback=False):
        if not isinstance(clock_id, str) or not clock_id:
            raise ValueError("declare the physical clock domain")
        if any(not _ns(v) or v == 0 for v in
               (camera_timeout_ns, serial_timeout_ns, motor_timeout_ns)):
            raise ValueError("timeouts must be positive integer nanoseconds")
        if type(require_motor_feedback) is not bool:
            raise ValueError("require_motor_feedback must be boolean")
        self.clock_id = clock_id
        self.camera_timeout_ns = camera_timeout_ns
        self.serial_timeout_ns = serial_timeout_ns
        self.motor_timeout_ns = motor_timeout_ns
        self.require_motor_feedback = require_motor_feedback
        self._last_ns = self._started_ns = None
        self._latched = []
        self._active = []
        self._degraded = ["NOT_INITIALIZED"]
        self._navigation_stop = False
        self._serial_verified = self._motor_verified = False
        self._inputs = None

    @property
    def state(self):
        return (FaultState.SAFE_STOP if self._latched else
                FaultState.DEGRADED if self._degraded else FaultState.NORMAL)

    def _fresh(self, t_ns, now_ns, limit):
        return _ns(t_ns) and 0 <= now_ns-t_ns <= limit

    def update(self, adapter_result, *, serial_status, now_ns, clock_id,
               motor_feedback=None, navigation_stop=False):
        """Update every tick; do not cache an old motion_allowed decision.

        camera_timeout_ns should equal adapter.max_age_ns. Other deadlines are
        provisional controller budgets, not measured hardware safety guarantees.
        Missing reliable motor feedback is reported, not a fabricated motor fault;
        set require_motor_feedback=True only when that source is commissioned.
        """
        faults, degraded = [], []
        self._inputs = deepcopy((adapter_result, serial_status, motor_feedback, navigation_stop))
        if (clock_id != self.clock_id or not _ns(now_ns)
                or (self._last_ns is not None and now_ns < self._last_ns)):
            faults.append("CONTROL_CLOCK_INVALID")
            now_ns = self._last_ns if self._last_ns is not None else 0
        self._last_ns = now_ns
        if self._started_ns is None:
            self._started_ns = now_ns
        if type(navigation_stop) is not bool:
            faults.append("INVALID_NAVIGATION_STOP")
        self._navigation_stop = navigation_stop is True

        if (not isinstance(adapter_result, dict)
                or type(adapter_result.get("stop_requested")) is not bool
                or adapter_result.get("stop_class") not in (None, "NAVIGATION", "SAFETY")):
            faults.append("CAMERA_STATUS_INVALID")
        elif adapter_result.get("clock_id") != self.clock_id:
            faults.append("CAMERA_CLOCK_MISMATCH")
        elif adapter_result["stop_class"] == "SAFETY":
            reason = adapter_result.get("stop_reason")
            faults.append(reason if isinstance(reason, str) and reason else "CAMERA_FAILURE")
        else:
            self._navigation_stop |= adapter_result["stop_requested"]
            captured = adapter_result.get("captured_ns")
            obs = adapter_result.get("observation")
            if obs is None and captured is None:
                if now_ns-self._started_ns > self.camera_timeout_ns:
                    faults.append("CAMERA_FRAME_TIMEOUT")
                else:
                    degraded.append("CAMERA_STARTING")
            elif not self._fresh(captured, now_ns, self.camera_timeout_ns):
                faults.append("CAMERA_STATUS_STALE_OR_INVALID")
            elif adapter_result.get("frame_accepted") is not True or not isinstance(obs, dict):
                faults.append("CAMERA_STATUS_INVALID")
            elif obs.get("valid") is False and obs.get("kind") == "UNKNOWN":
                degraded.append("UNKNOWN_PENDING")  # Adapter owns the 2 s escalation.
            elif obs.get("valid") is not True or obs.get("kind") not in ("NONE", "X", "T"):
                faults.append("CAMERA_OBSERVATION_INVALID")

        self._serial_verified = False
        if not isinstance(serial_status, SerialStatus):
            faults.append("SERIAL_UNVERIFIED")
        elif serial_status.clock_id != self.clock_id:
            faults.append("SERIAL_CLOCK_MISMATCH")
        elif not self._fresh(serial_status.checked_ns, now_ns, self.serial_timeout_ns):
            faults.append("SERIAL_STATUS_STALE_OR_INVALID")
        elif not isinstance(serial_status.state, str):
            faults.append("SERIAL_STATUS_INVALID")
        elif serial_status.state == "OK":
            self._serial_verified = True
        else:
            faults.append({"UNVERIFIED": "SERIAL_UNVERIFIED", "LOST": "SERIAL_LOST",
                           "TIMEOUT": "SERIAL_TIMEOUT", "PROTOCOL_ERROR": "SERIAL_PROTOCOL_ERROR"}
                          .get(serial_status.state, "SERIAL_STATUS_INVALID"))

        self._motor_verified = (
            isinstance(motor_feedback, MotorFeedback) and motor_feedback.reliable is True
            and type(motor_feedback.fault) is bool and motor_feedback.clock_id == self.clock_id
            and self._fresh(motor_feedback.checked_ns, now_ns, self.motor_timeout_ns))
        if self._motor_verified and motor_feedback.fault:
            faults.append("REPORTED_MOTOR_FAILURE")
        elif (self.require_motor_feedback or "REPORTED_MOTOR_FAILURE" in self._latched) and not self._motor_verified:
            faults.append("MOTOR_FEEDBACK_UNAVAILABLE")

        self._active, self._degraded = list(dict.fromkeys(faults)), degraded
        for reason in self._active:
            if reason not in self._latched:
                self._latched.append(reason)
        return self.snapshot()

    def snapshot(self):
        fault_stop = self.state != FaultState.NORMAL
        return dict(state=self.state.value, active_faults=list(self._active),
                    latched_faults=list(self._latched), degraded_reasons=list(self._degraded),
                    stop_reason=(self._latched[0] if self._latched else self._degraded[0]
                                 if self._degraded else "NAVIGATION_STOP" if self._navigation_stop else None),
                    stop_requested=bool(fault_stop or self._navigation_stop),
                    stop_class="SAFETY" if fault_stop else "NAVIGATION" if self._navigation_stop else None,
                    stop_latched=self.state == FaultState.SAFE_STOP,
                    navigation_stop_requested=self._navigation_stop,
                    motion_allowed=not fault_stop and not self._navigation_stop,
                    serial_verified=self._serial_verified,
                    motor_feedback_verified=self._motor_verified,
                    physical_stop_verified=False, stop_request_only=True)

    def recover(self, *, operator_authorized, rest_verified, now_ns, clock_id):
        """Explicit authorization plus independently verified rest; no motor I/O.

        Refresh/revalidate health at recovery time. rest_verified is a trusted
        external assertion from a controlled test/sensor, never a serial ACK or
        zero command. A healthy update alone cannot release SAFE_STOP.
        """
        if self._inputs is None:
            return False
        adapter, serial, motor, nav = self._inputs
        self.update(adapter, serial_status=serial, motor_feedback=motor, navigation_stop=nav,
                    now_ns=now_ns, clock_id=clock_id)
        if (operator_authorized is not True or rest_verified is not True
                or self._active or self._degraded):
            return False
        self._latched.clear()
        return True
