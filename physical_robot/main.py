"""Shared-frame physical control core, DRY-RUN MOTOR BACKEND ONLY.

Repository root: python -m physical_robot.main --dry-run --scenario T --frames 16
Live-camera Pi test: python3 -m physical_robot.main --dry-run --live-camera --frames 300
--camera picamera2 is a compatible alias (still MOCK motors, no serial).
One source owns one Picamera2Frames instance; each step calls read exactly once.
640x480 matches the physical follower; RGB888 arrays are BGR. SensorTimestamp
uses CLOCK_BOOTTIME, shared with adapter, T.9 and mock link. Synthetic runs use
one explicit fake monotonic clock and are NOT physical/simulation validation.
Tracker event t_ns retains its legacy elapsed clock; event_clock_epoch_ns maps
it to clock_id. event_camera_t_ns is the corresponding physical capture time.

Only Controller calls the mock SetMotors/Stop interface. All other modules emit
observations/requests. SAFE_STOP > latched intersection > DEGRADED/line-lost pause
> PD movement. No continuation/rearm/recovery API; restart only while safely
stationary under operator control. Shutdown always attempts Stop before close.

No serial/arduino/teammate controller imports and NO real motor option. A future
motor backend requires a bounded handshake AND SetMotors, a single ACK mode
(T.9 A22 differs from MotorBridge A12), verified watchdog/cutoff and wheel signs.
BoundedBlunoStop alone cannot bound the current SetMotors or connect calls.
Neither mock ACKs nor zero requests establish physical rest.
Telemetry: proposed_wheel_pwm is signed PWM, not measured speed. processing_ms
excludes camera waiting; cycle_ms includes capture, perception, policy and mock
requests. Serial health in decision is MOCK_ONLY, never a real Bluno check.
"""
import argparse
from concurrent.futures import TimeoutError as CameraTimeoutError
from copy import deepcopy
import json
import signal
import threading
import time

import cv2
import numpy as np

from intersections.fault_manager import FaultManager, SerialStatus
from intersections.physical_adapter import PhysicalIntersectionAdapter
from .line_following import LineFollower


class MockBluno:
    """Actual SetMotors(left,right) conventions, memory-only; no serial code.

    One owner capability is bound once. Fault injection maps probe/SetMotors/Stop
    to TIMEOUT/LOST/PROTOCOL_ERROR. status is explicitly mock transaction health.
    """
    def __init__(self, *, clock_id, now_ns):
        self.clock_id, self.now_ns = clock_id, now_ns
        self.status = SerialStatus(clock_id, now_ns(), "UNVERIFIED")
        self.failures = {}
        self._owner = None
        self._commands = []
        self.last_acknowledged_targets = None

    @property
    def commands(self):
        return deepcopy(self._commands)

    def bind(self, owner):
        if self._owner is not None:
            raise RuntimeError("motor transport already has an owner")
        self._owner = owner

    def _call(self, operation, owner, targets=None):
        if owner is not self._owner or self._owner is None:
            raise PermissionError("only the bound controller may issue motor commands")
        failure = self.failures.get(operation)
        self.status = SerialStatus(self.clock_id, self.now_ns(), failure or "OK")
        record = dict(operation=operation, t_ns=self.now_ns(), targets=targets,
                      acknowledged=not bool(failure), mock=True)
        self._commands.append(record)
        if failure:
            if failure == "TIMEOUT":
                raise TimeoutError("injected mock communication timeout")
            raise OSError("injected mock communication failure: " + failure)
        if targets is not None:
            self.last_acknowledged_targets = targets
        return self.status

    def probe(self, *, owner):
        return self._call("probe", owner)

    def SetMotors(self, left, right, *, owner):
        left, right = int(left), int(right)  # Same truncation as arduino.SetMotors.
        if max(abs(left), abs(right)) > 255:
            raise ValueError("PWM outside the professor's signed range")
        return self._call("SetMotors", owner, (right, left, right, left))

    def Stop(self, *, owner):
        return self._call("Stop", owner, (0, 0, 0, 0))


class Controller:
    def __init__(self, source, *, motors=None):
        self.source = source
        self.clock_id, self.now_ns = source.clock_id, source.now_ns
        self.motors = motors if motors is not None else MockBluno(clock_id=self.clock_id, now_ns=self.now_ns)
        if type(self.motors) is not MockBluno:
            raise TypeError("this dry-run controller accepts ONLY MockBluno, never real hardware")
        if self.motors.clock_id != self.clock_id:
            raise ValueError("camera and mock link must share one physical clock domain")
        self._owner = object()
        self.motors.bind(self._owner)
        self.adapter = PhysicalIntersectionAdapter(run_id="physical-main-dry-run",
                    clock_id=self.clock_id, now_ns=self.now_ns)
        self.faults = FaultManager(clock_id=self.clock_id, camera_timeout_ns=self.adapter.max_age_ns)
        self.follower = LineFollower()
        self.seq = 0
        self._started = self._closed = False
        self._last_stop = None
        self._shutdown_result = None
        self._lock = threading.Lock()

    def start(self):
        if self._closed or self._started:
            raise RuntimeError("controller cannot be restarted")
        self.motors.Stop(owner=self._owner)  # Initial zero, mock only.
        self.source.start()
        self._started = True

    def _health(self, result, motor_feedback):
        return self.faults.update(result, serial_status=self.motors.status,
                    motor_feedback=motor_feedback, now_ns=self.now_ns(), clock_id=self.clock_id)

    def _stop(self, priority, reason):
        key = (priority, reason)
        if key == self._last_stop:
            return False
        self.motors.Stop(owner=self._owner)
        self._last_stop = key  # Only after a successful MOCK acknowledgement.
        return True

    def step(self, *, motor_feedback=None):
        if not self._started or self._closed:
            raise RuntimeError("start one controller session before stepping")
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("concurrent/reentrant control cycle forbidden")
        try:
            if self._closed:
                raise RuntimeError("controller is shut down")
            cycle_started = time.perf_counter_ns()
            acquisition_finished = cycle_started
            self.seq += 1
            image = None
            line = None
            error = None
            frame_received = False
            try:
                image, captured_ns, _ = self.source.read(timeout=.25)
                acquisition_finished = time.perf_counter_ns()
                frame_received = True
                result = self.adapter.process_frame(image, pixel_format="BGR", captured_ns=captured_ns,
                                seq=self.seq, clock_id=self.source.clock_id)
                if result["frame_accepted"]:
                    line = self.follower.update(image)  # Same capture, no annotations/motor access.
            except (TimeoutError, CameraTimeoutError) as failure:
                if not frame_received:
                    acquisition_finished = time.perf_counter_ns()
                error = str(failure)
                result = self.adapter.camera_failed("CAMERA_OR_PROCESSING_FAILURE" if frame_received else "CAMERA_TIMEOUT")
            except Exception as failure:
                if not frame_received:
                    acquisition_finished = time.perf_counter_ns()
                error = str(failure)
                result = self.adapter.camera_failed("CAMERA_OR_PROCESSING_FAILURE")
            # Do not delay a known camera fault to perform a health probe.
            if result["stop_class"] != "SAFETY":
                try:
                    self.motors.probe(owner=self._owner)
                except OSError as failure:
                    error = str(failure)
            decision = self._health(result, motor_feedback)  # Freshness AFTER all processing.
            if decision["state"] == "SAFE_STOP":
                priority, reason = "SAFETY", decision["stop_reason"]
            elif result["stop_requested"]:
                priority, reason = "INTERSECTION", result["stop_reason"]
            elif not decision["motion_allowed"]:
                priority, reason = "DEGRADED", decision["stop_reason"]
            elif not line or not line["found"]:
                priority, reason = "LINE_LOST", "LINE_NOT_FOUND"
            else:
                priority, reason = "LINE_FOLLOWING", None
            stop_acknowledged = False
            actuation = None
            try:
                if priority == "LINE_FOLLOWING":
                    self.motors.SetMotors(line["left_pwm"], line["right_pwm"], owner=self._owner)
                    self._last_stop = None
                    actuation = "SetMotors"
                    # A transaction must not leave an over-age camera decision
                    # authorizing movement after it returns.
                    decision = self._health(result, motor_feedback)
                    if decision["state"] == "SAFE_STOP":
                        priority, reason = "SAFETY", decision["stop_reason"]
                        stop_acknowledged = self._stop(priority, reason)
                        actuation = "Stop"
                else:
                    sent = self._stop(priority, reason)
                    actuation = "Stop" if sent else "HOLD_STOP"
                    stop_acknowledged = sent  # This cycle's MOCK ACK only, not a historical ACK.
            except OSError as failure:
                error = str(failure)
                decision = self._health(result, motor_feedback)
                priority, reason = "SAFETY", decision["stop_reason"]
                actuation = "STOP_UNCONFIRMED"
                # A failed movement transaction still needs a best-effort stop.
                # A failed Stop is not retried blindly within the same cycle.
                if self.motors.commands[-1]["operation"] == "SetMotors":
                    try:
                        self._stop(priority, reason)
                        stop_acknowledged = True
                        actuation = "Stop"
                    except OSError:
                        decision = self._health(result, motor_feedback)
            finished = time.perf_counter_ns()
            observation = result["observation"] or {}
            return dict(seq=self.seq, captured_ns=result["captured_ns"], clock_id=self.clock_id,
                        intersection_kind=observation.get("kind", "UNKNOWN"),
                        proposed_wheel_pwm=({"left":line["left_pwm"], "right":line["right_pwm"]}
                                            if line is not None else None),
                        stop_requested=priority != "LINE_FOLLOWING", fault_state=decision["state"],
                        processing_ms=(finished-acquisition_finished)/1e6,
                        capture_wait_ms=(acquisition_finished-cycle_started)/1e6,
                        cycle_ms=(finished-cycle_started)/1e6,
                        observation=result["observation"], event=result["confirmed_event"],
                        event_clock_epoch_ns=result["epoch_ns"],
                        event_camera_t_ns=(result["epoch_ns"]+result["confirmed_event"]["t_ns"]
                                           if result["confirmed_event"] else None),
                        detector_pipeline_ms=result["processing_ms"],
                        frame_age_ms=((self.now_ns()-result["captured_ns"])/1e6
                                      if result["captured_ns"] is not None else None),
                        confirmed_kind=result["confirmed_kind"], decision=decision, line=line,
                        priority=priority, reason=reason, actuation=actuation, error=error,
                        stop_acknowledged=stop_acknowledged, dry_run=True, mock_motors=True,
                        serial_connection_opened=False, motor_commands_sent=False,
                        last_acknowledged_targets=self.motors.last_acknowledged_targets,
                        physical_stop_verified=False)
        finally:
            self._lock.release()

    def shutdown(self):
        with self._lock:
            return self._shutdown()

    def _shutdown(self):
        if self._closed:
            return deepcopy(self._shutdown_result)
        self._closed = True
        result = dict(stop_acknowledged=False, camera_closed=False, errors=[], physical_stop_verified=False)
        try:
            self.motors.Stop(owner=self._owner)
            result["stop_acknowledged"] = True
        except Exception as failure:
            result["errors"].append(str(failure))
        finally:
            try:
                self.source.close()
                result["camera_closed"] = True
            except Exception as failure:
                result["errors"].append(str(failure))
        self._shutdown_result = result
        return deepcopy(result)

    def run(self, *, frames, on_result=print):
        exit_code, count = 0, 0
        exit_reason = "FRAME_LIMIT"
        try:
            self.start()
            for _ in range(frames):
                result = self.step()
                count += 1
                on_result(result)
                if result["decision"]["state"] == "SAFE_STOP":
                    exit_code = 2
                    exit_reason = "SAFE_STOP"
                    break
        except KeyboardInterrupt as interrupt:
            exit_reason = "INTERRUPTED" + (": " + str(interrupt) if str(interrupt) else "")
        except Exception as failure:
            fault = self.adapter.camera_failed("CONTROLLER_STARTUP_OR_LOOP_FAILURE")
            decision = self._health(fault, None)
            on_result(dict(error=str(failure), dry_run=True, mock_motors=True,
                           fault_state=decision["state"], stop_requested=True, decision=decision,
                           serial_connection_opened=False, motor_commands_sent=False))
            exit_code = 2
            exit_reason = "CONTROLLER_FAILURE"
        finally:
            shutdown = self.shutdown()
        return dict(frames=count, exit_code=2 if shutdown["errors"] else exit_code,
                    exit_reason=exit_reason, shutdown=shutdown, dry_run=True, physical_stop_verified=False)


class SyntheticFrames:
    """Synthetic software stimuli only, not a captured/validated camera dataset."""
    clock_id = "DRY_RUN_CLOCK"

    def __init__(self, scenario):
        self.scenario, self.count = scenario, 0
        self.t_ns = 1_000_000_000

    def now_ns(self):
        return self.t_ns

    def start(self):
        pass

    def read(self, timeout=.25):
        self.count += 1
        self.t_ns += 64_000_000
        image = np.zeros((480, 640, 3), np.uint8)
        if self.scenario == "line" or self.count <= 5:
            cv2.line(image, (320, 0), (320, 479), (235,235,235), 8)
        elif self.scenario in ("X", "T"):
            for end in [(80,200),(560,200),(320,470)] + ([(320,10)] if self.scenario == "X" else []):
                cv2.line(image, (320,200), end, (235,235,235), 8)
        return image, self.t_ns, {"source": "SYNTHETIC"}

    def close(self):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", required=True)
    camera_options = parser.add_mutually_exclusive_group()
    camera_options.add_argument("--camera", choices=("synthetic", "picamera2"))
    camera_options.add_argument("--live-camera", dest="camera", action="store_const", const="picamera2",
                                help="real Pi camera, dry-run decisions only; no serial/motor actuation")
    parser.set_defaults(camera="synthetic")
    parser.add_argument("--scenario", choices=("line", "X", "T", "unknown"), default="T")
    parser.add_argument("--frames", type=int, default=60)
    args = parser.parse_args(argv)
    if args.frames <= 0:
        parser.error("frames must be positive")
    source = None
    try:
        if args.camera == "picamera2":
            from intersections.raspberry_camera import Picamera2Frames
            source = Picamera2Frames(size=(640,480), fps=15.)  # Exactly one instance.
        else:
            source = SyntheticFrames(args.scenario)
        controller = Controller(source)
    except Exception as failure:
        if source is not None:
            source.close()
        print(json.dumps(dict(error=str(failure), dry_run=True, mock_motors=True,
                              serial_connection_opened=False, motor_commands_sent=False,
                              physical_stop_verified=False)), flush=True)
        return 2
    # Handler only requests shutdown by unwinding to run's finally. No competing
    # motor writer in the signal handler. SIGKILL/power loss require the watchdog.
    def terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM")
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        summary = controller.run(frames=args.frames,
                        on_result=lambda result: print(json.dumps(result), flush=True))
    finally:
        signal.signal(signal.SIGTERM, previous)
    summary.update(camera_mode=args.camera, serial_connection_opened=False, motor_commands_sent=False,
                   camera_pixel_order="BGR", clock_id=source.clock_id,
                   camera_configuration=getattr(source, "configuration", None))
    print(json.dumps(summary, default=str), flush=True)
    return summary["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
