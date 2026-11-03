"""RenoCS quadrillage controller: one camera, one motor owner, latched stops.

Repository root: python -m physical_robot.main --dry-run --scenario T --frames 16
Live-camera Pi test: python3 -m physical_robot.main --dry-run --live-camera --frames 300
--camera picamera2 is a compatible alias for --live-camera.
One source owns one Picamera2Frames instance; each step calls read exactly once.
640x480 matches the physical follower; RGB888 arrays are BGR. SensorTimestamp
uses CLOCK_BOOTTIME, shared with adapter, T.9 and mock link. Synthetic runs use
one explicit fake monotonic clock and are NOT physical/simulation validation.
Tracker event t_ns retains its legacy elapsed clock; event_clock_epoch_ns maps
it to clock_id. event_camera_t_ns is the corresponding physical capture time.

Only Controller calls the SetMotors/Stop interface. All other modules emit
observations/requests. SAFE_STOP > latched intersection > DEGRADED/line-lost pause
> PD movement. No continuation/rearm/recovery API; restart only while safely
stationary under operator control. Shutdown always attempts Stop before close.
Real mode latches line loss as SAFE_STOP; legacy dry-run pause behavior remains.

--enable-motors explicitly selects real A22 hardware. It additionally requires
--confirm-power-cutoff and explicit --base-pwm/--pwm-limit values. The operator
must test the independent motor-power cutoff and remain ready to use it. The
team confirmed professor A22; no claim is made about an installed watchdog.
Neither mock ACKs nor zero requests establish physical rest.
Telemetry: proposed_wheel_pwm is signed PWM, not measured speed. processing_ms
excludes camera waiting; cycle_ms includes capture, perception, policy and motor
requests. Dry-run serial health is MOCK_ONLY; the authorized A22 backend reports
bounded communication results, which still do not prove physical movement/rest.
"""
import argparse
from concurrent.futures import TimeoutError as CameraTimeoutError
from copy import deepcopy
import json
import math
import signal
import threading
import time

import cv2
import numpy as np

from intersections.fault_manager import FaultManager, SerialStatus
from intersections.physical_adapter import PhysicalIntersectionAdapter
from .bluno_backend import A22MotorBackend, BlunoConfig, HardwareAuthorization
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
    def __init__(self, source, *, motors=None, enable_real_motors=False,
                 base_pwm=100, pwm_limit=255, latch_line_loss=None):
        self.source = source
        self.clock_id, self.now_ns = source.clock_id, source.now_ns
        self.motors = motors if motors is not None else MockBluno(clock_id=self.clock_id, now_ns=self.now_ns)
        self.dry_run = type(self.motors) is MockBluno
        if not self.dry_run:
            if type(self.motors) is not A22MotorBackend:
                raise TypeError("only MockBluno or the gated A22 backend is supported")
            if enable_real_motors is not True:
                raise PermissionError("real motors disabled by default")
            self.motors.assert_authorized(movement=True)
        if self.motors.clock_id != self.clock_id:
            raise ValueError("camera and motor link must share one physical clock domain")
        self._owner = object()
        self.motors.bind(self._owner)
        self.adapter = PhysicalIntersectionAdapter(run_id="physical-main-dry-run" if self.dry_run else "physical-quadrillage",
                    clock_id=self.clock_id, now_ns=self.now_ns)
        self.faults = FaultManager(clock_id=self.clock_id, camera_timeout_ns=self.adapter.max_age_ns)
        self.follower = LineFollower(base_speed=base_pwm, max_speed=pwm_limit)
        self.latch_line_loss = not self.dry_run if latch_line_loss is None else latch_line_loss
        self.last_image = None
        self.seq = 0
        self._started = self._closed = False
        self._last_stop = None
        self._shutdown_result = None
        self._lock = threading.Lock()

    def start(self):
        if self._closed or self._started:
            raise RuntimeError("controller cannot be restarted")
        if not self.dry_run:
            # Establish a working, warmed-up camera before opening the motor port.
            self.source.start()
            self.motors.connect(owner=self._owner)
        self.motors.Stop(owner=self._owner)  # Initial zero before camera processing.
        if self.dry_run:
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
        self._last_stop = key  # Only after an acknowledgement; not physical rest.
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
                self.last_image = image
                acquisition_finished = time.perf_counter_ns()
                frame_received = True
                result = self.adapter.process_frame(image, pixel_format="BGR", captured_ns=captured_ns,
                                seq=self.seq, clock_id=self.source.clock_id)
                if result["frame_accepted"]:
                    line = self.follower.update(image)  # Same capture, no annotations/motor access.
                    if self.latch_line_loss and not line["found"]:
                        result = self.adapter.camera_failed("LINE_LOST")
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
            # Stop real hardware BEFORE probing/logging a known stop request.
            # Keep the subsequent health check: its failure outranks navigation.
            early_stop_ack = False
            if not self.dry_run and result["stop_requested"]:
                try:
                    early_stop_ack = self._stop("SAFETY" if result["stop_class"] == "SAFETY"
                                               else "INTERSECTION", result["stop_reason"])
                except OSError as failure:
                    error = str(failure)
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
            attempted_movement = False
            try:
                if priority == "LINE_FOLLOWING":
                    attempted_movement = True
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
                    actuation = "Stop" if sent or early_stop_ack else "HOLD_STOP"
                    stop_acknowledged = sent or early_stop_ack  # This cycle's ACK only.
            except (OSError, ValueError) as failure:
                error = str(failure)
                if isinstance(failure, ValueError):
                    result = self.adapter.camera_failed("MOTOR_COMMAND_INVALID")
                decision = self._health(result, motor_feedback)
                priority, reason = "SAFETY", decision["stop_reason"]
                actuation = "STOP_UNCONFIRMED"
                # A failed movement transaction still needs a best-effort stop.
                # A failed Stop is not retried blindly within the same cycle.
                if attempted_movement:
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
                        stop_acknowledged=stop_acknowledged, dry_run=self.dry_run, mock_motors=self.dry_run,
                        serial_connection_opened=(not self.dry_run and self.motors.serial_connection_opened),
                        motor_commands_sent=(not self.dry_run and self.motors.motor_commands_sent),
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
            if not self.dry_run:
                try:
                    self.motors.close(owner=self._owner)
                except Exception as failure:
                    result["errors"].append(str(failure))
            try:
                self.source.close()
                result["camera_closed"] = True
            except Exception as failure:
                result["errors"].append(str(failure))
        self._shutdown_result = result
        return deepcopy(result)

    def run(self, *, frames, on_result=print, max_seconds=None):
        exit_code, count = 0, 0
        exit_reason = "FRAME_LIMIT"
        try:
            self.start()
            deadline = None if max_seconds is None else time.monotonic()+max_seconds
            for _ in range(frames):
                if deadline is not None and time.monotonic() >= deadline:
                    exit_reason = "TIME_LIMIT"
                    break
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
            on_result(dict(error=str(failure), dry_run=self.dry_run, mock_motors=self.dry_run,
                           fault_state=decision["state"], stop_requested=True, decision=decision,
                           serial_connection_opened=(not self.dry_run and self.motors.serial_connection_opened),
                           motor_commands_sent=(not self.dry_run and self.motors.motor_commands_sent)))
            exit_code = 2
            exit_reason = "CONTROLLER_FAILURE"
        finally:
            shutdown = self.shutdown()
        return dict(frames=count, exit_code=2 if shutdown["errors"] else exit_code,
                    exit_reason=exit_reason, shutdown=shutdown, dry_run=self.dry_run, physical_stop_verified=False)


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
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="mock motors, never open serial")
    mode.add_argument("--enable-motors", action="store_true", help="REAL motor actuation; supervised test only")
    camera_options = parser.add_mutually_exclusive_group()
    camera_options.add_argument("--camera", choices=("synthetic", "picamera2"))
    camera_options.add_argument("--live-camera", dest="camera", action="store_const", const="picamera2")
    parser.add_argument("--scenario", choices=("line", "X", "T", "unknown"), default="T")
    parser.add_argument("--frames", type=int)
    parser.add_argument("--max-seconds", type=float, help="bounded run, default 30 s with real motors")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--base-pwm", type=int, help="explicit real-test base PWM, not measured speed")
    parser.add_argument("--pwm-limit", type=int, help="explicit real-test absolute PWM cap, at most 80")
    parser.add_argument("--confirm-power-cutoff", action="store_true",
                        help="operator confirms a tested independent MOTOR-POWER cutoff is in reach")
    parser.add_argument("--preview", action="store_true", help="show shared camera image; q requests shutdown")
    args = parser.parse_args(argv)
    real = args.enable_motors
    args.camera = args.camera or ("picamera2" if real else "synthetic")
    args.frames = args.frames if args.frames is not None else (1_000_000_000 if real else 60)
    if args.frames <= 0:
        parser.error("frames must be positive")
    if real:
        if args.camera != 'picamera2':
            parser.error('real motors require the actual Picamera2 source')
        if not args.confirm_power_cutoff:
            parser.error('test the independent motor-power cutoff, then explicitly confirm it')
        if args.base_pwm is None or args.pwm_limit is None or not 0 < args.base_pwm <= args.pwm_limit <= 80:
            parser.error('real test requires explicit 0 < --base-pwm <= --pwm-limit <= 80')
        if args.max_seconds is None:
            args.max_seconds = 30.
    elif args.confirm_power_cutoff:
        parser.error('--confirm-power-cutoff applies only to --enable-motors')
    if args.max_seconds is not None and (not math.isfinite(args.max_seconds) or not 0 < args.max_seconds <= 300):
        parser.error('max-seconds must be finite and in (0,300]')
    base_pwm = args.base_pwm if args.base_pwm is not None else 100
    pwm_limit = args.pwm_limit if args.pwm_limit is not None else 255
    if not 0 < base_pwm <= pwm_limit <= 255:
        parser.error('require 0 < base PWM <= PWM limit <= 255')
    source = controller = None
    # Signals unwind to the sole owner's shutdown; they never write serial.
    def terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM")
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        if args.camera == "picamera2":
            from intersections.raspberry_camera import Picamera2Frames
            source = Picamera2Frames(size=(640,480))  # Exact huit preview defaults; no FPS override.
        else:
            source = SyntheticFrames(args.scenario)
        motors = None
        if real:
            authorization = HardwareAuthorization('', '', protocol_confirmed=True,
                independent_stop_verified=True,
                evidence_reference='Team confirmed professor A22; operator explicitly confirms tested power cutoff',
                protocol_profile='professor_A22')
            motors = A22MotorBackend(clock_id=source.clock_id, now_ns=source.now_ns,
                config=BlunoConfig(port=args.port, pwm_limit=pwm_limit),
                authorization=authorization, allow_hardware=True)
        controller = Controller(source, motors=motors, enable_real_motors=real,
                                base_pwm=base_pwm, pwm_limit=pwm_limit)
        def output(result):
            print(json.dumps(result), flush=True)
            if args.preview and controller.last_image is not None:
                shown = controller.last_image.copy()
                text = '{} {} {}'.format(result.get('intersection_kind','UNKNOWN'),
                       result.get('priority',result.get('fault_state','')),result.get('actuation',''))
                cv2.putText(shown,text,(8,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,255),2)
                cv2.imshow('RenoCS quadrillage - q: Stop/exit; physical cutoff for emergency',shown)
                if cv2.waitKey(1) & 255 == ord('q'):
                    raise KeyboardInterrupt('operator q')
        summary = controller.run(frames=args.frames, on_result=output, max_seconds=args.max_seconds)
        summary.update(camera_mode=args.camera,
                       serial_connection_opened=(real and controller.motors.serial_connection_opened),
                       motor_commands_sent=(real and controller.motors.motor_commands_sent),
                       camera_pixel_order="BGR", clock_id=source.clock_id,
                       camera_configuration=getattr(source,"configuration",None),
                       base_pwm=base_pwm, pwm_limit=pwm_limit)
        print(json.dumps(summary,default=str),flush=True)
        return summary['exit_code']
    except Exception as failure:
        errors = [str(failure)]
        if controller is not None:
            errors += controller.shutdown()['errors']
        elif source is not None:
            try:
                source.close()
            except Exception as error:
                errors.append(str(error))
        print(json.dumps(dict(error='; '.join(errors),dry_run=not real,mock_motors=not real,
                              serial_connection_opened=(real and controller is not None
                                                         and controller.motors.serial_connection_opened),
                              motor_commands_sent=(real and controller is not None
                                                    and controller.motors.motor_commands_sent),
                              physical_stop_verified=False)),flush=True)
        return 2
    finally:
        signal.signal(signal.SIGTERM,previous)
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
