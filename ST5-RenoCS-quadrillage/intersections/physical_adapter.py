"""Physical camera -> unchanged X/T pipeline -> stop REQUEST; no hardware on import.

Construct once per camera session. Supply owned uint8 frames with an explicit
channel order and capture timestamps from the SAME declared clock as now_ns.
The frozen pipeline's legacy SIM_TIME label is internal: this adapter maps the
physical clock to one isolated elapsed-nanosecond run clock, preserving intervals.
No detector/tracker reset, automatic rearming, navigation decision or restart.

The robot's sole motor owner must gate ALL movement while stop_requested is true.
apply_stop_request is an explicit optional bridge to an ALREADY connected
professor arduino object. Nothing here opens serial, connects or sends movement.
"""
import time

import cv2
import numpy as np

from .intersection_pipeline import IntersectionPipeline


def to_bgr(image, pixel_format):
    """Explicit channel conversion, never infer RGB/BGR from array shape."""
    channels = {"BGR": 3, "RGB": 3, "BGRA": 4, "RGBA": 4}
    if (pixel_format not in channels or not isinstance(image, np.ndarray)
            or image.dtype != np.uint8 or image.ndim != 3
            or image.shape[2] != channels[pixel_format] or min(image.shape[:2]) < 2):
        raise ValueError("expected uint8 camera frame with declared BGR/RGB/BGRA/RGBA layout")
    if pixel_format == "BGR":
        return np.ascontiguousarray(image).copy()
    return cv2.cvtColor(image, {"RGB": cv2.COLOR_RGB2BGR,
                               "BGRA": cv2.COLOR_BGRA2BGR,
                               "RGBA": cv2.COLOR_RGBA2BGR}[pixel_format])


class PhysicalIntersectionAdapter:
    """Single-owner API. Camera-only use never calls apply_stop_request.

    process_frame returns the unchanged observation plus event/stop classification.
    A camera failure can escalate a navigation stop to a permanent SAFETY stop.
    Rest cannot be verified here: motion_verified is always False. Navigation must
    obtain independent rest evidence before executing LEFT/RIGHT/STRAIGHT.
    """
    def __init__(self, *, run_id, clock_id="HOST_MONOTONIC", now_ns=None):
        if not isinstance(clock_id, str) or not clock_id:
            raise ValueError("declare one physical clock domain")
        if clock_id != "HOST_MONOTONIC" and now_ns is None:
            raise ValueError("a custom clock requires its matching now_ns callable")
        self.clock_id = clock_id
        self.now_ns = now_ns or time.monotonic_ns
        self.pipeline = IntersectionPipeline(run_id=run_id)
        self.max_age_ns = self.pipeline.tracker.config.max_observation_age_ns
        self.epoch_ns = None
        self.created_ns = self.now_ns()
        if type(self.created_ns) is not int or self.created_ns < 0:
            raise ValueError("physical clock must return nonnegative integer nanoseconds")
        self.last_now_ns = self.created_ns
        self.last_capture_ns = None
        self.unknown_since_ns = None
        self.safety_reason = None
        self.confirmed_kind = None
        self._delivered = False
        self._pending_action = False
        self._latest = dict(observation=None, tracker=None, stop_requested=False,
                            stop_action=False, stop_reason=None, stop_t_ns=None)
        self.processing_ms = None
        self.input_age_ms = None

    def _fault(self, reason):
        if self.safety_reason is None:
            self.safety_reason = reason
            self._pending_action = True
            self._delivered = False

    def _now(self):
        now = self.now_ns()
        if type(now) is not int or now < self.last_now_ns:
            self._fault("PHYSICAL_CLOCK_INVALID")
            return self.last_now_ns
        self.last_now_ns = now
        return now

    def snapshot(self):
        navigation = self._latest["stop_reason"] == "INTERSECTION_CONFIRMED"
        stopped = self.safety_reason is not None or self._latest["stop_requested"]
        tracker = self._latest["tracker"]
        return dict(observation=self._latest["observation"], tracker=tracker,
                    confirmed_event=None, confirmed_kind=self.confirmed_kind,
                    stop_requested=bool(stopped),
                    stop_action=bool(self._pending_action),
                    stop_class="SAFETY" if self.safety_reason else "NAVIGATION" if navigation else None,
                    stop_reason=self.safety_reason or self._latest["stop_reason"],
                    navigation_stop_requested=bool(navigation and not self.safety_reason),
                    motion_verified=False, ready_to_resume=False,
                    frame_accepted=bool(tracker is not None and tracker['accepted'] and not self.safety_reason),
                    clock_id=self.clock_id, epoch_ns=self.epoch_ns,
                    captured_ns=self.last_capture_ns, processing_ms=self.processing_ms,
                    input_age_ms=self.input_age_ms)

    def process_frame(self, image, *, pixel_format, captured_ns, seq, clock_id):
        self._pending_action = False
        now = self._now()
        self.input_age_ms = (now-captured_ns)/1e6 if type(captured_ns) is int else None
        if self.safety_reason:
            return self.snapshot()
        if clock_id != self.clock_id:
            self._fault("CAMERA_CLOCK_MISMATCH")
        elif (type(captured_ns) is not int or captured_ns < 0
              or not 0 <= now - captured_ns <= self.max_age_ns):
            self._fault("CAMERA_TIMESTAMP_STALE_OR_INVALID")
        if self.safety_reason:
            return self.snapshot()
        if self.epoch_ns is None:
            self.epoch_ns = captured_ns
        if captured_ns < self.epoch_ns:
            self._fault("CAMERA_TIMESTAMP_BEFORE_SESSION")
            return self.snapshot()
        started = time.perf_counter_ns()
        try:
            bgr = to_bgr(image, pixel_format)
        except (ValueError, cv2.error):
            return self.camera_failed("INVALID_CAMERA_FRAME")
        try:
            self._latest = self.pipeline.process_bgr(
                bgr, t_ns=captured_ns-self.epoch_ns, seq=seq,
                now_ns=now-self.epoch_ns, motion=None)
        except (ValueError, RuntimeError, cv2.error):
            return self.camera_failed("INVALID_CAMERA_ENVELOPE")
        self.processing_ms = (time.perf_counter_ns()-started)/1e6
        self.last_capture_ns = captured_ns
        self._pending_action = self._latest["stop_action"]
        observation, tracker = self._latest["observation"], self._latest["tracker"]
        if observation is None or tracker is None or not tracker["accepted"]:
            self._fault(self._latest["stop_reason"] or "INVALID_CAMERA_OBSERVATION")
        elif not observation["valid"]:
            if self.unknown_since_ns is None:
                self.unknown_since_ns = captured_ns
            if captured_ns-self.unknown_since_ns >= self.pipeline.stop_latch.unknown_timeout_ns:
                self._fault("PERSISTENT_UNKNOWN")
        else:
            self.unknown_since_ns = None
        # Also bound inference latency. A fresh input can become stale while
        # processing on a slower Pi; it must not authorize navigation afterward.
        if self._now()-captured_ns > self.max_age_ns:
            self._fault("PROCESSING_DEADLINE_MISSED")
        if tracker is not None and tracker["event"] is not None and not self.safety_reason:
            self.confirmed_kind = tracker["event"]["kind"]
        result = self.snapshot()
        if tracker is not None and tracker["event"] is not None and not self.safety_reason:
            result["confirmed_event"] = dict(tracker["event"])
        return result

    def camera_failed(self, reason="CAMERA_FAILURE"):
        self._pending_action = False
        self._fault(reason)
        self._latest = dict(self._latest, observation=None, tracker=None)
        return self.snapshot()

    def check_frame_timeout(self):
        """Call from the main loop when no new camera frame is available.

        The camera reader itself must have a bounded timeout. This Python hook is
        not an independent hardware watchdog and cannot stop a blocked process.
        """
        self._pending_action = False
        now = self._now()
        reference = self.created_ns if self.last_capture_ns is None else self.last_capture_ns
        if now-reference > self.max_age_ns:
            self._fault("CAMERA_FRAME_TIMEOUT")
        return self.snapshot()

    def apply_stop_request(self, connected_car):
        """Explicit bridge to basic_motion/arduino.py arduino.Stop().

        Caller owns the serial connection and gates every other motor writer.
        Call only during an explicitly authorized hardware test. Return True only
        means Stop() returned; its legacy ACK code does NOT verify physical rest.
        The camera-only CLI never constructs a car or calls this method.
        """
        if not self.snapshot()["stop_requested"] or self._delivered:
            return False
        try:
            connected_car.Stop()
        except Exception:
            self._fault("BLUNO_STOP_CALL_FAILED")
            raise
        self._delivered = True
        return True
