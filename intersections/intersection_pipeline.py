"""T.2/T.5 camera -> frozen detector -> tracker -> latched stop REQUEST.

Create one pipeline per simulation run. Feed each new frame once with its
SIM_TIME timestamp in integer nanoseconds and an increasing seq. The main
controller must override ALL wheel targets with zero while stop_requested=True;
this module owns no motors. Restart explicitly; never recreate on camera dropout.

result = pipeline.process_camera(camera, t_ns=camera_t_ns, seq=seq)
# result['observation'] retains the detector's seven-field BGR-image contract.
"""
import cv2
import numpy as np

if __package__:
    from .intersection_detector import detect_intersection
    from .intersection_tracker import IntersectionTracker
else:
    from intersection_detector import detect_intersection
    from intersection_tracker import IntersectionTracker


class StopLatch:
    """Integration policy only; never resets the tracker or resumes after stopping."""
    def __init__(self, unknown_timeout_ns=2_000_000_000):
        self.unknown_timeout_ns = unknown_timeout_ns
        self.unknown_since = None
        self.reason = None
        self.t_ns = None

    def stop(self, reason, t_ns):
        if self.reason is not None:
            return False
        self.reason, self.t_ns = reason, t_ns
        return True

    def observe(self, observation, tracker_status, t_ns, error=None):
        if self.reason is not None:
            return False
        if error:
            return self.stop(error, t_ns)
        if not tracker_status["accepted"]:
            return self.stop("TRACKER_REJECTED_FRAME", t_ns)
        if tracker_status["event"]:
            return self.stop("INTERSECTION_CONFIRMED", t_ns)
        if not observation["valid"]:
            if self.unknown_since is None:
                self.unknown_since = t_ns
            if t_ns - self.unknown_since >= self.unknown_timeout_ns:
                return self.stop("PERSISTENT_UNKNOWN", t_ns)
        else:
            self.unknown_since = None
        return False


class IntersectionPipeline:
    """Pure BGR API or Webots PiCam adapter; no clocks, files or devices on import.

    motion is the tracker's optional trusted PassageSample. With motion=None,
    confirmation works but the tracker cannot automatically rearm. The stop
    request stays latched regardless of later tracker observations/rearming.
    Missing/malformed images stop immediately; continuous UNKNOWN stops after
    two SIM_TIME seconds. This timeout is a simulation development parameter.
    """
    def __init__(self, *, run_id):
        self.run_id = run_id
        self.tracker = IntersectionTracker(run_id=run_id)
        self.stop_latch = StopLatch()

    def _result(self, observation, tracker, stop_action):
        return dict(observation=observation, tracker=tracker,
                    stop_requested=self.stop_latch.reason is not None,
                    stop_action=stop_action, stop_reason=self.stop_latch.reason,
                    stop_t_ns=self.stop_latch.t_ns)

    def _fault(self, reason, t_ns):
        return self._result(None, None, self.stop_latch.stop(reason, t_ns))

    def process_bgr(self, image, *, t_ns, seq, now_ns=None, motion=None):
        """image: NumPy uint8 (H,W,3), BGR, unflipped, no annotations.

        now_ns, when supplied, must be current time in the SAME simulation clock.
        A repeated/out-of-order/stale frame is rejected and requests a stop.
        """
        try:
            observation = detect_intersection(image)
            status = self.tracker.update(observation, t_ns=t_ns, seq=seq,
                                         run_id=self.run_id, clock_id="SIM_TIME",
                                         now_ns=now_ns, motion=motion)
        except (ValueError, RuntimeError, cv2.error):
            return self._fault("INVALID_IMAGE_OR_OBSERVATION", t_ns)
        action = self.stop_latch.observe(observation, status, t_ns)
        return self._result(observation, status, action)

    def process_camera(self, camera, *, t_ns, seq, now_ns=None, motion=None):
        """Read one already-enabled Webots camera, converting BGRA8 to owned BGR8.

        The main controller owns Robot.step(), PiCam.enable(64), camera sequence
        and acquisition timestamps; call only on a fresh camera sample.
        """
        try:
            raw = camera.getImage()
            if not raw:
                return self._fault("CAMERA_BUFFER_MISSING", t_ns)
            width, height = camera.getWidth(), camera.getHeight()
            if width <= 0 or height <= 0:
                raise ValueError("invalid camera dimensions")
            data = np.frombuffer(raw, dtype=np.uint8)
            if data.size != width * height * 4:
                raise ValueError("invalid BGRA camera buffer size")
            bgra = data.reshape(height, width, 4)
            image = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        except (ValueError, RuntimeError, cv2.error):
            return self._fault("INVALID_IMAGE_OR_OBSERVATION", t_ns)
        return self.process_bgr(image, t_ns=t_ns, seq=seq, now_ns=now_ns, motion=motion)
