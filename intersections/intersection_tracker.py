"""T.5 temporal confirmation only. No detector, Webots, clock or motor I/O.

All times are integer simulation nanoseconds. PassageSample is trusted external
localization evidence, NOT a distance inferred from the camera observation.
See TRACKER.md for the lifetime and fixed-reference requirements.
"""
from collections import deque
from dataclasses import dataclass
from math import isfinite
from numbers import Real


@dataclass(frozen=True)
class TrackerConfig:
    confirm_positives: int = 3
    confirmation_window_ns: int = 500_000_000
    max_positive_gap_ns: int = 250_000_000
    clear_observations: int = 3
    passage_clearance_m: float = 0.10
    approach_clearance_m: float = 0.10
    max_observation_age_ns: int = 250_000_000

    def __post_init__(self):
        for name in ("confirm_positives", "confirmation_window_ns",
                     "max_positive_gap_ns", "clear_observations",
                     "max_observation_age_ns"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.confirm_positives < 2:
            raise ValueError("confirmation requires at least two observations")
        for name in ("passage_clearance_m", "approach_clearance_m"):
            if not _finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class PassageSample:
    """Same-frame, same-run, trustworthy pose projection onto an approach axis.

    reference_id identifies both the crossing and a FIXED incoming direction.
    signed_distance_m is positive before the robot body reference reaches the
    crossing plane and negative beyond it. Mark invalid after localization
    jumps/resets, teleports, changed axes, or uncertain association.
    """
    reference_id: str
    t_ns: int
    signed_distance_m: float
    valid: bool = True


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and isfinite(value)


def _ns(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer simulation timestamp")


def _validate_observation(obs):
    fields = {"valid", "intersection_detected", "kind", "state", "score",
              "center_uv", "reason"}
    if not isinstance(obs, dict) or set(obs) != fields or not isinstance(obs["reason"], str):
        raise ValueError("expected the detector's unchanged seven-field observation")
    if obs["valid"] is False:
        if (obs["intersection_detected"] is not None or obs["kind"] != "UNKNOWN"
                or obs["state"] != "UNKNOWN" or obs["score"] is not None
                or obs["center_uv"] is not None):
            raise ValueError("inconsistent UNKNOWN observation")
        return "UNKNOWN"
    if obs["valid"] is not True or not _finite(obs["score"]) or not 0 <= obs["score"] <= 1:
        raise ValueError("invalid observation validity or score")
    if obs["intersection_detected"] is False:
        if obs["kind"] != "NONE" or obs["state"] != "NONE" or obs["center_uv"] is not None:
            raise ValueError("inconsistent NONE observation")
        return "NONE"
    center = obs["center_uv"]
    if (obs["intersection_detected"] is not True or obs["kind"] != "X"
            or obs["state"] not in ("AHEAD_CANDIDATE", "IN_VIEW_CANDIDATE")
            or not isinstance(center, (list, tuple)) or len(center) != 2
            or not all(_finite(v) and 0 <= v <= 1 for v in center)):
        raise ValueError("inconsistent positive observation")
    return "POSITIVE"


class IntersectionTracker:
    """One instance per uninterrupted run; consume each fresh camera frame once.

    update returns status plus event=None or ONE INTERSECTION_CONFIRMED event.
    No automatic reset on dropout, clock change or camera reconnect. A new
    instance is an explicit session reset, not a way to rearm an encounter.
    """

    def __init__(self, *, run_id, config=None):
        if not isinstance(run_id, str) or not run_id:
            raise ValueError("run_id must be a nonempty string")
        self.run_id = run_id
        self.config = config if config is not None else TrackerConfig()
        self.state = "ARMED"
        self._last_t_ns = self._last_seq = None
        self._votes = deque()
        self._clear_count = 0
        self._reference = None
        self._encounter_id = 0

    def _status(self, reason, *, accepted=True, event=None, rearmed=False, passage_cleared=False):
        return dict(accepted=accepted, state=self.state, event=event,
                    encounter_id=self._encounter_id, positive_count=len(self._votes),
                    clear_count=self._clear_count, passage_reference=self._reference,
                    rearmed=rearmed, passage_cleared=passage_cleared, reason=reason)

    @staticmethod
    def _motion(motion, t_ns):
        if motion is None:
            return None
        if not isinstance(motion, PassageSample):
            raise ValueError("motion must be PassageSample or None")
        _ns(motion.t_ns, "motion.t_ns")
        if (type(motion.valid) is not bool or not isinstance(motion.reference_id, str)
                or not motion.reference_id or not _finite(motion.signed_distance_m)):
            raise ValueError("malformed passage sample")
        # A stale/future/invalid pose never establishes passage or a new approach.
        return motion if motion.valid and motion.t_ns == t_ns else None

    def update(self, observation, *, t_ns, seq, run_id, clock_id="SIM_TIME",
               motion=None, now_ns=None):
        """Feed a seven-field observation with its camera envelope and optional pose.

        seq must increase (unwrap device sequence counters in the adapter).
        now_ns, if provided, is CURRENT SIM_TIME, allowing stale-frame rejection.
        Wrong run/clock, stale, repeated or out-of-order frames return
        accepted=False without altering state. Malformed arguments raise ValueError.
        """
        _ns(t_ns, "t_ns")
        if type(seq) is not int or seq < 0:
            raise ValueError("seq must be a nonnegative increasing integer")
        if now_ns is not None:
            _ns(now_ns, "now_ns")
        category = _validate_observation(observation)
        pose = self._motion(motion, t_ns)
        if run_id != self.run_id or clock_id != "SIM_TIME":
            return self._status("wrong run or clock domain", accepted=False)
        if ((self._last_t_ns is not None and t_ns <= self._last_t_ns)
                or (self._last_seq is not None and seq <= self._last_seq)):
            return self._status("duplicate or out-of-order camera frame", accepted=False)
        if now_ns is not None and not 0 <= now_ns - t_ns <= self.config.max_observation_age_ns:
            return self._status("stale or future camera frame", accepted=False)
        self._last_t_ns, self._last_seq = t_ns, seq

        rearmed = False
        if self.state == "WAITING_FOR_APPROACH":
            if pose is None or pose.signed_distance_m < self.config.approach_clearance_m:
                return self._status("passage cleared; waiting for trusted new approach")
            self.state, self._reference = "ARMED", None
            rearmed = True

        if self.state == "LATCHED":
            past = (pose is not None and self._reference is not None
                    and pose.reference_id == self._reference
                    and pose.signed_distance_m <= -self.config.passage_clearance_m)
            # UNKNOWN, a positive, a missing pose or a changed reference breaks
            # the CLEAR streak, but never clears the encounter latch.
            self._clear_count = self._clear_count + 1 if past and category == "NONE" else 0
            if self._clear_count >= self.config.clear_observations:
                self.state = "WAITING_FOR_APPROACH"
                self._clear_count = 0
                return self._status("verified passage and visual clearance; new approach required",
                                    passage_cleared=True)
            return self._status("encounter latched; no repeat event")

        # Evidence ages by simulation time, not the number of update calls.
        while self._votes and t_ns - self._votes[0][0] > self.config.confirmation_window_ns:
            self._votes.popleft()
        if self._votes and t_ns - self._votes[-1][0] > self.config.max_positive_gap_ns:
            self._votes.clear()
        if category == "NONE":
            self._clear_count += 1
            if self._clear_count >= self.config.clear_observations:
                self._votes.clear()
        else:
            self._clear_count = 0  # UNKNOWN is not a clear observation.
        if category == "POSITIVE":
            self._votes.append((t_ns, seq))
        self.state = "CANDIDATE" if self._votes else "ARMED"
        if len(self._votes) < self.config.confirm_positives:
            return self._status("collecting fresh positive evidence", rearmed=rearmed)

        self._encounter_id += 1
        # Bind the fixed approach reference only if the body is still before
        # the crossing at confirmation. Without it the latch remains locked.
        self._reference = (pose.reference_id if pose is not None
                           and pose.signed_distance_m > 0 else None)
        event = dict(type="INTERSECTION_CONFIRMED", encounter_id=self._encounter_id,
                     run_id=self.run_id, t_ns=t_ns, seq=seq,
                     first_positive_t_ns=self._votes[0][0], positive_count=len(self._votes),
                     kind="X", score=observation["score"],
                     center_uv=list(observation["center_uv"]))
        self._votes.clear()
        self.state, self._clear_count = "LATCHED", 0
        return self._status("new intersection confirmed", event=event, rearmed=rearmed)
