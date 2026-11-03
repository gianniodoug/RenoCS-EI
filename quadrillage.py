"""Standalone REAL quadrillage robot: open in the same Thonny environment as
huit.py, then F5. Existing arduino.py/utilitaries.py must remain importable.

No GUI, arguments, project-module imports, startup performance gate or 250 ms
processing deadline. The complete validated X/T geometric detector is embedded
below unchanged. SIGALRM bounds blocking I/O on Raspberry Pi Linux; it is NOT a
hardware watchdog. Thonny Stop/Restart, SIGKILL or lost USB may prevent cleanup.
Keep the operator-controlled motor-power cutoff available. STOP REQUESTED means
only that software requested zero targets, never that physical rest is measured.
"""
from contextlib import contextmanager, redirect_stdout
from itertools import combinations
import io
import math
import os
import signal
import time

# Simple run parameters. PWM is NOT measured wheel speed.
PORT = "/dev/ttyACM0"
BASE_PWM = 45
PWM_LIMIT = 60
KP = 0.04
KD = 0.015
CAMERA_SIZE = (640, 480)     # Same preview/RGB888 configuration as local huit.py.
ROTATE_180 = False          # Leave False unless the working camera needs rotation.
CONFIRM_FRAMES = 3          # Fresh positive observations; no 250 ms time window.
CAMERA_TIMEOUT_S = 2.0      # Only stalled acquisition, NOT inference time.
SERIAL_TIMEOUT_S = 1.0      # Bounds the professor's otherwise endless ACK loop.
CONNECT_TIMEOUT_S = 6.0     # Includes the professor's two-second boot delay.
WARMUP_SECONDS = 2.0
MAX_RUN_SECONDS = 30.0      # Conservative first run; measured from start of loop.

# Limit CPU oversubscription without reducing camera/detector resolution.
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import cv2
import numpy as np

# --- Validated detector, embedded verbatim (no RenoCS imports). ---
SETTINGS = {
    "analysis_max_width": 640,
    "white_value_min": 125,
    "white_saturation_max": 85,
    "minimum_axis_angle_deg": 35,
    "arm_support_min": 0.72,
    "ring_white_fraction_max": 0.65,
    "ahead_boundary_v": 0.45,
    "border_visible_support_min": 0.85,
    "border_visible_span_fraction": 0.035,
}


def _observation(valid=False, detected=None, kind="UNKNOWN", state="UNKNOWN",
                 score=None, center=None, reason="insufficient evidence"):
    return dict(valid=valid, intersection_detected=detected, kind=kind,
                state=state, score=score, center_uv=center, reason=reason)


def _validate(image):
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
            or image.ndim != 3 or image.shape[2] != 3
            or min(image.shape[:2]) <= 1):
        raise ValueError("image must be a uint8 BGR NumPy array of shape (H,W,3), H,W > 1")


def _ray_support(mask, center, direction, radii):
    points = center + radii[:, None] * direction
    pixels = np.rint(points).astype(int)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < mask.shape[1])
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < mask.shape[0]))
    hits = np.zeros(len(radii), dtype=bool)
    hits[inside] = mask[pixels[inside, 1], pixels[inside, 0]] != 0
    # Out-of-view samples are missing evidence, never assumed white.
    middle = len(hits) // 2
    return float(min(hits[:middle].mean(), hits[middle:].mean())), points, hits


def _border_junction(candidate, shape):
    """Three actually visible arms + clipped full-scale evidence => abstain.

    Never invent white support outside the frame. Require a substantial visible
    span on three distinct rays, so a border-touching line or two-arm bend alone
    is insufficient. This guard only changes a would-be negative to UNKNOWN.
    """
    h, w = shape
    visible_arms, clipped = 0, False
    for _, points, hits in candidate["rays"]:
        pixels = np.rint(points).astype(int)
        inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < w)
                  & (pixels[:, 1] >= 0) & (pixels[:, 1] < h))
        clipped |= not bool(inside.all())
        indices = np.flatnonzero(inside)
        if len(indices) < 8:
            continue
        span = np.linalg.norm(points[indices[-1]] - points[indices[0]])
        if (span >= min(h, w) * SETTINGS["border_visible_span_fraction"]
                and float(hits[inside].mean()) >= SETTINGS["border_visible_support_min"]):
            visible_arms += 1
    return bool(clipped and visible_arms >= 3)


def _t_junction(candidate, mask, supported, allow_wide=False, fit_visible=False):
    """Require three real branches AND observable absence of a fourth branch.

    A clipped X cannot supply negative evidence outside the image: the entire
    evidence disk (or fitted ellipse) must be visible. Check arm support, three
    separated stripes on several complete rings, and a dark missing-arm sector.
    No direction is assumed to be vertical, horizontal, or the robot's front.
    """
    if candidate["strong_arms"] != 3 or candidate["ring_arms"] != 3:
        return None
    h, w = mask.shape
    size = min(h, w)
    center = candidate["center"]
    guard = size * .24
    axes = np.array([guard, guard])
    if fit_visible:
        # Perspective can put a complete junction above the fixed disk's margin.
        # Fit a whole ellipse, never assume darkness in out-of-frame samples.
        # Its shortest radius must exceed the original .10-size inner ring.
        # The original three-arm .16-size prefilter above is still mandatory.
        clearance = np.minimum(center, [w - 1 - center[0], h - 1 - center[1]])
        axes = np.minimum(axes, clearance - 2)
        if min(axes) < size * .10 + 4:
            return None
    if not (axes[0] + 2 <= center[0] < w - axes[0] - 2
            and axes[1] + 2 <= center[1] < h - axes[1] - 2):
        return None
    directions = [(ray[1][-1] - center) / np.linalg.norm(ray[1][-1] - center)
                  for ray in candidate["rays"]]
    present = [i for i, arm in enumerate(candidate["arms"])
               if arm >= SETTINGS["arm_support_min"]]
    missing = next(i for i in range(4) if i not in present)
    supports = []
    for i in present:
        # Keep the original scale where visible; shorten only the constrained axis.
        extent = min(size * .22, .22 / .24 / np.linalg.norm(directions[i] / axes))
        radii = np.linspace(size * .015, extent, 64)
        supports.append(_ray_support(supported, center, directions[i], radii)[0])
    if min(supports) < .85:
        return None
    # Search a sector, not just one thin axis which could miss a displaced arm.
    direction = directions[missing]
    phi = np.arctan2(direction[1], direction[0]) + np.deg2rad(np.linspace(-12, 12, 25))
    sector_directions = np.stack((np.cos(phi), np.sin(phi)), axis=1)
    # Use the shortest ellipse boundary across the entire sector, so every
    # negative-evidence sample is observed at its true direction, never warped.
    sector_extent = min(1 / np.linalg.norm(sector_directions / axes, axis=1))
    rr = np.linspace(size * .045, sector_extent, 64)
    points = center + rr[:, None, None] * sector_directions[None]
    pixels = np.rint(points).astype(int)
    hits = mask[pixels[:, :, 1], pixels[:, :, 0]] != 0
    dark = float(hits.mean()) <= .03 and not np.any(hits.mean(axis=1) > .20)
    # At close range the projected through-stripe can occupy the inner samples.
    # Require the same dark-sector evidence beyond that central junction area;
    # a genuine fourth arm still crosses this sector (including short arms).
    outer_hits = hits[rr >= size * .08]
    outer_dark = float(outer_hits.mean()) <= .03 and not np.any(outer_hits.mean(axis=1) > .20)
    # A fitted region must retain ALL inner absence evidence. Ignoring its inner
    # samples can conceal a short fourth arm when the outer extent is shortened.
    if not (dark or (allow_wide and not fit_visible and outer_dark)):
        return None
    angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    # The candidate already has 3 separated arms at radius .14. Use two more
    # outer rings, rather than .10 where perspective makes nearby stripes merge.
    for radius in ((.16, .22) if allow_wide else (.10, .16, .22)):
        ring = np.rint(center + axes * (radius / .24) * np.stack((np.cos(angles), np.sin(angles)), axis=1)).astype(int)
        hits = np.uint8(mask[ring[:, 1], ring[:, 0]] != 0)
        circular = np.tile(hits, 3)[None]
        circular = cv2.morphologyEx(circular, cv2.MORPH_CLOSE, np.ones((1, 3), np.uint8))
        circular = cv2.morphologyEx(circular, cv2.MORPH_OPEN, np.ones((1, 3), np.uint8))
        hits = circular[0, 360:720]
        branches = np.count_nonzero((hits != 0) & (np.roll(hits, 1) == 0))
        if branches != 3 or float(hits.mean()) > .50:
            return None
    return float(min(supports))


def _analyze(image, debug_candidates=True):
    _validate(image)
    height, width = image.shape[:2]
    scale = min(1.0, SETTINGS["analysis_max_width"] / width)
    frame = image if scale == 1.0 else cv2.resize(
        image, (max(1, round(width * scale)), max(1, round(height * scale))), interpolation=cv2.INTER_AREA)
    h, w = frame.shape[:2]
    debug = {"frame": frame, "mask": np.zeros((h, w), np.uint8),
             "lines": [], "candidate": None, "white_fraction": 0.0}
    if min(h, w) < 64:
        return _observation(reason="insufficient image resolution (minimum 64 pixels per side)"), debug

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, SETTINGS["white_value_min"]),
                       (179, SETTINGS["white_saturation_max"], 255))
    # Remove isolated pixels without filling the gaps between unrelated stripes.
    count, components, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    # Same area predicate, one label lookup instead of a full image scan for
    # EACH retained component (costly on a noisy physical Pi camera image).
    keep = np.uint8(stats[:, cv2.CC_STAT_AREA] >= max(8, h * w * 0.00005)) * 255
    keep[0] = 0
    mask = keep[components]
    fraction = float(np.count_nonzero(mask) / mask.size)
    debug.update(mask=mask, white_fraction=fraction)
    if fraction < 0.0008:
        return _observation(reason="no sufficiently supported white track"), debug
    if fraction > 0.30:
        return _observation(reason="excessive white area: glare, exposure or unsuitable scene"), debug

    size = min(h, w)
    raw = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=max(18, int(size * 0.09)),
                         minLineLength=max(20, int(size * 0.14)), maxLineGap=8)
    lines = [] if raw is None else raw.reshape(-1, 4).tolist()
    # Longest segments first; bound pairwise work in cluttered images.
    lines.sort(key=lambda p: (p[2] - p[0]) ** 2 + (p[3] - p[1]) ** 2, reverse=True)
    # Collapse duplicate stripe edges/segments before considering line pairs.
    unique = []
    unique_starts, unique_axes = [], []
    duplicate_cos = np.cos(np.deg2rad(3))
    for line in lines:
        start, end = np.array(line[:2], float), np.array(line[2:], float)
        direction = (end - start) / np.linalg.norm(end - start)
        duplicate = False
        if unique:
            # Same direction/offset predicates, batched across previous axes.
            axes = np.asarray(unique_axes)
            delta = (start+end)/2-np.asarray(unique_starts)
            duplicate = bool(np.any((np.abs(axes @ direction) > duplicate_cos)
                & (np.abs(delta[:,0]*axes[:,1]-delta[:,1]*axes[:,0]) < size*.02)))
        if not duplicate:
            unique.append(line)
            unique_starts.append(start)
            unique_axes.append(direction)
        if len(unique) == 60:
            break
    lines = unique
    debug["lines"] = lines
    tolerance = max(1, int(round(size * 0.008)))
    supported = cv2.dilate(mask, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * tolerance + 1, 2 * tolerance + 1)))
    radii = np.linspace(size * 0.045, size * 0.16, 36)
    geometry = []
    for segment in lines:
        start = np.array(segment[:2], float)
        direction = np.array(segment[2:], float)-start
        geometry.append((start, direction/np.linalg.norm(direction)))
    angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    ring_offsets = size * 0.14 * np.column_stack((np.cos(angles), np.sin(angles)))
    ring_kernel = np.ones((1, 3), np.uint8)
    candidates = []
    for (a, d), (b, e) in combinations(geometry, 2):
        angle = math.degrees(math.acos(min(1., abs(float(d @ e)))))
        if angle < SETTINGS["minimum_axis_angle_deg"]:
            continue
        # Exact 2x2 intersection formula; avoids many tiny BLAS solves and
        # thread-pool startup. The angle guard above excludes parallel axes.
        delta = b-a
        t = (delta[0]*e[1]-delta[1]*e[0])/(d[0]*e[1]-d[1]*e[0])
        center = a + t * d
        x, y = center
        if not (0 <= x < w - 1 and 0 <= y < h - 1):
            continue
        if not supported[int(round(y)), int(round(x))]:
            continue
        # Identical near/far samples, four rays at once. No image downsampling
        # or arm-threshold changes are required for the Pi 3 fast path.
        directions = np.stack((d,-d,e,-e))
        points = center + radii[None,:,None]*directions[:,None,:]
        pixels = np.rint(points).astype(int)
        inside = ((pixels[:,:,0] >= 0) & (pixels[:,:,0] < w)
                  & (pixels[:,:,1] >= 0) & (pixels[:,:,1] < h))
        hits = np.zeros((4,len(radii)),bool)
        hits[inside] = supported[pixels[:,:,1][inside],pixels[:,:,0][inside]] != 0
        middle = len(radii)//2
        arms = np.minimum(hits[:,:middle].mean(axis=1),hits[:,middle:].mean(axis=1)).tolist()
        rays = list(zip(arms,points,hits))
        # Both scales must support every arm; a V or a T cannot be a full X.
        strong = sum(s >= SETTINGS["arm_support_min"] for s in arms)
        score = float(min(arms))
        if strong < 3 and not debug_candidates:
            # Rings cannot make fewer than 3 supported rays positive. Border
            # abstention uses the actual rays, not these unused ring metrics.
            candidates.append(dict(center=center,arms=arms,rays=rays,angle_deg=angle,
                strong_arms=strong,score=score,ring_white_fraction=0.,ring_arms=0))
            continue
        # A broad bright patch can support every ray without being four stripes.
        ring = np.rint(center + ring_offsets).astype(int)
        inside = ((ring[:, 0] >= 0) & (ring[:, 0] < w)
                  & (ring[:, 1] >= 0) & (ring[:, 1] < h))
        ring_hits = np.zeros(len(ring), np.uint8)
        ring_hits[inside] = mask[ring[inside, 1], ring[inside, 0]] != 0
        ring_white = float(ring_hits.mean())
        # Count separated angular white regions. Four rays inside one thick
        # stripe are still only two actual arms. Circular closing fills 1-bin gaps.
        circular = np.tile(ring_hits, 3)[None, :]
        circular = cv2.morphologyEx(circular, cv2.MORPH_CLOSE, ring_kernel)
        circular = cv2.morphologyEx(circular, cv2.MORPH_OPEN, ring_kernel)
        ring_hits = circular[0, len(ring):2 * len(ring)]
        ring_arms = int(np.count_nonzero((ring_hits != 0) & (np.roll(ring_hits, 1) == 0)))
        candidates.append(dict(center=center, arms=arms, rays=rays, angle_deg=angle,
                               strong_arms=strong, score=score, ring_white_fraction=ring_white,
                               ring_arms=ring_arms))

    candidates.sort(key=lambda c: (c["strong_arms"], c["score"], sum(c["arms"])), reverse=True)
    best = candidates[0] if candidates else None
    debug["candidate"] = best
    positives = [c for c in candidates if c["strong_arms"] == 4
                 and c["ring_white_fraction"] <= SETTINGS["ring_white_fraction_max"]
                 and c["ring_arms"] == 4]
    if positives:
        best = positives[0]
        debug["candidate"] = best
        # Competing, separated Xs are outside this single-junction contract.
        if any(np.linalg.norm(c["center"] - best["center"]) > size * 0.12
               for c in positives[1:]):
            return _observation(reason="multiple separated X candidates; single center ambiguous"), debug
        center = [float(best["center"][0] / (w - 1)), float(best["center"][1] / (h - 1))]
        state = "AHEAD_CANDIDATE" if center[1] < SETTINGS["ahead_boundary_v"] else "IN_VIEW_CANDIDATE"
        return _observation(True, True, "X", state, best["score"], center,
                            "two crossing axes; all four arms supported at near and far scales"), debug
    # X evidence has priority and its validated decision path above is unchanged.
    t_candidates = []
    for candidate in candidates:
        score = _t_junction(candidate, mask, supported)
        if score is not None:
            t_candidates.append((candidate, score))
    # Preserve already accepted T observations and their center selection. Only
    # use the close/wide-stripe fallback when the original profile abstains.
    if not t_candidates:
        for candidate in candidates:
            score = _t_junction(candidate, mask, supported, allow_wide=True)
            if score is not None:
                t_candidates.append((candidate, score))
    # Last resort only: preserve previously accepted T observations unchanged.
    # A full in-frame ellipse is mandatory, including the missing-arm sector.
    if not t_candidates:
        for candidate in candidates:
            score = _t_junction(candidate, mask, supported, allow_wide=True, fit_visible=True)
            if score is not None:
                t_candidates.append((candidate, score))
    if t_candidates:
        best, score = t_candidates[0]
        debug["candidate"] = best
        if any(np.linalg.norm(c["center"] - best["center"]) > size * .12
               for c, _ in t_candidates[1:]):
            return _observation(reason="multiple separated T candidates; single center ambiguous"), debug
        center = [float(best["center"][0] / (w - 1)), float(best["center"][1] / (h - 1))]
        state = "AHEAD_CANDIDATE" if center[1] < SETTINGS["ahead_boundary_v"] else "IN_VIEW_CANDIDATE"
        return _observation(True, True, "T", state, score, center,
                            "three connected supported arms; fourth-arm sector visibly absent at multiple scales"), debug
    if best and best["strong_arms"] >= 3:
        return _observation(reason="incomplete or broad junction: separated four-arm X not established"), debug
    for candidate in candidates:
        if _border_junction(candidate, (h, w)):
            debug["candidate"] = candidate
            return _observation(reason="partial border junction: three visible arms, full arm evidence clipped"), debug
    if not lines:
        return _observation(reason="white regions present but no supported track-length segment"), debug
    # Long narrow components support ordinary track; broad blobs do not.
    track_like = False
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < max(8, h * w * 0.00005):
            continue
        points = cv2.findNonZero(np.uint8(components == i))
        sides = cv2.minAreaRect(points)[1]
        density = stats[i, cv2.CC_STAT_AREA] / max(1, sides[0] * sides[1])
        if (max(sides) >= size * 0.20
                and (max(sides) / max(1, min(sides)) >= 2 or density < 0.35)):
            track_like = True
            break
    if not track_like:
        return _observation(reason="white shape lacks plausible elongated track support"), debug
    return _observation(True, False, "NONE", "NONE", best["score"] if best else 0.0,
                        reason="supported track; no four-arm X (single line or curve)"), debug


def detect_intersection(image):
    """Return the seven-field README observation for a uint8 BGR image.

    Invalid formats raise ValueError. Unusable/ambiguous scenes return UNKNOWN.
    Score is geometric support, not probability. This is stateless: repeated
    frames yield repeated candidates; T.5 must latch/validate navigation events.
    Input pixels are not modified. No pose, filename or label is consumed.
    """
    return _analyze(image, debug_candidates=False)[0]


# --- Physical loop: the SAME Picamera2 / professor arduino API as huit.py. ---
@contextmanager
def _io_timeout(seconds):
    """Linux main-thread I/O timeout, not an image-processing deadline."""
    if not hasattr(signal, "setitimer"):
        raise RuntimeError("Raspberry Pi Linux required")
    def expired(signum, frame):
        raise TimeoutError("I/O timeout")
    previous = signal.getsignal(signal.SIGALRM)
    previous_timer = signal.getitimer(signal.ITIMER_REAL)
    if previous_timer != (0.0, 0.0):
        raise RuntimeError("another application is using SIGALRM")
    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def _call(seconds, function, *args, quiet=False, **kwargs):
    with _io_timeout(seconds):
        if quiet:
            # The original WaitACK prints every motor ACK. Suppress that noise.
            with redirect_stdout(io.StringIO()):
                return function(*args, **kwargs)
        return function(*args, **kwargs)


def _wheel_targets(image, previous_error):
    # huit.py's bottom 30%, threshold80, three scan rows and nearest-centre groups.
    # Picamera2 RGB888 memory is BGR, as required by the embedded detector.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    roi = gray[int(h * .7):h, :]
    _, binary = cv2.threshold(roi, 80, 255, cv2.THRESH_BINARY)
    centers = []
    for y in (int(binary.shape[0]*.9), int(binary.shape[0]*.75), int(binary.shape[0]*.6)):
        white = np.where(binary[y] > 0)[0]
        if len(white):
            groups = np.split(white, np.where(np.diff(white) > 1)[0]+1)
            centers.append(min((int((g[0]+g[-1])/2) for g in groups), key=lambda x:abs(x-w/2)))
    if not centers:
        return None, previous_error
    error = int(np.mean(centers))-w/2
    correction = KP*error+KD*(error-previous_error)
    left = max(-PWM_LIMIT, min(PWM_LIMIT, BASE_PWM+correction))
    right = max(-PWM_LIMIT, min(PWM_LIMIT, BASE_PWM-correction))
    return (left, right), error


def _terminate(signum, frame):
    raise KeyboardInterrupt


def main():
    print("STARTING", flush=True)
    camera = robot = None
    camera_started = False
    previous_term = None
    phase = "setup"
    try:
        if not (0 < BASE_PWM <= PWM_LIMIT <= 255 and CONFIRM_FRAMES >= 2
                and min(CAMERA_SIZE) >= 64
                and min(CAMERA_TIMEOUT_S, SERIAL_TIMEOUT_S, CONNECT_TIMEOUT_S, MAX_RUN_SECONDS) > 0):
            raise ValueError("invalid parameters")
        if not hasattr(signal, "setitimer"):
            raise RuntimeError("Raspberry Pi Linux required")
        previous_term = signal.signal(signal.SIGTERM, _terminate)
        from picamera2 import Picamera2
        from arduino import arduino
        cv2.setNumThreads(1)
        phase = "camera"
        camera = _call(CAMERA_TIMEOUT_S, Picamera2)
        # queue=False: capture_array waits for the next frame rather than
        # reusing the last completed request. Same acquisition API as huit.py.
        config = camera.create_preview_configuration(
            main={"format":"RGB888", "size":CAMERA_SIZE}, queue=False)
        _call(CAMERA_TIMEOUT_S, camera.configure, config)
        _call(CAMERA_TIMEOUT_S, camera.start)
        camera_started = True
        time.sleep(WARMUP_SECONDS)
        phase = "Arduino"
        robot = _call(CONNECT_TIMEOUT_S, arduino, PORT, quiet=True)
        if not _call(CONNECT_TIMEOUT_S, robot.connect, quiet=True):
            raise OSError("Arduino connection failed")
        _call(SERIAL_TIMEOUT_S, robot.Stop, quiet=True)
        previous_error = 0
        positives = 0
        pending_kind = None
        following_message = False
        began = time.monotonic()
        while time.monotonic()-began < MAX_RUN_SECONDS:
            phase = "camera"
            # A fresh synchronous request, bounded if acquisition stalls.
            # No cached fallback and no guessed sensor/wall-clock conversion.
            image = _call(CAMERA_TIMEOUT_S, camera.capture_array)
            _validate(image)
            if image.shape != (CAMERA_SIZE[1], CAMERA_SIZE[0], 3):
                raise RuntimeError("unexpected camera shape")
            if ROTATE_180:
                image = cv2.rotate(image, cv2.ROTATE_180)
            phase = "line lost"
            targets, previous_error = _wheel_targets(image, previous_error)
            if targets is None:
                raise RuntimeError("line lost")
            phase = "image processing"
            observation = detect_intersection(image)
            if not observation["valid"]:
                # UNKNOWN is not evidence of a clear path. End this run;
                # the finally block calls the real Stop(), with no restart.
                phase = "uncertain image"
                raise RuntimeError("UNKNOWN")
            if observation["intersection_detected"]:
                # A mixed X/T sequence is still positive evidence for stopping;
                # print the final observed kind, without choosing any navigation.
                positives += 1
                pending_kind = observation["kind"]
                if positives >= CONFIRM_FRAMES:
                    phase = "Arduino"
                    _call(SERIAL_TIMEOUT_S, robot.Stop, quiet=True)
                    print(pending_kind+" DETECTED", flush=True)
                    break  # LATCH: no movement or automatic restart after this.
            else:
                positives = 0
                pending_kind = None
            # Same real API and left/right ordering used by huit.py.
            phase = "Arduino"
            _call(SERIAL_TIMEOUT_S, robot.SetMotors, *targets, quiet=True)
            if not following_message:
                print("FOLLOWING LINE", flush=True)
                following_message = True
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception:
        print("ERROR: "+phase, flush=True)
        return 1
    finally:
        # Best effort only: a lost link, torn packet, hard kill or driver fault
        # can prevent physical stopping. Use the independent motor-power cutoff.
        if robot is not None:
            try:
                _call(SERIAL_TIMEOUT_S, robot.Stop, quiet=True)
                print("STOP REQUESTED", flush=True)
            except Exception:
                print("ERROR: stop unconfirmed - USE POWER CUTOFF", flush=True)
            try:
                _call(SERIAL_TIMEOUT_S, robot.disconnect, quiet=True)
            except Exception:
                pass
        if camera is not None:
            try:
                if camera_started:
                    _call(CAMERA_TIMEOUT_S, camera.stop)
            except Exception:
                print("ERROR: camera cleanup", flush=True)
            try:
                _call(CAMERA_TIMEOUT_S, camera.close)
            except Exception:
                pass
        if previous_term is not None:
            signal.signal(signal.SIGTERM, previous_term)


if __name__ == "__main__":
    main()
