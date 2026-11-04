"""Real Pi/Bluno: open THIS file in Thonny and F5. No Webots imports.
Start with the axle reference centred on START_NODE, facing START_HEADING.
PASSED is an encoder estimate; destination stops remain BEFORE its node."""
from contextlib import redirect_stdout
import io
import signal
import struct
import time
import cv2
import numpy as np
from path_planning import physical_route, turn_command

MODE = "STOP"  # Stage 1 STOP; Stage 2 LEFT/RIGHT/STRAIGHT once; Stage 3 AUTO
START_NODE, DESTINATION, START_HEADING = "M", "S", "E"
BASE_PWM, PWM_LIMIT, TURN_PWM = 45, 60, 40  # initial PWM trials, NOT m/s or rad/s
KP, KD = .04, .015  # working huit.py gains; line following stays at 640x480
ROTATE_180 = False
STOP_ROW = None  # None: first 3 positives. Later: measured image-row fraction 0..1.
METRES_PER_TICK = None  # measure real forward travel / mean signed count change
FORWARD_SIGNS = None  # measured tuple (FR,FL,RR,RL), each +1 or -1
STOP_TO_CENTRE_M = None  # axle reference -> crossing centre AFTER coasting to rest
CENTRE_BRAKE_M = 0.0  # initial trial ONLY: Stop lead distance for the centre advance
TURN_90_TICKS = {"LEFT": None, "RIGHT": None}  # cutoff counts yielding 90 degrees AFTER coasting
REARM_DISTANCE_M = .25  # design: quarter of a 1 m edge, PLUS 3 clear images
IO_TIMEOUT, CAMERA_TIMEOUT, REST_SECONDS = 1., 2., .5  # I/O/encoder quiet, no CPU deadline
ENCODER_STALL_SECONDS = 2.  # motion requested but no encoder change: stop, not blind navigation

def call(function, *args, timeout=IO_TIMEOUT):
    """Bound the professor's endless WaitACK; suppress its per-command printing."""
    def expired(signum, frame):
        raise TimeoutError("hardware I/O timeout")
    previous = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        with redirect_stdout(io.StringIO()):
            return function(*args)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)

def encoder_counts(car):
    """Same ONE serial connection: A22/commode=2 N,n -> two int32 pairs, no ACK."""
    values = []
    for command in (b"N", b"n"):
        if car.serialport.write(command) != 1:
            raise OSError("incomplete encoder request")
        data = bytearray()
        while len(data) < 8:
            data.extend(car.serialport.read(8-len(data)))
        values.extend(struct.unpack("<2i", data))
    return np.array(values, dtype=np.int64)

def follow_line(mask, previous_error):
    roi, w = mask[int(.7*len(mask)):], mask.shape[1]
    centers = []
    for fraction in (.9, .75, .6):
        white = np.flatnonzero(roi[int(fraction*len(roi))])
        if white.size:
            groups = np.split(white, np.flatnonzero(np.diff(white) > 1)+1)
            centers.append(min((int((g[0]+g[-1])/2) for g in groups), key=lambda x: abs(x-w/2)))
    if not centers:
        return (0, 0), 0.
    error = int(np.mean(centers))-w/2
    correction = KP*error+KD*(error-previous_error)
    return tuple(int(np.clip(v, -PWM_LIMIT, PWM_LIMIT))
                 for v in (BASE_PWM+correction, BASE_PWM-correction)), error

def junction_row(mask):
    """Normalised junction row, or None; same separate-arm check as grid_simple."""
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 18,
                            minLineLength=22, maxLineGap=7)
    if lines is None:
        return None
    lines = sorted(lines.reshape(-1, 4), key=lambda p: np.hypot(p[2]-p[0], p[3]-p[1]),
                   reverse=True)[:18]
    evidence = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    h, w = mask.shape
    radii = np.linspace(12, 30, 16)
    angles = np.arange(72)*np.pi/36
    ring_directions = np.column_stack((np.cos(angles), np.sin(angles)))
    for i, a in enumerate(lines):
        p, u = a[:2].astype(float), (a[2:]-a[:2]).astype(float)
        u /= np.linalg.norm(u)
        for b in lines[i+1:]:
            q, v = b[:2].astype(float), (b[2:]-b[:2]).astype(float)
            v /= np.linalg.norm(v)
            cross = u[0]*v[1]-u[1]*v[0]
            if abs(cross) < 0.55:
                continue
            delta = q-p
            center = p+u*((delta[0]*v[1]-delta[1]*v[0])/cross)
            if not (30 <= center[0] < w-30 and 32 <= center[1] < h-32):
                continue
            arms = 0
            for direction in (u, -u, v, -v):
                samples = np.rint(center+radii[:, None]*direction).astype(int)
                hits = evidence[samples[:, 1], samples[:, 0]] > 0
                arms += bool(hits[:8].mean() >= .75 and hits[8:].mean() >= .75)
            if arms >= 3:
                # Require distinct white arcs: two Hough axes can describe one curve.
                branches = []
                for radius in (24, 30):
                    xy = np.rint(center+radius*ring_directions).astype(int)
                    ring = evidence[xy[:, 1], xy[:, 0]] > 0
                    ring = ring & np.roll(ring, 1)  # discard isolated noise hits
                    branches.append(np.count_nonzero(ring & ~np.roll(ring, 1)))
                if min(branches) >= 3:
                    return float(center[1]/h)
    return None

def main():
    camera = car = previous_term = None
    try:
        if not 0 < BASE_PWM <= PWM_LIMIT <= 255 or not 0 < TURN_PWM <= PWM_LIMIT:
            raise ValueError("Require positive PWM values within the configured limit")
        grid, route = physical_route(START_NODE, DESTINATION, START_HEADING, MODE)
        if MODE != "STOP":
            if (METRES_PER_TICK is None or STOP_TO_CENTRE_M is None or FORWARD_SIGNS is None
                    or not np.isfinite(METRES_PER_TICK) or METRES_PER_TICK <= 0 or not 0 < STOP_TO_CENTRE_M < 1
                    or not 0 <= CENTRE_BRAKE_M < STOP_TO_CENTRE_M
                    or len(FORWARD_SIGNS) != 4 or any(s not in (-1, 1) for s in FORWARD_SIGNS)):
                raise ValueError("Set physical METRES_PER_TICK, FORWARD_SIGNS, STOP_TO_CENTRE_M")
            headings = [START_HEADING]+[grid.direction(a, b) for a, b in zip(route, route[1:])]
            turns = {turn_command(a, b) for a, b in zip(headings, headings[1:])} if MODE == "AUTO" else {MODE}
            if any(TURN_90_TICKS[t] is None or TURN_90_TICKS[t] <= 0 for t in turns & {"LEFT", "RIGHT"}):
                raise ValueError("Set real LEFT/RIGHT TURN_90_TICKS first")
        if STOP_ROW is not None and not 0 < STOP_ROW < 1:
            raise ValueError("STOP_ROW is None or a measured fraction between 0 and 1")
        from picamera2 import Picamera2
        from arduino import arduino
        cv2.setNumThreads(1)
        previous_term = signal.signal(signal.SIGTERM, signal.default_int_handler)
        print("STARTING", flush=True)
        camera = call(Picamera2, timeout=CAMERA_TIMEOUT)
        config = camera.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}, queue=False)
        call(camera.configure, config, timeout=CAMERA_TIMEOUT)
        call(camera.start, timeout=CAMERA_TIMEOUT)
        time.sleep(2)  # same exposure warm-up as working huit.py
        car = call(arduino, "/dev/ttyACM0")
        car.serialport.write_timeout = IO_TIMEOUT
        if not call(car.connect, timeout=6):
            raise OSError("Arduino connection failed")
        call(car.Stop)
        state, index, heading = "FOLLOWING", 1, START_HEADING
        previous, last_targets, positives, clear, error = None, (0, 0), 0, 0, 0.
        travel = reference = centre_at = 0.
        quiet_since, trial_done, centred, turned = time.monotonic(), False, True, True
        centre_stopping = False
        pivot_counts, decision = None, "STRAIGHT"
        if MODE == "AUTO":
            state = "PASSING" if len(route) > 1 else "WAITING_FOR_DECISION"
            decision = turn_command(heading, grid.direction(route[0], route[1])) if len(route) > 1 else "STRAIGHT"
            turned = decision == "STRAIGHT"
        print("LINE FOLLOWING" if state != "WAITING_FOR_DECISION" else state, flush=True)
        while True:
            if state == "WAITING_FOR_DECISION" and (MODE == "STOP" or trial_done or index >= len(route)-1):
                time.sleep(.05)  # alive; Stop already sent, no automatic restart
                continue
            counts = call(encoder_counts, car) if MODE != "STOP" else None
            if counts is not None:
                if previous is not None:
                    delta = (counts-previous+(1 << 31)) % (1 << 32)-(1 << 31)
                    travel += float(np.mean(delta*np.array(FORWARD_SIGNS)))*METRES_PER_TICK
                    if np.any(delta): quiet_since = time.monotonic()
                    elif any(last_targets) and time.monotonic()-quiet_since > ENCODER_STALL_SECONDS:
                        raise OSError("NO ENCODER MOTION: stop and check motors/feedback")
                previous = counts
            try:
                frame = call(camera.capture_array, timeout=CAMERA_TIMEOUT)
                if not isinstance(frame, np.ndarray) or frame.shape != (480, 640, 3) or frame.dtype != np.uint8:
                    raise ValueError("invalid camera frame")
                if ROTATE_180: frame = cv2.rotate(frame, cv2.ROTATE_180)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # RGB888 is BGR in Picamera2 memory
                line = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)[1]
                mask = cv2.threshold(cv2.resize(gray, (320, 240)), 110, 255, cv2.THRESH_BINARY)[1]
            except (OSError, ValueError, RuntimeError, cv2.error):
                call(car.Stop); last_targets, positives, error = (0, 0), 0, 0.
                print("CAMERA ERROR - STOP REQUESTED", flush=True)
                continue
            targets = (0, 0)
            row = junction_row(mask) if state == "FOLLOWING" else None
            if state == "FOLLOWING":
                positives = positives+1 if row is not None and (STOP_ROW is None or row >= STOP_ROW) else 0
                # Corners have TWO arms: map/odometry fallback, never call them X/T detections.
                corner = MODE != "STOP" and len(grid.adjacency[route[index]]) == 2 and travel-reference >= 1.-STOP_TO_CENTRE_M
                if positives >= 3 or corner:
                    state = "WAITING_FOR_DECISION"
                    print("MAP CORNER (ODOMETRY ESTIMATE)" if corner else "INTERSECTION DETECTED", flush=True)
                    print("WAITING_FOR_DECISION: EXPECTED "+route[index]+" AHEAD - NOT AT NODE", flush=True)
                    if row is not None: print("DETECTION ROW "+str(round(row, 3)), flush=True)
                else: targets, error = follow_line(line, error)
            if state == "WAITING_FOR_DECISION":
                if MODE != "STOP" and not trial_done and index < len(route)-1 and last_targets == (0, 0) and time.monotonic()-quiet_since >= REST_SECONDS:
                    decision = turn_command(heading, grid.direction(route[index], route[index+1]))
                    centre_at, centred, turned = travel+STOP_TO_CENTRE_M, False, decision == "STRAIGHT"
                    centre_stopping = False
                    state = "PASSING"
                    trial_done = MODE != "AUTO"
            elif state == "PASSING":
                if not centred and not centre_stopping and travel < centre_at-CENTRE_BRAKE_M: targets = (BASE_PWM, BASE_PWM)
                elif not centred and (last_targets != (0, 0) or time.monotonic()-quiet_since < REST_SECONDS):
                    centre_stopping = True  # request zero and wait out the actual firmware ramp
                else:
                    if not centred:
                        if not centre_at <= travel <= centre_at+.03:
                            raise RuntimeError("CENTRE NOT REACHED/OVERSHOT: adjust CENTRE_BRAKE_M")
                        print("PASSED "+route[index]+" (ENCODER ESTIMATE)", flush=True)
                        index += 1; reference, pivot_counts, centred = travel, counts.copy(), True
                    if pivot_counts is None: pivot_counts = counts.copy()
                    if not turned:
                        ticks = np.mean(np.abs((counts-pivot_counts+(1 << 31)) % (1 << 32)-(1 << 31)))
                        progress = ticks/TURN_90_TICKS[decision]
                        if progress > 1.4: raise RuntimeError("TURN NOT ACQUIRED: check calibration")
                        if progress >= 1.:
                            turned, clear, error = "SETTLING", 0, 0.
                            print("TURN STOP REQUESTED - WAITING FOR LINE", flush=True)
                        else: targets = (-TURN_PWM, TURN_PWM) if decision == "LEFT" else (TURN_PWM, -TURN_PWM)
                    elif turned == "SETTLING":
                        visible, turn_error = follow_line(line, 0.)
                        clear = clear+1 if time.monotonic()-quiet_since >= REST_SECONDS and any(visible) and abs(turn_error) <= 12 else 0
                        if clear >= 3: turned = True  # fresh alignment AFTER the ramp, not before
                    else:
                        heading = grid.direction(route[index-1], route[index])
                        targets, error = follow_line(line, error)
                        clear = clear+1 if junction_row(mask) is None else 0
                        if travel-reference >= REARM_DISTANCE_M and clear >= 3:
                            state, positives = "FOLLOWING", 0
                            print("LINE FOLLOWING", flush=True)
            if targets != last_targets or any(targets):
                call(car.SetMotors, *targets) if any(targets) else call(car.Stop)
                last_targets = targets
                if not any(targets): quiet_since = time.monotonic()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("ERROR: "+str(error), flush=True)
    finally:
        if car is not None:
            try: call(car.Stop)
            except Exception: print("STOP UNCONFIRMED - USE MOTOR POWER CUTOFF", flush=True)
            try: call(car.disconnect)
            except Exception: pass
        if camera is not None:
            try: call(camera.stop, timeout=CAMERA_TIMEOUT); call(camera.close, timeout=CAMERA_TIMEOUT)
            except Exception: pass
        if previous_term is not None: signal.signal(signal.SIGTERM, previous_term)

if __name__ == "__main__":
    main()
