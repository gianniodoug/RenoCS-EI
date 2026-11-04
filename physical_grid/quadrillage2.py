"""F5 in Thonny. Initially place the axle reference at START_NODE, facing HEADING.
A..E are the north row; U..Y the south row. Every grid edge is 1 metre.
Fill the physical measurements below: no simulation distances are substituted.
"""
import time
from collections import deque
import cv2
import numpy as np
from grid_map import GridMap
from path_planning import plan_mission_from_state

START_NODE, HEADING = "M", "E"
DESTINATION = "S"  # node A..Y, or ("A", "B", 0.40): metres from A towards B
PORT = "/dev/ttyACM0"
BASE_SPEED, MAX_SPEED = 100, 255  # working local Basique_Navigation/huit.py
KP, KD, STEERING_SIGN = .04, .015, 1  # same image, gains and left/right convention
TURN_PWM = 60  # trial PWM; measure turns using this value, on this floor
METRES_PER_TICK = None  # measured forward metres / mean signed encoder change
FORWARD_SIGNS = None  # measured (FR, FL, RR, RL), each +1 or -1
BRAKE_DISTANCE_M = None  # measured coast after Stop(), at BASE_SPEED on this floor
TURN_TICKS = {"LEFT": None, "RIGHT": None}  # command cutoff yielding 90 degrees AFTER coasting
NODE_APPROACH_M = 0.0  # optional measured camera blind gap before node; 0 keeps line-loss stopping
POSITION_TOLERANCE_M = .05  # requested accuracy, NOT a claim about real odometry
REST_SECONDS, NO_MOTION_SECONDS = .5, 5.  # encoder quiet / no feedback during commanded motion


def route():
    grid = GridMap()
    if HEADING not in ("N", "E", "S", "W"):
        raise ValueError("HEADING must be N/E/S/W")
    if isinstance(DESTINATION, str):
        target = dict(type="node", node=DESTINATION)
    else:
        a, b, metres = DESTINATION
        if b not in grid.adjacency.get(a, set()):
            raise ValueError("Destination endpoints must share a white 1 m edge")
        target = dict(type="edge", edge=(a, b), progress=metres/grid.length_m(a, b))
    plan = plan_mission_from_state(grid, dict(current_node=START_NODE,
                                  at_node=True, orientation=HEADING), target)
    if not plan["feasible"]:
        raise ValueError(plan["reason"])
    return grid, plan


def check_measurements(steps):
    needed = {s["action"] for s in steps} - {"STRAIGHT"}
    if "UTURN" in needed:
        needed.remove("UTURN"); needed.add("LEFT")  # two measured quarter turns
    if (METRES_PER_TICK is None or not np.isfinite(METRES_PER_TICK) or METRES_PER_TICK <= 0
            or FORWARD_SIGNS is None or len(FORWARD_SIGNS) != 4
            or any(s not in (-1, 1) for s in FORWARD_SIGNS)
            or BRAKE_DISTANCE_M is None or not np.isfinite(BRAKE_DISTANCE_M)
            or not 0 <= BRAKE_DISTANCE_M < 1):
        raise ValueError("Measure METRES_PER_TICK, FORWARD_SIGNS and BRAKE_DISTANCE_M first")
    if any(TURN_TICKS[t] is None or not np.isfinite(TURN_TICKS[t]) or TURN_TICKS[t] <= 0 for t in needed):
        raise ValueError("Measure the required LEFT/RIGHT TURN_TICKS first")
    if any(POSITION_TOLERANCE_M < s["cost"] <= BRAKE_DISTANCE_M for s in steps):
        raise ValueError("Target closer than measured braking distance: lower BASE_SPEED and remeasure")


def encoders(robot):
    # Professor's retrCmdl, unchanged. Commandes.cpp: N=front, n=rear counters.
    # GetSpeed() is NOT a position counter; its update is absent in the supplied firmware.
    from utilitaries import retrCmdl
    return np.array(retrCmdl(robot.serialport, b"N") +
                    retrCmdl(robot.serialport, b"n"), dtype=np.int64)


def difference(now, start):
    return (now-start+(1 << 31)) % (1 << 32)-(1 << 31)


def stop_and_wait(robot):
    robot.Stop()
    last, quiet = encoders(robot), time.monotonic()
    while time.monotonic()-quiet < REST_SECONDS:
        time.sleep(.02)
        now = encoders(robot)
        if np.any(difference(now, last)):
            quiet = time.monotonic()
        last = now
    return last  # encoder quiet, never inferred from the motor command ACK


def line_error(binary):
    # Exact three scan rows / nearest-group / mean from the working local huit.py.
    roi, w = binary[int(.7*len(binary)):], binary.shape[1]
    centers = []
    for fraction in (.9, .75, .6):
        white = np.flatnonzero(roi[int(fraction*len(roi))])
        if white.size:
            groups = np.split(white, np.flatnonzero(np.diff(white) > 1)+1)
            centers.append(min((int((g[0]+g[-1])/2) for g in groups), key=lambda x: abs(x-w/2)))
    return int(np.mean(centers))-w/2 if centers else None


def junction_row(mask, arms_required=3):
    """Three supported arms for X/T; two only when the map explicitly expects a corner."""
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
            if arms >= arms_required:
                # Require distinct white arcs: two Hough axes can describe one curve.
                branches = []
                for radius in (24, 30):
                    xy = np.rint(center+radius*ring_directions).astype(int)
                    ring = evidence[xy[:, 1], xy[:, 0]] > 0
                    ring = ring & np.roll(ring, 1)  # discard isolated noise hits
                    branches.append(np.count_nonzero(ring & ~np.roll(ring, 1)))
                if min(branches) >= arms_required:
                    return float(center[1]/h)
    return None


def turn(robot, action):
    if action == "STRAIGHT":
        return
    direction = "LEFT" if action == "UTURN" else action
    for _ in range(2 if action == "UTURN" else 1):
        start = last = encoders(robot)
        changed = time.monotonic()
        left, right = (-TURN_PWM, TURN_PWM) if direction == "LEFT" else (TURN_PWM, -TURN_PWM)
        robot.SetMotors(left, right)
        while True:
            now = encoders(robot)
            if np.mean(np.abs(difference(now, start))) >= TURN_TICKS[direction]:
                break
            if np.any(difference(now, last)):
                changed = time.monotonic()
            elif time.monotonic()-changed > NO_MOTION_SECONDS:
                raise RuntimeError("No encoder motion during turn")
            last = now
            time.sleep(.02)
        stop_and_wait(robot)


def drive_edge(robot, camera, step, corner=False):
    distance = step["cost"]
    start = last = encoders(robot)
    previous_error, moving, seen = 0., False, False
    changed = time.monotonic()
    votes = deque(maxlen=3)
    while True:
        now = encoders(robot)
        travelled = float(np.mean(difference(now, start)*FORWARD_SIGNS))*METRES_PER_TICK
        if travelled >= distance-BRAKE_DISTANCE_M:
            break
        if np.any(difference(now, last)) or not moving:
            changed = time.monotonic()
        elif time.monotonic()-changed > NO_MOTION_SECONDS:
            raise RuntimeError("No forward encoder feedback: check wheels/encoders")
        last = now
        try:
            image = camera.capture_array()
            if not isinstance(image, np.ndarray) or image.shape != (480, 640, 3) or image.dtype != np.uint8:
                raise ValueError("Invalid camera frame")
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  # exactly the working huit.py
            binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)[1]
        except (OSError, RuntimeError, ValueError, cv2.error) as error:
            robot.Stop(); moving, previous_error = False, 0.
            print("CAMERA ERROR - STOP REQUESTED:", error)
            continue  # only a fresh usable image can authorize movement again
        error = line_error(binary)
        # One observation per 1 m edge; ignore the departure crossing in its first half.
        # Camera observations never assert arrival or reset the measured distance.
        if step["next_node"] != "__GOAL__" and travelled >= .5 and not seen:
            votes.append(junction_row(cv2.resize(binary, (320, 240), interpolation=cv2.INTER_NEAREST),
                                      2 if corner else 3) is not None)
            if sum(votes) >= 2:
                seen = True
                print("GRID CORNER AHEAD:" if corner else "INTERSECTION AHEAD:", step["next_node"])
        if error is None and seen and distance-travelled <= NODE_APPROACH_M:
            error = previous_error = 0.  # measured final approach after the incoming line leaves view
        if error is None:
            if moving: robot.Stop()
            moving, previous_error = False, 0.
            continue  # line loss stops; a usable line permits recovery
        correction = KP*error+KD*(error-previous_error)
        left = int(np.clip(BASE_SPEED+STEERING_SIGN*correction, -MAX_SPEED, MAX_SPEED))
        right = int(np.clip(BASE_SPEED-STEERING_SIGN*correction, -MAX_SPEED, MAX_SPEED))
        robot.SetMotors(left, right)
        moving, previous_error = True, error
    final = stop_and_wait(robot)
    travelled = float(np.mean(difference(final, start)*FORWARD_SIGNS))*METRES_PER_TICK
    print("STOPPED (encoder estimate): %.3f / %.3f m" % (travelled, distance))
    if abs(travelled-distance) > POSITION_TOLERANCE_M:
        raise RuntimeError("Position outside requested tolerance; check encoder/braking calibration")
    return seen, travelled


def main():
    robot = camera = None
    try:
        grid, plan = route()
        steps = plan["detailed_steps"]
        print("ROUTE:", " -> ".join(plan["path"] or [START_NODE]))
        if not steps:
            print("ALREADY AT DESTINATION (configured start)")
            return
        check_measurements(steps)
        from picamera2 import Picamera2
        from arduino import arduino
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        camera.start()
        time.sleep(2)
        robot = arduino(PORT)
        if not robot.connect():
            raise RuntimeError("Arduino connection failed")
        stop_and_wait(robot)
        node, heading = START_NODE, HEADING
        for step in steps:
            turn(robot, step["action"])
            heading = step["heading"]  # calibrated turn estimate, not a compass measurement
            print("FOLLOWING:", step["segment"], "heading", heading)
            corner = len(grid.adjacency[step["next_node"]]) == 2 if step["next_node"] != "__GOAL__" else False
            seen, travelled = drive_edge(robot, camera, step, corner)
            if step["next_node"] == "__GOAL__":
                print("DESTINATION ON EDGE (estimate):", step["segment"], "distance", travelled)
            else:
                node = step["next_node"]
                print("ESTIMATED NODE:", node, "heading", heading,
                      "camera junction seen" if seen else "odometry only")
        print("DESTINATION STOPPED (estimated position)")
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("ERROR:", error)
    finally:
        if robot is not None:
            try: robot.Stop()
            except Exception: print("STOP FAILED - USE MOTOR POWER CUTOFF")
            try: robot.disconnect()
            except Exception: pass
        if camera is not None:
            try: camera.stop(); camera.close()
            except Exception: pass


if __name__ == "__main__":
    main()
