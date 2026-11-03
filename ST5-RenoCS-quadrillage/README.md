# RenoCS physical quadrillage robot

## Normal use: no terminal

1. Copy this WHOLE folder to the Raspberry Pi Desktop. Open it and double-click
   **START_ROBOT.desktop**. You may also copy that launcher file onto the Desktop
   beside this folder; it locates the folder automatically.
2. If Raspberry Pi OS asks, choose **Execute / Allow Launching**. If it opens as
   text, use File Manager > Properties > Permissions > allow execution first.
   Git records the launcher as executable; ZIP/Windows copies can lose that bit.
3. The small window opens with START and STOP. START asks you to confirm that
   the independent MOTOR-POWER cutoff has been tested and an operator is ready
   at it. No camera or serial connection opens before confirmation.
4. The existing controller checks the camera and processing speed at rest, then
   uses real A22 `/dev/ttyACM0`, base PWM **45**, absolute PWM cap **60**. The run
   has an independent **30-second limit**, including startup, plus the normal
   controller limit. These are PWM limits, not calibrated physical speed.
5. A confirmed X/T, lost line or critical fault sends zero targets and latches.
   STOP also latches; it works while image processing is busy. There is no
   navigation, automatic restart or resume. Close and reopen deliberately only
   after checking physical rest. Closing the active window requests Stop first.

**Use the physical motor-power cutoff immediately if movement is unexpected or
stopping is uncertain. A serial reply never proves that the robot has stopped.**
The window always asks you to verify physical rest; it never claims to measure it.
Do not rely on an unverified firmware watchdog, USB removal or killing Python.

## One-time Raspberry Pi setup

The launcher explicitly uses **/usr/bin/python3**, the Raspberry Pi OS system
interpreter, not an IDE/desktop PATH or a Windows virtual environment. It uses
its installed packages. Through the graphical Add/Remove Software application,
ensure these Raspberry Pi OS packages exist if the launcher reports them missing:
`python3-tk`, `python3-picamera2`, `python3-opencv`, `python3-numpy`, `python3-serial`
(Picamera2 supplies/requires libcamera). Python 3.9+ is required. The normal user
must have serial-device permission (typically dialout); log out/in if changed.
Stop huit.py, Arduino IDE serial monitors and all other robot/camera programs.
Only one RenoCS window per user may run at a time. No firmware change is needed.

Keep `START_ROBOT.desktop`, `launch_robot.py`, `physical_robot/` and
`intersections/` together; paths are resolved from the launcher, including spaces.
The existing `run_quadrillage.py` command entry remains available for developers.
The self-contained modules need no files elsewhere in the team repository.

## What changed for the Pi processing failure

`PROCESSING_DEADLINE_MISSED` means a frame passed the input timestamp check but
its age exceeded **250 ms after intersection processing**. SensorTimestamp and
now() remain CLOCK_BOOTTIME; capture time is never replaced with receipt time.
The live 250 ms checks and tracker thresholds have NOT been relaxed.

Compared with the lightweight huit.py loop, the old integrated path also ran
connected components, Hough candidates and four-arm/three-arm geometric checks.
It did this cold after opening the motor port. The correction keeps the same
640x480 preview/RGB888 BGR stream and two-second camera warm-up, but:

- removes redundant resizing and per-component full-frame mask scans;
- batches duplicate-axis and four-ray calculations, caches ring geometry, and
  replaces tiny repeated matrix solves with the same intersection formula;
- skips unused ring calculations for candidates with fewer than three supported
  rays (debug visualizations still calculate full evidence);
- uses one OpenCV/BLAS thread to avoid oversubscription;
- warms up inference on five real camera frames BEFORE opening serial; the last
  three must each complete within 200 ms of capture, leaving 50 ms of margin.
  Slow warm processing is shown as an error with motors still disabled.

This identifies the software failure condition and removes measured host-side
bottlenecks. The exact cold/warm timing on the Pi 3 B+ has NOT been measured here.
If the warm timing gate fails, do not bypass it: inspect the recorded timings.
The line-following scan algorithm, PD gains, X/T thresholds, temporal tracker,
A22 byte protocol and latched-stop policy are preserved. There is no silent
image downsampling or deadline increase. All consumers use the same camera frame.

## STOP, logs and remaining physical verification

The UI never owns serial. STOP sets a shared event observed by the existing
serial owner every 20 ms, independently of inference. It attempts the existing
zero-motor command with a 100 ms transaction budget (after any current bounded
transaction) and permanently rejects later movement. An independent 30 s timer
sets the same event. UI callbacks never wait on camera, serial, inference or join.
OS/USB faults still prevent a hard real-time guarantee or physical stop guarantee.
If the command cannot be confirmed or the controller is busy, the window says to
use the motor-power cutoff. No failed link is blindly retried or auto-reconnected.

Detailed startup, camera clock/configuration, cold/warm processing age, per-frame
results, serial failures and shutdown information go to `logs/robot-*.log`, not
hundreds of terminal messages. Logs/caches are ignored and are not shipped.
The short window error remains specific (camera, processing, line loss or serial).

Software tests use fake camera/serial devices, including the actual serial-worker
process/Pipe, a blocked inference test, UI buttons/confirmation and an independent
run-limit test. Existing X/T, tracker, adapter, T.9 and physical-controller tests
and the recorded 900-frame replay are regressions, not physical validation.
On the actual Pi, verify startup timing, camera mounting/orientation, real-grid
X/T observations, wheel directions/torque at low PWM and actual stopping margin
BEFORE the intersection. First test with wheels raised, then one clear approach
with a spotter at the tested power cutoff. Acknowledge only what was observed.
