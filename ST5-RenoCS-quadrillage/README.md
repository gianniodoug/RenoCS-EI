# RenoCS quadrillage deployment

One shared 640x480 Picamera2 stream; one A22 serial owner. The robot follows the
white line, confirms X/T, sends Stop and latches. No navigation or restart API.
Real mode also latches line loss and critical faults. UNKNOWN pauses movement;
persistent UNKNOWN escalates through the existing T.9 policy. No other robot
controller may run simultaneously. No original controller or firmware is needed
at runtime: bluno_backend.py implements the team-confirmed professor A22 wire API.

## Raspberry Pi prerequisites

Use the same system Python/camera environment that runs huit.py. If missing:

```sh
sudo apt install python3-picamera2 python3-opencv python3-numpy python3-serial
```

Python >=3.9 on Raspberry Pi OS/Linux with CLOCK_BOOTTIME. Picamera2 must expose
capture_arrays(["main"]) and metadata SensorTimestamp. RGB888 arrays are BGR.
Preview configuration and two-second warm-up match huit.py; capture_arrays is
capture_array's synchronous same-request image+metadata equivalent:
https://github.com/raspberrypi/picamera2/blob/main/picamera2/picamera2.py

Stop huit.py, IDE serial monitors and other camera/serial users. Identify the
correct Bluno device (default /dev/ttyACM0); obtain normal user serial access
(e.g. dialout group), and log in again if membership changed. Do not blindly use
sudo or change device permissions to mask a wrong device or competing owner.

## Motor power OFF: camera dry-run

From this folder, with motor power disconnected:

```sh
python3 -u run_quadrillage.py --dry-run --live-camera --frames 300 --preview
```

Omit --preview for a headless terminal. Check upright view, line in lower ROI,
correct X/T, fresh timestamps, processing latency, and absence of camera faults.
Move the unpowered robot by hand toward actual X/T: confirm exactly one event
and a latched stop request. This is not physical motor stopping validation.

## REAL motors: supervised low-PWM first test

The operator must FIRST test an immediate independent MOTOR-POWER cutoff and
keep a hand on it throughout the test. The flag below is an operator attestation,
not an electronic interlock and not a claim that a watchdog is installed.

```sh
python3 -u run_quadrillage.py --enable-motors --confirm-power-cutoff --port /dev/ttyACM0 --base-pwm 45 --pwm-limit 60 --max-seconds 30
```

Camera warms up and produces frames before the serial connection is opened.
A22 handshake and initial zero targets must ACK before movement. Signed PWM cap
is 60/255, base 45/255; these are deliberately limited test values, NOT measured
speed or proof of sufficient motor torque. KP=.04, KD=.015 and line geometry are
unchanged. Limit applies after the unchanged PD calculation; no PWM above 60.
If torque is insufficient or steering is wrong, cut power and review settings;
do not increase limits while the robot is running. Real-test CLI rejects caps
above 80. No encoder-based stopping verification is implemented.

First verify wheel directions and zero targets with wheels clear of the ground.
Then use a clear, level section with ample approach distance, a spotter and one
intersection. Confirm actual wheel/body rest BEFORE the crossing, measure the
margin, and keep observing after the crossing disappears from view. Repeat X/T
separately only after a deliberate stationary restart. Cover/remove the line to
verify a latched Stop. Never test USB loss or Pi freeze while freely moving.
If an ACK fails, UNKNOWN persists, camera stops, motion is unexpected, or the
robot does not stop promptly: use the MOTOR-POWER cutoff immediately.

Ctrl+C, SIGTERM, q (with --preview), the 30 s limit and errors request bounded
Stop and cleanup. They are not replacements for motor-power removal. No software
can deliver Stop over a lost USB link, and killing the process is not a reliable
physical emergency stop. No automatic reconnect/rearm/recovery is performed.

## Timing and validation limits

Camera startup deadline 12 s (includes 2 s warm-up), per-read wait 250 ms,
frame/processing freshness limit 250 ms, bounded camera cleanup 500 ms. Camera
calls have one daemon owner; a blocked native driver is reported without holding
up the main Stop path. Serial connect budget 4 s (includes 2 s Bluno boot wait),
transaction budget 100 ms, failed-worker cleanup at most two 50 ms joins. OS
scheduling is not hard real-time. Failed serial sessions are poisoned: no blind
retries or automatic reconnect. Normal intersection stops remain distinct from
fault stops; neither can be overridden by the follower.

The original watchdog is nonfunctional; a separate 350 ms firmware candidate is
not confirmed flashed. Both firmware variants share the same connection banner.
Do not rely on that watchdog. If commissioning it later, verify actual timeout
and physical rest, and measure full Pi control-loop latency with margin (health
probes do not refresh the candidate motor watchdog).

Software-only tests cover fake camera startup/stall/shutdown, A22 wire format,
timeouts, owner exclusivity, low-PWM proposals, X/T stop latching, line loss and
fault priority. This package has NOT been run against physical hardware here.
The camera-timeout correction, real-camera detection distances, motor torque,
stop distance and power-cutoff effectiveness require supervised Pi verification.
A Stop ACK proves a protocol reply, never physical rest. Do not attempt a floor
run until the camera dry-run and independent power-cutoff checks pass.
