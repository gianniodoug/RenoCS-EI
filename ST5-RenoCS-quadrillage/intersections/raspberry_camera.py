"""Camera-only Picamera2 runner: no serial import, motor connection or movement.

Run from repository root:
  python3 -m intersections.raspberry_camera --frames 300 --preview
Dependencies on supported Raspberry Pi OS: python3-picamera2, python3-opencv,
python3-numpy. This is a NEW backend; the supplied professor code has no camera
implementation. Confirm the installed OS/camera stack before using it.

Preview RGB888 follows huit.py; no frame-rate override by default.
Camera-only CLI size/rotation are explicit test settings. Inspect --preview for mounting orientation; --rotate180 is an
explicit correction. Configured stream, sensor properties and timing are printed.
"""
import argparse
import importlib.util
import json
import platform
import time

import cv2
import numpy as np

from .physical_adapter import PhysicalIntersectionAdapter, to_bgr


def boottime_ns():
    if not hasattr(time, "CLOCK_BOOTTIME"):
        raise RuntimeError("Picamera2 SensorTimestamp requires Linux CLOCK_BOOTTIME")
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


class Picamera2Frames:
    """One camera owner, matching huit.py preview/RGB888/2 s warm-up.

    capture_arrays(["main"]) is the synchronous capture_array equivalent that
    also returns metadata from the SAME request (Picamera2 official API).
    RGB888 memory is BGR. No separate capture_metadata/timestamp guessing.
    A daemon acquisition thread owns ALL camera calls. The control thread waits
    only on a latest-frame mailbox: startup <=12 s, read <= caller timeout,
    shutdown join <=0.5 s. A stalled camera cannot block motor Stop/shutdown.
    A stuck native driver may leave the daemon alive until process exit; report
    that error and use the physical cutoff. This is not a hardware watchdog.
    """
    clock_id = "LINUX_BOOTTIME"
    now_ns = staticmethod(boottime_ns)

    def __init__(self, *, size=(640, 480), fps=None, camera_num=0, rotate180=False,
                 startup_timeout_s=12., close_timeout_s=.5, _camera_factory=None,
                 _now_ns=None, _warmup_s=2.):
        import math
        import queue
        import threading
        if (len(size) != 2 or any(type(x) is not int or x < 64 for x in size)
                or type(camera_num) is not int or camera_num < 0):
            raise ValueError("valid camera index and stream size required")
        for x in (startup_timeout_s, close_timeout_s):
            if not isinstance(x, (int,float)) or isinstance(x,bool) or not math.isfinite(x) or not 0 < x <= 30:
                raise ValueError("finite positive camera deadline <=30 s required")
        if not isinstance(_warmup_s,(int,float)) or not math.isfinite(_warmup_s) or not 0 <= _warmup_s < startup_timeout_s:
            raise ValueError("warm-up must fit inside startup deadline")
        if fps is not None and (not isinstance(fps,(int,float)) or not math.isfinite(fps) or not 0 < fps <= 60):
            raise ValueError("FPS in (0,60] required")
        self.now_ns = _now_ns or boottime_ns
        self.now_ns()  # Verify clock support before any device access.
        self.size, self.fps, self.camera_num = tuple(size), fps, camera_num
        self.rotate180 = rotate180
        self.startup_timeout_s, self.close_timeout_s = startup_timeout_s, close_timeout_s
        self._factory, self._warmup_s = _camera_factory, _warmup_s
        self._frames = queue.Queue(maxsize=1)
        self._ready, self._stop = threading.Event(), threading.Event()
        self._thread = None
        self._error = self._close_error = None
        self._closed = False
        self.camera = self.configuration = None
        self.started = False

    def _acquire(self):
        import queue
        started = False
        try:
            factory = self._factory
            if factory is None:
                from picamera2 import Picamera2
                factory = Picamera2
            self.camera = factory(camera_num=self.camera_num)
            options = dict(main={"format": "RGB888", "size": self.size})
            if self.fps is not None:
                period = round(1e6/self.fps)
                options['controls'] = {"FrameDurationLimits": (period,period)}
            self.camera.configure(self.camera.create_preview_configuration(**options))
            self.configuration = self.camera.camera_configuration()
            if (self.configuration['main']['format'] != 'RGB888'
                    or tuple(self.configuration['main']['size']) != self.size):
                raise RuntimeError("camera negotiated an unexpected stream")
            if self._stop.is_set():
                return
            self.camera.start()
            started = True
            warm_until = time.monotonic()+self._warmup_s
            previous = None
            while not self._stop.is_set():
                arrays, metadata = self.camera.capture_arrays(["main"])
                captured_ns = metadata.get('SensorTimestamp')
                if type(captured_ns) is not int or captured_ns <= 0:
                    raise RuntimeError('camera lacks a valid SensorTimestamp')
                if previous is not None and captured_ns <= previous:
                    raise RuntimeError('camera repeated or reversed SensorTimestamp')
                previous = captured_ns
                if time.monotonic() < warm_until:
                    continue
                image = to_bgr(arrays[0], 'BGR')
                if image.shape != (self.size[1],self.size[0],3):
                    raise RuntimeError('camera array does not match negotiated stream')
                if self.rotate180:
                    image = cv2.rotate(image, cv2.ROTATE_180)
                item = (image,captured_ns,metadata)
                try:
                    self._frames.put_nowait(item)
                except queue.Full:
                    try:
                        self._frames.get_nowait()  # Drop old frames, never queue a backlog.
                    except queue.Empty:
                        pass
                    self._frames.put_nowait(item)
                self._ready.set()  # First post-warm-up actual frame, not merely camera.start().
        except Exception as error:
            self._error = error
        finally:
            if self.camera is not None:
                try:
                    if started:
                        self.camera.stop()
                except Exception as error:
                    self._close_error = error
                finally:
                    try:
                        self.camera.close()
                    except Exception as error:
                        self._close_error = error
            self._ready.set()

    def start(self):
        import threading
        if self._thread is not None or self._closed:
            raise RuntimeError('one camera session only; no automatic restart')
        self._thread = threading.Thread(target=self._acquire, name='RenoCS-camera', daemon=True)
        self._thread.start()
        if not self._ready.wait(self.startup_timeout_s):
            self._stop.set()
            raise TimeoutError('CAMERA_STARTUP_TIMEOUT: no post-warm-up frame')
        if self._error is not None:
            raise OSError('CAMERA_STARTUP_FAILURE: '+str(self._error)) from self._error
        self.started = True

    def read(self, timeout=.25):
        import math
        import queue
        if not self.started or self._closed:
            raise RuntimeError('start one camera session before reading')
        if not isinstance(timeout,(int,float)) or isinstance(timeout,bool) or not math.isfinite(timeout) or not 0 < timeout <= 30:
            raise ValueError('finite positive camera timeout required')
        if self._error is not None:
            raise OSError('CAMERA_CAPTURE_FAILURE: '+str(self._error)) from self._error
        try:
            item = self._frames.get(timeout=timeout)
        except queue.Empty:
            if self._error is not None:
                raise OSError('CAMERA_CAPTURE_FAILURE: '+str(self._error)) from self._error
            raise TimeoutError('CAMERA_TIMEOUT: no new frame within deadline') from None
        if self._error is not None:
            raise OSError('CAMERA_CAPTURE_FAILURE: '+str(self._error)) from self._error
        return item

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.close_timeout_s)
            if self._thread.is_alive():
                raise OSError('camera driver still blocked; bounded shutdown, use motor-power cutoff')
        self.started = False
        if self._close_error is not None:
            raise OSError('CAMERA_CLOSE_FAILURE: '+str(self._close_error))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="dependency check, opens no camera")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--rotate180", action="store_true")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args(argv)
    if args.frames <= 0 or args.camera < 0 or min(args.width, args.height) < 64 or (args.fps is not None and not 0 < args.fps <= 60):
        parser.error("positive frame count, image sides >=64 and FPS in (0,60] required")
    if args.check:
        available = {n: importlib.util.find_spec(n) is not None
                     for n in ("cv2", "numpy", "picamera2", "libcamera")}
        print(json.dumps(dict(python=platform.python_version(), packages=available,
                             boottime=hasattr(time, "CLOCK_BOOTTIME"), hardware_opened=False)))
        return 0 if all(available.values()) and hasattr(time, "CLOCK_BOOTTIME") else 2
    source = None
    adapter = None
    durations = []
    started = time.perf_counter()
    try:
        source = Picamera2Frames(size=(args.width,args.height), fps=args.fps,
                                camera_num=args.camera, rotate180=args.rotate180)
        adapter = PhysicalIntersectionAdapter(run_id="pi-camera-only",
                    clock_id=source.clock_id, now_ns=source.now_ns)
        source.start()
        print(json.dumps(dict(camera_only=True, motor_connection=False,
                  configuration=source.configuration, properties=source.camera.camera_properties,
                  pixel_order="BGR", rotate180=args.rotate180), default=str), flush=True)
        for seq in range(1, args.frames+1):
            image, captured_ns, metadata = source.read()
            result = adapter.process_frame(image, pixel_format="BGR", captured_ns=captured_ns,
                                           seq=seq, clock_id=source.clock_id)
            durations.append(result["processing_ms"])
            print(json.dumps(dict(seq=seq, shape=list(image.shape), captured_ns=captured_ns,
                      sensor_frame_us=metadata.get("FrameDuration"), result=result)), flush=True)
            if args.preview:
                shown = image.copy()
                observation = result["observation"] or {}
                cv2.putText(shown, f"{observation.get('kind','INVALID')} {result['stop_class'] or ''}",
                            (8,25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,220,255), 2)
                cv2.imshow("RenoCS CAMERA ONLY - no motor connection", shown)
                if cv2.waitKey(1) & 255 == ord('q'):
                    break
            if result["stop_class"] == "SAFETY":
                return 2
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        result = adapter.camera_failed() if adapter is not None else None
        print(json.dumps(dict(error=str(error), camera_only=True, stop_request=result)), flush=True)
        return 2
    finally:
        if source is not None:
            source.close()
        if args.preview:
            cv2.destroyAllWindows()
        values = [x for x in durations if x is not None]
        if values:
            print(json.dumps(dict(frames=len(values), processing_median_ms=float(np.median(values)),
                      processing_p95_ms=float(np.percentile(values,95)),
                      observed_loop_fps=len(values)/max(.001,time.perf_counter()-started),
                      platform=platform.platform(), physical_stop_tested=False)), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
