"""Camera-only Picamera2 runner: no serial import, motor connection or movement.

Run from repository root:
  python3 -m intersections.raspberry_camera --frames 300 --preview
Dependencies on supported Raspberry Pi OS: python3-picamera2, python3-opencv,
python3-numpy. This is a NEW backend; the supplied professor code has no camera
implementation. Confirm the installed OS/camera stack before using it.

640x360, 15 FPS and no flips are requested test settings, not measured robot
specifications. Inspect --preview for mounting orientation; --rotate180 is an
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
    """Explicit RGB888 stream = BGR NumPy pixels under Picamera2's naming.

    https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
    https://docs.libcamera.org/master/internal-api/namespacelibcamera_1_1controls.html
    SensorTimestamp and now() both use CLOCK_BOOTTIME; no clock-domain guess.
    Requires Picamera2 asynchronous wait(timeout=...) support (0.3.17 or later).
    """
    clock_id = "LINUX_BOOTTIME"
    now_ns = staticmethod(boottime_ns)

    def __init__(self, *, size=(640, 360), fps=15., camera_num=0, rotate180=False):
        from picamera2 import Picamera2
        from libcamera import Transform
        boottime_ns()  # Fail before opening a device on unsupported platforms.
        self.camera = Picamera2(camera_num=camera_num)
        self.rotate180 = rotate180
        self.started = False
        try:
            period = round(1e6/fps)
            configuration = self.camera.create_video_configuration(
                main={"size": tuple(size), "format": "RGB888"},
                transform=Transform(hflip=False, vflip=False),
                controls={"FrameDurationLimits": (period, period)},
                buffer_count=4, queue=False)
            self.camera.configure(configuration)
            self.configuration = self.camera.camera_configuration()
            if self.configuration["main"]["format"] != "RGB888":
                raise RuntimeError("camera negotiated an unexpected channel format")
        except Exception:
            self.camera.close()
            raise

    def start(self):
        self.camera.start()
        self.started = True

    def read(self, timeout=.25):
        job = self.camera.capture_request(wait=False)
        request = self.camera.wait(job, timeout=timeout)
        try:
            metadata = request.get_metadata()
            captured_ns = metadata.get("SensorTimestamp")
            if type(captured_ns) is not int or captured_ns <= 0:
                raise RuntimeError("camera lacks a valid SensorTimestamp")
            # Copy before releasing the camera-owned request buffers.
            image = to_bgr(request.make_array("main"), "BGR")
            width, height = self.configuration['main']['size']
            if image.shape != (height, width, 3):
                raise RuntimeError("camera array does not match the negotiated stream size")
        finally:
            request.release()
        if self.rotate180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        return image, captured_ns, metadata

    def close(self):
        try:
            if self.started:
                self.camera.stop()
        finally:
            self.camera.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="dependency check, opens no camera")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=float, default=15.)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--rotate180", action="store_true")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args(argv)
    if args.frames <= 0 or args.camera < 0 or min(args.width, args.height) < 64 or not 0 < args.fps <= 60:
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
        print(json.dumps(dict(camera_only=True, motor_connection=False,
                  configuration=source.configuration, properties=source.camera.camera_properties,
                  pixel_order="BGR", rotate180=args.rotate180), default=str), flush=True)
        source.start()
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
