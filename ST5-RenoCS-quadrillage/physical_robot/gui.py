"""Thin graphical controls for the EXISTING Controller, not another controller.

Tk owns the UI thread. One background runner owns Controller. STOP sets a shared
event: the existing serial worker also observes it independently of inference,
attempts bounded zero PWM and permanently rejects subsequent movement. A reply
is never presented as evidence of physical rest. No hardware opens on import.
"""
import importlib.util
import json
import logging
import multiprocessing
import queue
import threading
import time


class StopSignal:
    """Monotonic shared byte: no reset and no interprocess lock to orphan.

    The serial worker may be terminated on a timeout. An Event/Condition mutex
    held by that dead worker must not be able to block the graphical STOP path.
    Only a single byte is stored, always 0 -> 1; no compound shared update.
    """
    def __init__(self, context):
        self.flag = context.RawValue('b', 0)
    def set(self):
        self.flag.value = 1
    def is_set(self):
        return bool(self.flag.value)
    def wait(self, timeout=None):
        deadline = None if timeout is None else time.monotonic()+timeout
        while not self.is_set():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            time.sleep(.005)
        return True


def production_controller(stop_event, stop_status):
    # Deliberately lazy: the window opens without importing OpenCV or devices.
    import cv2
    from intersections.raspberry_camera import Picamera2Frames
    from .bluno_backend import A22MotorBackend, BlunoConfig, HardwareAuthorization
    from .main import Controller
    cv2.setNumThreads(1)
    source = Picamera2Frames(size=(640, 480))
    authorization = HardwareAuthorization('', '', True, True,
        'Team-confirmed A22; operator confirmed tested independent motor-power cutoff', 'professor_A22')
    backend = A22MotorBackend(clock_id=source.clock_id, now_ns=source.now_ns,
        config=BlunoConfig(port='/dev/ttyACM0', pwm_limit=60),
        authorization=authorization, allow_hardware=True,
        stop_event=stop_event, stop_status=stop_status)
    return Controller(source, motors=backend, enable_real_motors=True,
        base_pwm=45, pwm_limit=60, stop_event=stop_event, processing_preflight=True)


def readable_error(result):
    reason = result.get('reason') or result.get('error') or ''
    if not reason:
        reasons = result.get('decision', {}).get('latched_faults', [])
        reason = reasons[0] if reasons else 'Unknown controller error'
    messages = {
        'PROCESSING_DEADLINE_MISSED': 'Image processing exceeded 250 ms',
        'CAMERA_STATUS_STALE_OR_INVALID': 'Camera processing or communication took too long',
        'CAMERA_TIMESTAMP_STALE_OR_INVALID': 'Camera timestamp is stale or invalid',
        'CAMERA_TIMEOUT': 'No new camera frame',
        'CAMERA_OR_PROCESSING_FAILURE': 'Camera or image processing failed',
        'LINE_LOST': 'White line lost',
        'SERIAL_TIMEOUT': 'Bluno did not reply in time',
        'SERIAL_LOST': 'Bluno communication lost',
        'SERIAL_PROTOCOL_ERROR': 'Unexpected Bluno reply',
        'PERSISTENT_UNKNOWN': 'Track view remained uncertain',
    }
    return messages.get(reason, str(reason)[:130])


class RobotSession:
    """Testable UI model; no Tk and no device access until confirmed start()."""
    def __init__(self, *, controller_factory=production_controller, context=None, logger=None):
        self.factory = controller_factory
        self.context = context or multiprocessing.get_context('spawn')
        self.stop_event = StopSignal(self.context)
        self.stop_status = self.context.RawValue('i', 0)  # One writer, serial worker.
        self.logger = logger or logging.getLogger('renocs')
        self.messages = queue.SimpleQueue()
        self.thread = self.controller = None
        self.timer = None
        self.started = False
        self.summary = None
        self.stop_requested_at = None
        self.stop_reason = None
        self._last_state = None
        self.last_fault = None

    @property
    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, confirmed):
        if not confirmed:
            return False
        if self.started:
            raise RuntimeError('This run is latched. Close and reopen only after checking physical rest.')
        self.started = True
        self.messages.put('Checking camera and processing; motors remain disabled until checks pass...')
        self.thread = threading.Thread(target=self._run, name='RenoCS-controller', daemon=True)
        # Also enforce the limit outside inference/control. If the detector is
        # blocked, the serial owner still sees the shared Stop event.
        self.timer = threading.Timer(30., self.stop, args=('TIME_LIMIT',))
        self.timer.daemon = True
        self.timer.start()
        self.thread.start()
        return True

    def stop(self, reason='OPERATOR_STOP'):
        if self.stop_requested_at is None:
            self.stop_requested_at = time.monotonic()
            self.stop_reason = reason
        self.stop_event.set()  # No serial I/O or join in the UI callback.
        self.messages.put('STOP requested. Verify physical rest; use motor-power cutoff if moving.')

    def _result(self, result):
        self.logger.debug('FRAME %s', json.dumps(result, default=str))
        state = (result.get('priority'), result.get('intersection_kind'), result.get('reason'))
        if state == self._last_state or self.stop_event.is_set():
            return
        self._last_state = state
        if result.get('fault_state') == 'SAFE_STOP':
            text = readable_error(result) + '. Stop requested; use cutoff if moving. See log.'
            self.last_fault = text
        elif result.get('priority') == 'INTERSECTION':
            text = '{} confirmed: zero targets latched. Verify physical rest.'.format(result.get('confirmed_kind') or result.get('intersection_kind'))
        elif result.get('priority') == 'DEGRADED':
            text = 'Uncertain camera view: movement paused.'
        elif result.get('priority') == 'LINE_FOLLOWING':
            text = 'Following white line. STOP and motor-power cutoff remain available.'
        else:
            text = 'Stop requested; verify physical rest.'
        self.messages.put(text)

    def _run(self):
        try:
            if self.stop_event.is_set():
                return
            self.logger.info('START authorized: real A22 /dev/ttyACM0; base PWM45, cap60, maximum30s; cutoff confirmed')
            self.controller = self.factory(self.stop_event, self.stop_status)
            self.summary = self.controller.run(frames=1_000_000_000, max_seconds=30., on_result=self._result)
            self.logger.info('END %s', json.dumps(self.summary, default=str))
            if self.summary.get('shutdown', {}).get('errors'):
                self.messages.put('Shutdown could not be fully confirmed. Use motor-power cutoff; see log.')
            elif self.summary.get('exit_code'):
                # Preserve the short fault message from _result, no fake success.
                self.messages.put(self.last_fault or 'Run ended on a fault. Zero targets requested; verify rest. See log.')
            else:
                self.messages.put('Run ended; zero targets requested. Verify physical rest. Restart is locked.')
        except Exception as error:
            self.logger.exception('Controller could not run')
            self.messages.put(str(error)[:140] + '. Motors must remain off; use cutoff if moving. See log.')
        finally:
            if self.timer is not None:
                self.timer.cancel()
            self.logger.info('STOP signal monotonic=%s reason=%s protocol_status=%s (not physical rest)',
                             self.stop_requested_at, self.stop_reason, self.stop_status.value)
            # run() normally closes itself. This also covers factory/run errors.
            if self.controller is not None:
                try:
                    self.logger.info('CLEANUP %s', self.controller.shutdown())
                except Exception:
                    self.logger.exception('Cleanup failed; independent motor-power cutoff required')
                    self.messages.put('Shutdown failed. Use motor-power cutoff now; see log.')

    def latest(self):
        latest = None
        while True:
            try:
                latest = self.messages.get_nowait()
            except queue.Empty:
                break
        if self.stop_event.is_set() and self.active:
            if self.stop_status.value == 3:
                return 'Stop communication failed. Use MOTOR-POWER CUTOFF now.'
            if self.stop_status.value == 2:
                return 'Zero command replied; physical rest is NOT verified. Check wheels; use cutoff if moving.'
            if self.stop_requested_at is not None and time.monotonic()-self.stop_requested_at > 1.:
                return 'STOP pending. Use MOTOR-POWER CUTOFF if the robot is moving.'
        return latest


class RobotWindow:
    def __init__(self, root, session, log_path, *, confirm=None):
        import tkinter as tk
        from tkinter import messagebox
        self.root, self.session = root, session
        self.confirm = confirm or messagebox.askyesno
        self.closing = False
        root.title('RenoCS quadrillage — REAL ROBOT')
        root.geometry('560x260')
        root.resizable(False, False)
        self.status = tk.StringVar(value='Ready. START requires motor-power cutoff confirmation.')
        tk.Label(root, text='REAL ROBOT · PWM 45 / limit 60 · 30 seconds', font=('sans', 13, 'bold')).pack(pady=12)
        buttons = tk.Frame(root); buttons.pack()
        self.start_button = tk.Button(buttons, text='START', width=15, bg='#267b37', fg='white', command=self.start)
        self.start_button.pack(side='left', padx=12)
        self.stop_button = tk.Button(buttons, text='STOP', width=15, bg='#b42025', fg='white', command=self.stop)
        self.stop_button.pack(side='left', padx=12)
        tk.Label(root, textvariable=self.status, wraplength=530, justify='center', height=4).pack(pady=8)
        tk.Label(root, text='Physical stopping is not verified by a serial reply.', fg='#9b2323').pack()
        tk.Label(root, text='Detailed log: '+str(log_path), wraplength=530, font=('sans', 8)).pack(pady=5)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.poll)

    def start(self):
        if self.session.started:
            return
        confirmed = self.confirm('Enable REAL motors?',
            'The robot will move for up to 30 seconds.\n\n'
            'Is the track clear, and have you TESTED the independent motor-power cutoff?\n'
            'Keep an operator ready at that cutoff.\n\nEnable real motors now?', parent=self.root)
        if confirmed and self.session.start(True):
            self.start_button.configure(state='disabled')

    def stop(self):
        if self.session.started:
            self.session.stop()
            self.status.set('STOP requested. Use the motor-power cutoff if the robot is moving.')

    def close(self):
        if self.session.active:
            self.closing = True
            self.stop()
        else:
            self.root.destroy()

    def poll(self):
        latest = self.session.latest()
        if latest:
            self.status.set(latest)
        if self.closing and not self.session.active:
            self.root.destroy()
            return
        self.root.after(50, self.poll)


def main(folder, log_path):
    import tkinter as tk
    from tkinter import messagebox
    import os
    import sys
    logging.basicConfig(filename=str(log_path), level=logging.DEBUG,
        format='%(asctime)s %(levelname)s %(threadName)s %(message)s', force=True)
    root = tk.Tk()
    logger = logging.getLogger('renocs')
    logger.info('Python %s; folder %s; 250ms freshness retained', sys.executable, folder)
    missing = [name for name in ('numpy','cv2','picamera2','libcamera','serial')
               if importlib.util.find_spec(name) is None]
    if missing:
        messagebox.showerror('Missing Raspberry Pi packages',
            'System Python is missing: '+', '.join(missing)+'.\nSee README for Raspberry Pi OS packages.', parent=root)
        root.destroy()
        return 2
    # One UI/controller per user, including copies in different folders.
    import fcntl
    lock = open('/tmp/renocs-quadrillage-'+str(os.getuid())+'.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        messagebox.showerror('RenoCS already open', 'Use the existing RenoCS START/STOP window.', parent=root)
        lock.close();root.destroy();return 2
    try:
        session = RobotSession(logger=logger)
        window = RobotWindow(root, session, log_path)
        import signal
        previous = signal.signal(signal.SIGTERM, lambda *_: root.after(0, window.close))
        try:
            root.mainloop()
        finally:
            signal.signal(signal.SIGTERM, previous)
            if session.active:
                session.stop()
                session.thread.join(timeout=2.)
                if session.active:
                    logger.error('Controller cleanup still pending: independent motor-power cutoff required')
    finally:
        lock.close()
    return 0
