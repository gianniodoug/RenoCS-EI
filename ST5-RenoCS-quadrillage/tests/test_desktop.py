"""Fake cameras/serial ONLY. No hardware connections, GUI activation or firmware."""
import configparser
import contextlib
import functools
import io
import multiprocessing
from pathlib import Path
import queue
import shlex
import struct
import threading
import time
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from physical_robot.bluno_backend import A22MotorBackend, BlunoConfig, HardwareAuthorization
from physical_robot.gui import RobotSession, RobotWindow, production_controller, readable_error
from physical_robot.main import Controller
from intersections.physical_adapter import PhysicalIntersectionAdapter

cv2.setNumThreads(1)


def image():
    result = np.zeros((480,640,3),np.uint8)
    cv2.line(result,(320,0),(320,479),(235,235,235),8)
    return result


class Frames:
    clock_id = 'MOCK_MONOTONIC'
    now_ns = staticmethod(time.monotonic_ns)
    def __init__(self): self.closed=False;self.started=False;self.reads=0
    def start(self): self.started=True
    def read(self, timeout=.25):
        self.reads+=1
        return image(),self.now_ns(),{}
    def close(self): self.closed=True


class FakeSerial:
    def __init__(self, *, records, fail_stop=False, **kwargs):
        self.records=records;self.rx=bytearray();self.timeout=self.write_timeout=.1
        self.fail_stop=fail_stop;self.moving=False
    @property
    def in_waiting(self):return len(self.rx)
    def reset_input_buffer(self):self.rx.clear()
    def write(self, payload):
        self.records.put(bytes(payload))
        if self.fail_stop and self.moving and payload==b'C'+bytes(8):
            raise OSError('mock serial lost during operator Stop')
        if payload==b'A22': reply=b'OK Arduino connecte version 1.0 en mode 2\r\n'
        elif payload==b'B':reply=b'OK\r\n'
        else:
            assert payload[:1]==b'C' and len(payload)==9
            self.moving=any(struct.unpack('<4h',payload[1:]))
            reply=('OK Moteurs mis aux consignes : '+' '.join(map(str,struct.unpack('<4h',payload[1:])))+'\r\n').encode()
        self.rx.extend(reply);return len(payload)
    def read(self, count):
        if self.rx:return bytes([self.rx.pop(0)])
        time.sleep(min(self.timeout,.005));return b''
    def close(self):pass


class FakeSerialContext:
    def __init__(self, fail_stop=False):
        self.context=multiprocessing.get_context('spawn')
        self.records=self.context.Queue()
        self.fail_stop=fail_stop
    def Pipe(self):return self.context.Pipe()
    def Process(self, **kwargs):
        kwargs['args']=(*kwargs['args'],functools.partial(FakeSerial,records=self.records,fail_stop=self.fail_stop))
        return self.context.Process(**kwargs)


def authorization():
    return HardwareAuthorization('','',True,True,'MOCK evidence, NOT hardware validation','professor_A22')


class StopOwnerTests(unittest.TestCase):
    def test_async_stop_failure_is_reported_and_never_claims_rest(self):
        context=FakeSerialContext(fail_stop=True);event=context.context.Event();status=context.context.RawValue('i',0)
        source=Frames()
        backend=A22MotorBackend(clock_id=source.clock_id,now_ns=source.now_ns,
            config=BlunoConfig(port='MOCK_LOST',pwm_limit=60,boot_wait_s=0,transaction_timeout_s=.3),
            authorization=authorization(),allow_hardware=True,_context=context,stop_event=event,stop_status=status)
        ctl=Controller(source,motors=backend,enable_real_motors=True,base_pwm=45,pwm_limit=60,stop_event=event)
        try:
            ctl.start();ctl.step();ctl.request_stop()
            limit=time.monotonic()+1
            while status.value!=3 and time.monotonic()<limit:time.sleep(.005)
            self.assertEqual(status.value,3)
            result=ctl.shutdown()
            self.assertTrue(result['errors'])
            self.assertFalse(result['stop_acknowledged']);self.assertFalse(result['physical_stop_verified'])
        finally:
            ctl.shutdown();context.records.close();context.records.join_thread()

    def test_worker_stop_latch_cannot_be_cleared_by_clearing_event(self):
        from physical_robot.bluno_backend import _serial_worker
        event=threading.Event();records=queue.SimpleQueue();status=SimpleNamespace(value=0)
        class Channel:
            index=0;signalled=False;replies=[]
            def poll(self, timeout):
                if self.index==1 and not self.signalled:
                    event.set();self.signalled=True;return False
                if self.signalled:event.clear()  # Simulate an erroneous future caller.
                return True
            def recv(self):
                self.index+=1
                return (self.index,'connect' if self.index==1 else 'SetMotors',time.monotonic_ns()+2_000_000_000,
                        None if self.index==1 else (10,10,10,10))
            def send(self, reply):self.replies.append(reply)
            def close(self):pass
        channel=Channel()
        _serial_worker(channel,BlunoConfig(port='MOCK',pwm_limit=60,boot_wait_s=0),
            serial_factory=functools.partial(FakeSerial,records=records),stop_event=event,stop_status=status)
        payloads=[]
        while not records.empty():payloads.append(records.get())
        self.assertEqual(payloads,[b'A22',b'C'+bytes(8),b'C'+bytes(8)])
        self.assertFalse(channel.replies[-1][1]);self.assertIn('OPERATOR_STOP_LATCHED',channel.replies[-1][3])
    def test_gui_background_runner_uses_existing_controller_and_real_ipc_with_fake_serial(self):
        context=FakeSerialContext();source=Frames()
        def factory(event,status):
            backend=A22MotorBackend(clock_id=source.clock_id,now_ns=source.now_ns,
                config=BlunoConfig(port='MOCK_GUI',pwm_limit=60,boot_wait_s=0,transaction_timeout_s=.3),
                authorization=authorization(),allow_hardware=True,_context=context,stop_event=event,stop_status=status)
            return Controller(source,motors=backend,enable_real_motors=True,base_pwm=45,
                              pwm_limit=60,stop_event=event,processing_preflight=True)
        session=RobotSession(controller_factory=factory)
        try:
            session.start(True)
            limit=time.monotonic()+5;movement=False
            while time.monotonic()<limit:
                payload=context.records.get(timeout=2)
                if payload==b'C'+struct.pack('<4h',45,45,45,45):movement=True;break
            self.assertTrue(movement)
            session.stop();session.thread.join(3)
            self.assertFalse(session.active);self.assertTrue(source.closed)
            self.assertFalse(session.summary['physical_stop_verified'])
            self.assertEqual(len(session.summary['preflight_timings']),5)
        finally:
            session.stop()
            if session.thread:session.thread.join(3)
            context.records.close();context.records.join_thread()

    def test_stop_reaches_serial_owner_while_detector_is_blocked(self):
        context=FakeSerialContext();stop=context.context.Event();status=context.context.RawValue('i',0)
        source=Frames()
        backend=A22MotorBackend(clock_id=source.clock_id,now_ns=source.now_ns,
            config=BlunoConfig(port='MOCK_ONLY',pwm_limit=60,boot_wait_s=0,transaction_timeout_s=.3),
            authorization=authorization(),allow_hardware=True,_context=context,stop_event=stop,stop_status=status)
        controller=Controller(source,motors=backend,enable_real_motors=True,
            base_pwm=45,pwm_limit=60,stop_event=stop)
        release=threading.Event();entered=threading.Event();results=[]
        worker=None
        try:
            controller.start();controller.step()
            # Discard the A22/startup-zero/probe/movement records.
            before=[]
            while True:
                try:before.append(context.records.get(timeout=.03))
                except queue.Empty:break
            self.assertIn(b'C'+struct.pack('<4h',45,45,45,45),before)
            original=controller.adapter.process_frame
            def blocked(*args,**kwargs):
                entered.set();release.wait(2)
                return original(*args,**kwargs)
            with patch.object(controller.adapter,'process_frame',side_effect=blocked):
                worker=threading.Thread(target=lambda:results.append(controller.step()))
                worker.start();self.assertTrue(entered.wait(1))
                began=time.monotonic();controller.request_stop()
                self.assertEqual(context.records.get(timeout=.8),b'C'+bytes(8))
                self.assertLess(time.monotonic()-began,.8)  # Host scheduling tolerance, NOT hardware timing.
                limit=time.monotonic()+.5
                while status.value!=2 and time.monotonic()<limit:time.sleep(.005)
                self.assertEqual(status.value,2)
                self.assertTrue(worker.is_alive())  # Stop did not wait for inference.
                with self.assertRaises(InterruptedError):backend.SetMotors(1,1,owner=controller._owner)
                release.set();worker.join(2);self.assertFalse(worker.is_alive())
            self.assertFalse(results[0]['physical_stop_verified'])
            self.assertNotEqual(results[0]['priority'],'LINE_FOLLOWING')
        finally:
            release.set()
            if worker is not None:worker.join(2)
            controller.shutdown()
            context.records.close();context.records.join_thread()

    def test_stop_before_start_opens_neither_camera_nor_serial(self):
        source=Frames();controller=Controller(source)
        controller.request_stop()
        summary=controller.run(frames=2,on_result=lambda r:None)
        self.assertFalse(source.started)
        self.assertEqual(summary['exit_reason'],'OPERATOR_STOP')
        self.assertFalse(summary['physical_stop_verified'])


class TimingTests(unittest.TestCase):
    def test_cancel_during_preflight_never_connects_serial(self):
        source,backend,ctl=self.setup_clock()
        def cancel(im):ctl.request_stop()
        with patch('physical_robot.main.detect_intersection',side_effect=cancel),\
             patch.object(backend,'connect') as connect:
            with self.assertRaises(InterruptedError):ctl.start()
        connect.assert_not_called();ctl.shutdown()
    def setup_clock(self):
        source=Frames();source.t=10_000_000_000;source.now_ns=lambda:source.t
        def read(timeout=.25):
            source.t+=33_000_000
            return image(),source.t,{}
        source.read=read
        backend=A22MotorBackend(clock_id=source.clock_id,now_ns=source.now_ns,
            config=BlunoConfig(port='MOCK',pwm_limit=60),authorization=authorization(),allow_hardware=True)
        controller=Controller(source,motors=backend,enable_real_motors=True,
            base_pwm=45,pwm_limit=60,processing_preflight=True)
        return source,backend,controller
    def test_cold_work_is_warmed_before_motor_connection(self):
        source,backend,ctl=self.setup_clock();calls=[]
        def inference(im):
            calls.append(1);source.t+=350_000_000 if len(calls)==1 else 20_000_000
        with patch('physical_robot.main.detect_intersection',side_effect=inference),\
             patch.object(backend,'connect') as connect:
            ctl._preflight()
        connect.assert_not_called()
        self.assertGreater(ctl.preflight_timings[0]['frame_age_ms'],250)
        self.assertEqual([x['frame_age_ms'] for x in ctl.preflight_timings[-3:]],[20]*3)
        self.assertEqual(ctl.adapter.max_age_ns,250_000_000)
        self.assertEqual(ctl.follower.previous_error,0)
    def test_slow_warm_processing_blocks_motor_connection(self):
        source,backend,ctl=self.setup_clock()
        def slow(im):source.t+=201_000_000
        with patch('physical_robot.main.detect_intersection',side_effect=slow),\
             patch.object(backend,'connect') as connect:
            with self.assertRaisesRegex(RuntimeError,'Processing too slow'):ctl.start()
        connect.assert_not_called();ctl.shutdown()
        self.assertFalse(backend.serial_connection_opened)
    def test_live_250ms_deadline_still_latches(self):
        source,backend,ctl=self.setup_clock();adapter=ctl.adapter
        original=adapter.pipeline.process_bgr
        def slow(*args,**kwargs):
            value=original(*args,**kwargs);source.t+=251_000_000;return value
        with patch.object(adapter.pipeline,'process_bgr',side_effect=slow):
            result=adapter.process_frame(image(),pixel_format='BGR',captured_ns=source.t,seq=1,clock_id=source.clock_id)
        self.assertEqual(result['stop_reason'],'PROCESSING_DEADLINE_MISSED')
        self.assertEqual(result['stop_class'],'SAFETY')
        self.assertFalse(result['ready_to_resume'])
        self.assertEqual(result['input_age_ms'],0)


class FakeController:
    def __init__(self, event, status):self.event=event;self.closed=False;self.limits=None
    def run(self, **kwargs):
        self.limits=(kwargs['frames'],kwargs['max_seconds'])
        self.event.wait(1)
        return dict(exit_code=0,shutdown=dict(errors=[]),physical_stop_verified=False)
    def shutdown(self):self.closed=True;return dict(errors=[],physical_stop_verified=False)


class SessionTests(unittest.TestCase):
    def test_30_second_limit_signals_stop_without_waiting_for_inference(self):
        session=self.session()
        with patch('physical_robot.gui.threading.Timer') as timer:
            session.start(True)
            self.assertEqual(timer.call_args.args[:2],(30.,session.stop))
            self.assertEqual(timer.call_args.kwargs['args'],('TIME_LIMIT',))
            # Fire the timer callback without sleeping or touching a real device.
            timer.call_args.args[1](*timer.call_args.kwargs['args'])
            self.assertTrue(session.stop_event.is_set())
            session.thread.join(2)
            timer.return_value.cancel.assert_called()
    def session(self, factory=FakeController):
        session=RobotSession(controller_factory=factory)
        self.addCleanup(lambda:(session.stop(), session.thread.join(2) if session.thread else None))
        return session
    def test_no_confirmation_never_constructs_controller(self):
        made=[];session=self.session(lambda *a:made.append(a))
        self.assertFalse(session.start(False));self.assertFalse(session.started);self.assertEqual(made,[])
    def test_start_stop_is_nonblocking_and_run_cannot_restart(self):
        session=self.session();self.assertTrue(session.start(True))
        began=time.monotonic();session.stop()
        self.assertLess(time.monotonic()-began,.05)
        session.thread.join(2);self.assertFalse(session.active)
        self.assertTrue(session.controller.closed)
        self.assertEqual(session.controller.limits,(1_000_000_000,30.))
        with self.assertRaises(RuntimeError):session.start(True)
        self.assertFalse(session.summary['physical_stop_verified'])
    def test_failure_is_short_and_logged_without_json_to_stdout(self):
        def fail(*args):raise RuntimeError('Camera package missing')
        session=self.session(fail)
        with self.assertLogs('renocs',level='ERROR'),contextlib.redirect_stdout(io.StringIO()) as output:
            session.start(True);session.thread.join(2)
        self.assertEqual(output.getvalue(),'')
        self.assertIn('Camera package missing',session.latest())
    def test_fault_reason_not_hidden_by_generic_end_message(self):
        class Faulty(FakeController):
            def run(self,**kwargs):
                kwargs['on_result'](dict(fault_state='SAFE_STOP',reason='PROCESSING_DEADLINE_MISSED'))
                return dict(exit_code=2,shutdown=dict(errors=[]))
        session=self.session(Faulty);session.start(True);session.thread.join(2)
        self.assertIn('250 ms',session.latest())
    def test_communication_stop_failure_is_not_reported_as_rest(self):
        session=self.session();session.start(True)
        # Keep active model alive while inspecting asynchronous serial status.
        session.stop_status.value=3
        session.stop_event.set();session.stop_requested_at=time.monotonic()
        session.thread.join(2)
        # Also check the active-run branch deterministically with a held worker.
        release=threading.Event()
        class Held(FakeController):
            def run(self,**kwargs):release.wait(1);return dict(exit_code=0,shutdown=dict(errors=[]))
        other=self.session(Held);other.start(True);other.stop();other.stop_status.value=3
        self.assertIn('CUTOFF',other.latest());release.set()
    def test_production_factory_uses_exact_settings_but_opens_nothing(self):
        source=Frames();ctx=multiprocessing.get_context('spawn');event=ctx.Event();status=ctx.RawValue('i',0)
        with patch('intersections.raspberry_camera.Picamera2Frames',return_value=source):
            controller=production_controller(event,status)
        self.assertEqual(controller.follower.base_speed,45)
        self.assertEqual(controller.follower.max_speed,60)
        self.assertEqual(controller.motors.config.port,'/dev/ttyACM0')
        self.assertEqual(controller.motors.config.pwm_limit,60)
        self.assertTrue(controller.processing_preflight)
        self.assertIs(controller.motors.stop_event,event)
        self.assertFalse(controller.motors.serial_connection_opened)
        self.assertFalse(source.started);controller.shutdown()


class LauncherTests(unittest.TestCase):
    def test_launcher_also_works_copied_beside_deployment_folder(self):
        source=Path(__file__).resolve().parents[1]/'START_ROBOT.desktop'
        config=configparser.ConfigParser(interpolation=None);config.read(source,encoding='utf-8')
        code=shlex.split(config['Desktop Entry']['Exec'])[2]
        with tempfile.TemporaryDirectory(prefix='RenoCS Desktop ') as tmp:
            desktop=Path(tmp)/'START_ROBOT.desktop';desktop.write_text(source.read_text())
            folder=Path(tmp)/'ST5-RenoCS-quadrillage';folder.mkdir();(folder/'launch_robot.py').write_text('# mock')
            with patch('sys.argv',['-c',str(desktop)]),patch('os.execv') as execute:
                exec(compile(code,'<desktop-exec>','exec'),{})
            self.assertEqual(execute.call_args.args[1][1],str(folder/'launch_robot.py'))
    def test_window_close_requests_stop_before_destroying_active_window(self):
        from unittest.mock import Mock
        root=Mock();session=SimpleNamespace(active=True)
        window=SimpleNamespace(root=root,session=session,closing=False,stop=Mock())
        RobotWindow.close(window)
        self.assertTrue(window.closing);window.stop.assert_called_once();root.destroy.assert_not_called()
        session.active=False;RobotWindow.close(window);root.destroy.assert_called_once()
    def test_desktop_exec_is_relocatable_with_spaces_and_file_uri(self):
        folder=Path(__file__).resolve().parents[1]
        config=configparser.ConfigParser(interpolation=None)
        config.read(folder/'START_ROBOT.desktop',encoding='utf-8')
        entry=config['Desktop Entry'];tokens=shlex.split(entry['Exec'])
        self.assertEqual(entry['Terminal'],'false')
        self.assertEqual(tokens[:2],['/usr/bin/python3','-c']);self.assertEqual(tokens[-1],'%k')
        for desktop in (str(folder/'START_ROBOT.desktop'),(folder/'START_ROBOT.desktop').as_uri()):
            with patch('sys.argv',['-c',desktop]),patch('os.execv') as execute:
                exec(compile(tokens[2],'<desktop-exec>','exec'),{})
            self.assertEqual(execute.call_args.args,('/usr/bin/python3',['/usr/bin/python3',str(folder/'launch_robot.py')]))
    def test_launcher_import_has_no_gui_or_hardware_side_effects(self):
        import runpy
        with patch('tkinter.Tk',side_effect=AssertionError('must not open GUI on import')):
            runpy.run_path(str(Path(__file__).resolve().parents[1]/'launch_robot.py'),run_name='safe_import_test')
    def test_readable_errors(self):
        self.assertIn('250 ms',readable_error({'reason':'PROCESSING_DEADLINE_MISSED'}))
        self.assertIn('White line',readable_error({'reason':'LINE_LOST'}))
    def test_actual_buttons_and_confirmation_with_mock_controller(self):
        import tkinter as tk
        try:root=tk.Tk()
        except tk.TclError as error:self.skipTest('No graphical display: '+str(error))
        root.withdraw();session=RobotSession(controller_factory=FakeController)
        try:
            window=RobotWindow(root,session,'MOCK.log',confirm=lambda *a,**k:False)
            window.start_button.invoke();self.assertFalse(session.started)
            window.confirm=lambda *a,**k:True
            window.start_button.invoke();root.update()
            self.assertTrue(session.started)
            self.assertEqual(str(window.start_button['state']),'disabled')
            window.stop_button.invoke();root.update()
            session.thread.join(2)
            self.assertFalse(session.active);self.assertTrue(session.stop_event.is_set())
            self.assertIn('STOP requested',window.status.get())
        finally:
            session.stop()
            if session.thread:session.thread.join(2)
            root.destroy()


if __name__=='__main__':unittest.main(verbosity=2)
