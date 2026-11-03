"""Dry-run integration only; no hardware imports/connections or firmware calls."""
import ast
import builtins
from copy import deepcopy
import contextlib
import io
import json
from pathlib import Path
import signal
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from intersections.fault_manager import MotorFeedback
from physical_robot import line_following
from physical_robot.main import Controller, MockBluno, main

cv2.setNumThreads(1)


def line(x=320):
    im = np.zeros((480,640,3), np.uint8)
    cv2.line(im, (x,0), (x,479), (235,235,235), 8)
    return im


def junction(kind):
    im = np.zeros((480,640,3), np.uint8)
    ends = [(80,200),(560,200),(320,470)]
    if kind == "X": ends.append((320,10))
    for end in ends: cv2.line(im, (320,200), end, (235,235,235), 8)
    return im


class Frames:
    clock_id = "TEST"

    def __init__(self, images):
        self.images = images
        self.t = 10_000_000_000
        self.reads = self.starts = self.closes = 0
        self.age = 0

    def now_ns(self):
        return self.t

    def start(self):
        self.starts += 1

    def read(self, timeout=.25):
        self.t += 64_000_000
        item = self.images[min(self.reads,len(self.images)-1)]
        self.reads += 1
        if isinstance(item, BaseException): raise item
        self.last_image = item
        return item, self.t-self.age, {}

    def close(self):
        self.closes += 1


class LineParityTests(unittest.TestCase):
    def test_original_pd_scan_algorithm_and_gains(self):
        # Extract ONLY the calculation AST for comparison. Never import/execute
        # the original camera/Arduino initialization or its infinite loop.
        source = Path(__file__).resolve().parents[1]/"Basique_Navigation/suivi_ligne.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        constants = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
                     if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)
                     and n.targets[0].id in ("BASE_SPEED","KP","KD","MAX_SPEED")}
        self.assertEqual(constants, dict(BASE_SPEED=100,KP=.04,KD=.015,MAX_SPEED=255))
        for key,value in constants.items(): self.assertEqual(getattr(line_following,key),value)
        loop = next(n for n in tree.body if isinstance(n,ast.Try)).body[0]
        start = next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign)
                     and isinstance(n.targets[0],ast.Name) and n.targets[0].id=="gray")
        end = next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Expr)
                   and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute)
                   and n.value.func.attr=="imshow")
        body = deepcopy(loop.body[start:end])
        body += ast.parse("return robot.last, previous_error").body
        fn = ast.FunctionDef(name="reference", args=ast.arguments(posonlyargs=[],
                args=[ast.arg(arg=x) for x in ("image","previous_error","robot")],
                kwonlyargs=[],kw_defaults=[],defaults=[]),body=body,decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]))
        scope = dict(cv2=cv2,np=np,**constants)
        exec(compile(module,"<original-calculation-only>","exec"),scope)
        class Recorder:
            def SetMotors(self,left,right): self.last=(True,left,right)
            def Stop(self): self.last=(False,0,0)
        follower = line_following.LineFollower(); previous=0
        fixtures = [line(x) for x in (320,380,200,50,580)]
        fixtures += [np.zeros((480,640,3),np.uint8),line(320)]
        multiple = line(150); cv2.line(multiple,(340,0),(340,479),(235,235,235),8)
        fixtures.append(multiple)
        curve = np.zeros((480,640,3),np.uint8)
        cv2.ellipse(curve,(600,300),(300,300),0,80,200,(235,235,235),8)
        fixtures.append(curve)
        fixtures += [np.full((480,640,3), (255,70,0),np.uint8)]
        for im in fixtures:
            recorder = Recorder()
            original, previous = scope['reference'](cv2.cvtColor(im,cv2.COLOR_BGR2RGB),previous,recorder)
            before=im.copy();actual=follower.update(im)
            self.assertEqual((actual['found'],actual['left_pwm'],actual['right_pwm']),original)
            self.assertEqual(follower.previous_error,previous)
            np.testing.assert_array_equal(im,before)
        follower.previous_error = previous = 100_000
        expected,previous=scope['reference'](cv2.cvtColor(line(),cv2.COLOR_BGR2RGB),previous,Recorder())
        actual=follower.update(line());self.assertEqual((actual['found'],actual['left_pwm'],actual['right_pwm']),expected)

    def test_bgr_channel_order_and_missing_line_history(self):
        follower=line_following.LineFollower()
        self.assertFalse(follower.update(np.full((480,640,3),(255,70,0),np.uint8))['found'])
        follower.update(line(370));previous=follower.previous_error
        self.assertFalse(follower.update(np.zeros((480,640,3),np.uint8))['found'])
        self.assertEqual(follower.previous_error,previous)


class IntegrationTests(unittest.TestCase):
    def controller(self, images):
        source=Frames(images);ctl=Controller(source);ctl.start()
        self.addCleanup(ctl.shutdown)
        return source,ctl

    def test_line_following_actual_pwm_order_and_truncation(self):
        source,ctl=self.controller([line(370)])
        result=ctl.step()
        self.assertEqual(result['priority'],'LINE_FOLLOWING')
        self.assertEqual(result['line']['left_pwm'],102.75)
        self.assertEqual(result['last_acknowledged_targets'],(97,102,97,102))
        self.assertEqual(source.reads,1)
        self.assertFalse(result['physical_stop_verified'])

    def test_one_shared_capture_and_no_image_mutation(self):
        image=line();source,ctl=self.controller([image]);before=image.copy();seen=[]
        adapter=ctl.adapter.process_frame;follower=ctl.follower.update
        def observe(im,**kwargs): seen.append(('detector',id(im)));return adapter(im,**kwargs)
        def follow(im): seen.append(('follower',id(im)));return follower(im)
        with patch.object(ctl.adapter,'process_frame',side_effect=observe),patch.object(ctl.follower,'update',side_effect=follow):ctl.step()
        self.assertEqual(seen,[('detector',id(image)),('follower',id(image))])
        self.assertEqual(source.reads,1);np.testing.assert_array_equal(image,before)

    def test_both_intersection_kinds_confirm_and_override_movement(self):
        for kind in ('X','T'):
            source,ctl=self.controller([junction(kind)])
            results=[ctl.step() for _ in range(8)]
            events=[r['event'] for r in results if r['event']]
            self.assertEqual([e['kind'] for e in events],[kind])
            self.assertEqual(events[0]['seq'],3)
            confirmed=next(r for r in results if r['event'])
            self.assertEqual(confirmed['event_camera_t_ns'],confirmed['captured_ns'])
            self.assertEqual(confirmed['event_camera_t_ns'],confirmed['event_clock_epoch_ns']+events[0]['t_ns'])
            self.assertEqual(confirmed['clock_id'],'TEST')
            self.assertTrue(all(r['priority']=='INTERSECTION' for r in results[2:]))
            moves=[c for c in ctl.motors.commands if c['operation']=='SetMotors']
            self.assertEqual(len(moves),2)
            stops=[c for c in ctl.motors.commands if c['operation']=='Stop']
            self.assertEqual(len(stops),2)  # Startup + one intersection transition.
            self.assertEqual(results[-1]['actuation'],'HOLD_STOP')
            self.assertFalse(results[-1]['stop_acknowledged'])  # No new ACK on hold.

    def test_intersection_latch_survives_clear_view(self):
        source,ctl=self.controller([junction('T')]*3+[line()]*8)
        results=[ctl.step() for _ in range(11)]
        self.assertTrue(all(r['priority']=='INTERSECTION' for r in results[2:]))
        self.assertEqual(sum(r['event'] is not None for r in results),1)

    def test_brief_unknown_pause_preserves_tracker_confirmation(self):
        unknown=np.zeros((480,640,3),np.uint8)
        source,ctl=self.controller([junction('T'),junction('T'),unknown,junction('T')])
        results=[ctl.step() for _ in range(4)]
        self.assertEqual(results[2]['priority'],'DEGRADED')
        self.assertFalse(results[2]['decision']['motion_allowed'])
        self.assertEqual(results[3]['event']['kind'],'T')
        self.assertEqual(results[3]['priority'],'INTERSECTION')

    def test_persistent_unknown_escalates_through_existing_adapter(self):
        source,ctl=self.controller([np.zeros((480,640,3),np.uint8)])
        results=[ctl.step() for _ in range(34)]
        self.assertEqual(results[0]['priority'],'DEGRADED')
        self.assertEqual(results[-1]['priority'],'SAFETY')
        self.assertIn('PERSISTENT_UNKNOWN',results[-1]['decision']['latched_faults'])
        self.assertFalse(any(c['operation']=='SetMotors' for c in ctl.motors.commands))

    def test_reported_motor_failure_outranks_simultaneous_intersection(self):
        source,ctl=self.controller([junction('X')]);ctl.step();ctl.step()
        # The next read advances the shared clock by 64 ms.
        feedback=MotorFeedback('TEST',source.now_ns()+64_000_000,True,True)
        result=ctl.step(motor_feedback=feedback)
        self.assertEqual(result['event']['kind'],'X')
        self.assertEqual(result['priority'],'SAFETY')
        self.assertIn('REPORTED_MOTOR_FAILURE',result['decision']['latched_faults'])

    def test_probe_timeout_outranks_intersection_and_cannot_auto_recover(self):
        source,ctl=self.controller([junction('T')]);ctl.step();ctl.step()
        ctl.motors.failures['probe']='TIMEOUT';result=ctl.step()
        self.assertEqual(result['event']['kind'],'T')
        self.assertEqual(result['priority'],'SAFETY')
        ctl.motors.failures.clear()
        self.assertEqual(ctl.step()['priority'],'SAFETY')

    def test_failed_movement_attempts_stop_and_latches_fault(self):
        source,ctl=self.controller([line()]);ctl.motors.failures['SetMotors']='LOST'
        result=ctl.step()
        self.assertEqual(result['priority'],'SAFETY')
        self.assertEqual([r['operation'] for r in ctl.motors.commands][-2:],['SetMotors','Stop'])
        self.assertTrue(result['stop_acknowledged'])  # Mock Stop succeeded; still no rest proof.
        self.assertFalse(result['physical_stop_verified'])
        ctl.motors.failures.clear();self.assertEqual(ctl.step()['priority'],'SAFETY')

    def test_failed_stop_never_claims_zero_targets_or_success(self):
        source,ctl=self.controller([line()]);ctl.step()
        prior=ctl.motors.last_acknowledged_targets
        ctl.motors.failures.update(SetMotors='LOST',Stop='TIMEOUT')
        result=ctl.step()
        self.assertFalse(result['stop_acknowledged'])
        self.assertEqual(result['actuation'],'STOP_UNCONFIRMED')
        self.assertEqual(result['last_acknowledged_targets'],prior)
        self.assertEqual(result['priority'],'SAFETY')

    def test_camera_exception_stops_and_run_closes(self):
        source=Frames([OSError('camera unplugged')]);ctl=Controller(source)
        records=[];summary=ctl.run(frames=5,on_result=records.append)
        self.assertEqual(summary['exit_code'],2)
        self.assertEqual(source.reads,1);self.assertEqual(source.closes,1)
        self.assertEqual(records[0]['priority'],'SAFETY')
        self.assertFalse(any(c['operation']=='SetMotors' for c in ctl.motors.commands))

    def test_invalid_image_stops_without_calling_follower(self):
        source,ctl=self.controller([None])
        with patch.object(ctl.follower,'update',side_effect=AssertionError('must not process invalid frame')):
            result=ctl.step()
        self.assertEqual(result['priority'],'SAFETY')

    def test_clock_mismatch_and_stale_frames_stop(self):
        source,ctl=self.controller([line()]);source.age=251_000_000
        self.assertEqual(ctl.step()['priority'],'SAFETY')
        source,ctl=self.controller([line()]);source.clock_id='OTHER'
        self.assertEqual(ctl.step()['priority'],'SAFETY')

    def test_slow_follower_cannot_authorize_movement_from_stale_frame(self):
        source,ctl=self.controller([line()]);original=ctl.follower.update
        def slow(image):
            result=original(image);source.t+=251_000_000;return result
        with patch.object(ctl.follower,'update',side_effect=slow):result=ctl.step()
        self.assertEqual(result['priority'],'SAFETY')
        self.assertFalse(any(c['operation']=='SetMotors' for c in ctl.motors.commands))

    def test_slow_mock_transaction_triggers_stop_after_return(self):
        source,ctl=self.controller([line()]);original=ctl.motors.SetMotors
        def slow(*args,**kwargs):
            result=original(*args,**kwargs);source.t+=251_000_000;return result
        with patch.object(ctl.motors,'SetMotors',side_effect=slow):result=ctl.step()
        self.assertEqual(result['priority'],'SAFETY')
        self.assertEqual(ctl.motors.commands[-1]['operation'],'Stop')

    def test_line_not_in_bottom_roi_pauses_without_navigation_decision(self):
        image=np.zeros((480,640,3),np.uint8);cv2.line(image,(320,0),(320,250),(235,235,235),8)
        source,ctl=self.controller([image,line()])
        result=ctl.step();self.assertEqual(result['priority'],'LINE_LOST')
        self.assertEqual(result['decision']['state'],'NORMAL')
        self.assertEqual(ctl.step()['priority'],'LINE_FOLLOWING')

    def test_motor_owner_and_real_backend_rejection(self):
        source,ctl=self.controller([line()])
        with self.assertRaises(PermissionError):ctl.motors.SetMotors(100,100,owner=object())
        with self.assertRaises(PermissionError):ctl.motors.Stop(owner=object())
        with self.assertRaises(RuntimeError):Controller(source,motors=ctl.motors)
        with self.assertRaises(TypeError):Controller(source,motors=object())

    def test_concurrent_cycle_rejected(self):
        source,ctl=self.controller([line()]);ctl._lock.acquire()
        try:
            with self.assertRaises(RuntimeError):ctl.step()
        finally:ctl._lock.release()
        self.assertEqual(source.reads,0)

    def test_shutdown_orders_stop_before_camera_close(self):
        source,ctl=self.controller([line()]);ctl.step();seen=[]
        source.close=lambda:seen.append(ctl.motors.commands[-1]['operation'])
        result=ctl.shutdown();self.assertEqual(seen,['Stop'])
        self.assertTrue(result['camera_closed'])
        count=len(ctl.motors.commands);self.assertEqual(ctl.shutdown(),result)
        self.assertEqual(len(ctl.motors.commands),count)
        with self.assertRaises(RuntimeError):ctl.step()

    def test_shutdown_stop_failure_still_closes_camera(self):
        source,ctl=self.controller([line()]);ctl.motors.failures['Stop']='TIMEOUT'
        result=ctl.shutdown()
        self.assertFalse(result['stop_acknowledged']);self.assertTrue(result['camera_closed'])
        self.assertTrue(result['errors']);self.assertFalse(result['physical_stop_verified'])

    def test_shutdown_camera_error_is_reported(self):
        source=Frames([line()]);ctl=Controller(source)
        source.close=lambda:(_ for _ in ()).throw(OSError('close failure'))
        summary=ctl.run(frames=1,on_result=lambda r:None)
        self.assertEqual(summary['exit_code'],2)
        self.assertFalse(summary['shutdown']['camera_closed'])

    def test_interrupt_attempts_stop_and_cleanup(self):
        source=Frames([KeyboardInterrupt()]);ctl=Controller(source)
        result=ctl.run(frames=3,on_result=lambda r:None)
        self.assertTrue(result['shutdown']['stop_acknowledged'])
        self.assertEqual(source.closes,1)
        self.assertFalse(any(c['operation']=='SetMotors' for c in ctl.motors.commands))

    def test_startup_failure_still_cleans_up(self):
        source=Frames([line()]);ctl=Controller(source);ctl.motors.failures['Stop']='TIMEOUT'
        summary=ctl.run(frames=3,on_result=lambda r:None)
        self.assertEqual(summary['exit_code'],2)
        self.assertEqual(source.starts,0);self.assertEqual(source.closes,1)

    def test_processing_exception_is_safe_stop(self):
        source,ctl=self.controller([line()])
        with patch.object(ctl.follower,'update',side_effect=RuntimeError('PD error')):result=ctl.step()
        self.assertEqual(result['priority'],'SAFETY')
        self.assertIn('CAMERA_OR_PROCESSING_FAILURE',result['decision']['latched_faults'])


class CliTests(unittest.TestCase):
    def test_synthetic_dry_run_never_imports_hardware_modules(self):
        original=builtins.__import__
        def guarded(name,*args,**kwargs):
            if name.split('.')[0] in ('serial','arduino','picamera2','libcamera','Basique_Navigation'):
                raise AssertionError('hardware/legacy import forbidden: '+name)
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=guarded),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(['--dry-run','--scenario','T','--frames','12']),0)
        rows=[json.loads(r) for r in output.getvalue().splitlines()]
        self.assertEqual(sum(bool(r.get('event')) for r in rows),1)
        self.assertTrue(all(r['dry_run'] for r in rows))
        self.assertEqual(rows[-1]['shutdown']['physical_stop_verified'],False)

    def test_camera_only_runner_constructs_exactly_one_shared_camera(self):
        source=Frames([line()])
        with patch('intersections.raspberry_camera.Picamera2Frames',return_value=source) as factory,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(['--dry-run','--camera','picamera2','--frames','4']),0)
        factory.assert_called_once_with(size=(640,480),fps=15.)
        self.assertEqual((source.starts,source.reads,source.closes),(1,4,1))

    def test_explicit_dry_run_flag_required(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:main([])
        self.assertEqual(error.exception.code,2)

    def test_camera_initialization_failure_reports_no_motor_activation(self):
        with patch('intersections.raspberry_camera.Picamera2Frames',side_effect=OSError('camera missing')),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(['--dry-run','--camera','picamera2']),2)
        result=json.loads(output.getvalue())
        self.assertTrue(result['mock_motors']);self.assertFalse(result['physical_stop_verified'])

    def test_sigterm_unwinds_to_single_owner_stop_and_restores_handler(self):
        source=Frames([line()]);previous=signal.getsignal(signal.SIGTERM)
        def interrupted_read(timeout=.25):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
        source.read=interrupted_read
        with patch('intersections.raspberry_camera.Picamera2Frames',return_value=source),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(['--dry-run','--camera','picamera2']),0)
        result=json.loads(output.getvalue().splitlines()[-1])
        self.assertEqual(result['exit_reason'],'INTERRUPTED: SIGTERM')
        self.assertTrue(result['shutdown']['stop_acknowledged'])
        self.assertEqual(source.closes,1)
        self.assertIs(signal.getsignal(signal.SIGTERM),previous)


class LiveCameraTests(unittest.TestCase):
    def run_live(self, source, frames=8):
        with patch('intersections.raspberry_camera.Picamera2Frames',return_value=source) as factory,contextlib.redirect_stdout(io.StringIO()) as output:
            code=main(['--dry-run','--live-camera','--frames',str(frames)])
        factory.assert_called_once_with(size=(640,480),fps=15.)
        return code,[json.loads(row) for row in output.getvalue().splitlines()]

    def test_live_alias_uses_one_camera_and_same_frame_for_both_algorithms(self):
        from intersections.physical_adapter import PhysicalIntersectionAdapter
        source=Frames([line(),line(350),line(280)])
        detected=[];followed=[]
        detect=PhysicalIntersectionAdapter.process_frame
        follow=line_following.LineFollower.update
        def detection(instance,image,**kwargs):
            detected.append(id(image));return detect(instance,image,**kwargs)
        def following(instance,image):
            followed.append(id(image));return follow(instance,image)
        with patch.object(PhysicalIntersectionAdapter,'process_frame',autospec=True,side_effect=detection),patch.object(line_following.LineFollower,'update',autospec=True,side_effect=following):
            code,rows=self.run_live(source,3)
        self.assertEqual(code,0)
        self.assertEqual(detected,followed)
        self.assertEqual(detected,[id(im) for im in source.images])
        self.assertEqual((source.starts,source.reads,source.closes),(1,3,1))
        self.assertEqual(rows[-1]['camera_mode'],'picamera2')

    def test_live_prints_proposals_stops_faults_and_processing_latency(self):
        source=Frames([junction('T')]);code,rows=self.run_live(source)
        self.assertEqual(code,0)
        frames=rows[:-1]
        self.assertEqual(len(frames),8)
        for row in frames:
            self.assertEqual(row['intersection_kind'],'T')
            self.assertEqual(set(row['proposed_wheel_pwm']),{'left','right'})
            self.assertEqual(row['fault_state'],'NORMAL')
            self.assertGreaterEqual(row['processing_ms'],0)
            self.assertGreaterEqual(row['capture_wait_ms'],0)
            self.assertAlmostEqual(row['cycle_ms'],row['capture_wait_ms']+row['processing_ms'])
            self.assertFalse(row['serial_connection_opened'])
            self.assertFalse(row['motor_commands_sent'])
        self.assertFalse(frames[0]['stop_requested'])
        self.assertTrue(all(row['stop_requested'] for row in frames[2:]))
        self.assertEqual(sum(bool(row['event']) for row in frames),1)

    def test_live_never_imports_serial_arduino_or_original_controller(self):
        original=builtins.__import__
        def forbidden(name,*args,**kwargs):
            if name.split('.')[0] in ('serial','arduino','Basique_Navigation','VAC_ST5_pretest_completo'):
                raise AssertionError('forbidden physical control import: '+name)
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=forbidden):
            code,rows=self.run_live(Frames([line()]),3)
        self.assertEqual(code,0)
        self.assertFalse(rows[-1]['serial_connection_opened'])
        self.assertFalse(rows[-1]['motor_commands_sent'])

    def test_live_camera_timeout_is_latched_fault_and_closes(self):
        source=Frames([TimeoutError('camera request timed out')]);code,rows=self.run_live(source)
        self.assertEqual(code,2)
        self.assertEqual(source.reads,1);self.assertEqual(source.closes,1)
        result=rows[0]
        self.assertEqual(result['fault_state'],'SAFE_STOP')
        self.assertIn('CAMERA_TIMEOUT',result['decision']['latched_faults'])
        self.assertEqual(result['intersection_kind'],'UNKNOWN')
        self.assertTrue(result['stop_requested'])
        self.assertIsNone(result['proposed_wheel_pwm'])
        self.assertTrue(rows[-1]['shutdown']['camera_closed'])

    def test_live_camera_start_failure_keeps_fault_manager_active(self):
        source=Frames([line()])
        source.start=lambda:(_ for _ in ()).throw(OSError('camera failed to start'))
        code,rows=self.run_live(source)
        self.assertEqual(code,2)
        self.assertEqual(rows[0]['fault_state'],'SAFE_STOP')
        self.assertTrue(rows[0]['stop_requested'])
        self.assertEqual(source.closes,1);self.assertEqual(source.reads,0)

    def test_live_unknown_escalation_reuses_existing_fault_policy(self):
        source=Frames([np.zeros((480,640,3),np.uint8)])
        code,rows=self.run_live(source,40)
        self.assertEqual(code,2)
        self.assertEqual(rows[0]['fault_state'],'DEGRADED')
        self.assertEqual(rows[-2]['fault_state'],'SAFE_STOP')
        self.assertIn('PERSISTENT_UNKNOWN',rows[-2]['decision']['latched_faults'])
        self.assertEqual(source.closes,1)

    def test_live_ctrl_c_cleanup(self):
        source=Frames([KeyboardInterrupt()]);code,rows=self.run_live(source)
        self.assertEqual(code,0)
        self.assertTrue(rows[-1]['exit_reason'].startswith('INTERRUPTED'))
        self.assertTrue(rows[-1]['shutdown']['camera_closed'])
        self.assertEqual(source.closes,1)

    def test_live_option_requires_explicit_dry_run(self):
        with patch('intersections.raspberry_camera.Picamera2Frames') as factory,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            main(['--live-camera'])
        factory.assert_not_called()

    def test_live_option_conflicts_with_a_different_camera_selection(self):
        with patch('intersections.raspberry_camera.Picamera2Frames') as factory,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            main(['--dry-run','--live-camera','--camera','synthetic'])
        factory.assert_not_called()


if __name__=='__main__':
    unittest.main(verbosity=2)
