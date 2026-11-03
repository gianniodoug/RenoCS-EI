"""T.9 mocks only. No device opening, motor operation or firmware changes."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from intersections.fault_manager import FaultManager, MotorFeedback, SerialStatus
from intersections.bounded_bluno_stop import BoundedBlunoStop, StopProtocolError


class Clock:
    def __init__(self):
        self.t = 10_000_000_000

    def __call__(self):
        return self.t


def camera(now, kind="NONE", navigation=False):
    return dict(clock_id="TEST", captured_ns=now, frame_accepted=True,
                observation=dict(valid=kind != "UNKNOWN", kind=kind),
                stop_requested=navigation, stop_class="NAVIGATION" if navigation else None)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.manager = FaultManager(clock_id="TEST")

    def tick(self, kind="NONE", serial="OK", motor=None, navigation=False, result=None):
        return self.manager.update(camera(self.clock(), kind, navigation) if result is None else result,
                    serial_status=SerialStatus("TEST", self.clock(), serial),
                    motor_feedback=motor, now_ns=self.clock(), clock_id="TEST")

    def recover(self, operator=True, rest=True):
        return self.manager.recover(operator_authorized=operator, rest_verified=rest,
                                    now_ns=self.clock(), clock_id="TEST")

    def test_startup_cannot_move(self):
        self.assertEqual(self.manager.state, "DEGRADED")
        self.assertFalse(self.manager.snapshot()["motion_allowed"])

    def test_normal_and_normal_navigation_stop_are_not_faults(self):
        self.assertTrue(self.tick()["motion_allowed"])
        r = self.tick(navigation=True)
        self.assertEqual(r["state"], "NORMAL")
        self.assertEqual(r["stop_class"], "NAVIGATION")
        self.assertFalse(r["motion_allowed"])
        self.assertEqual(r["latched_faults"], [])

    def test_brief_unknown_pauses_then_recovers_without_latch(self):
        r = self.tick("UNKNOWN")
        self.assertEqual(r["state"], "DEGRADED")
        self.assertFalse(r["motion_allowed"])
        self.assertFalse(r["stop_latched"])
        self.assertEqual(self.tick()["state"], "NORMAL")

    def test_adapter_fault_reasons_reused_and_latched(self):
        for reason in ("INVALID_CAMERA_FRAME", "CAMERA_FAILURE", "PERSISTENT_UNKNOWN",
                       "CAMERA_FRAME_TIMEOUT", "PROCESSING_DEADLINE_MISSED"):
            self.manager = FaultManager(clock_id="TEST")
            bad = dict(camera(self.clock()), stop_class="SAFETY", stop_requested=True,
                       stop_reason=reason)
            r = self.tick(result=bad)
            self.assertEqual(r["state"], "SAFE_STOP")
            self.assertIn(reason, r["latched_faults"])
            self.assertEqual(self.tick()["state"], "SAFE_STOP")

    def test_missing_invalid_camera_and_pending_startup(self):
        for bad in ({}, None, "not a status"):
            r = self.manager.update(bad, serial_status=SerialStatus("TEST", self.clock(), "OK"),
                                   now_ns=self.clock(), clock_id="TEST")
            self.assertIn("CAMERA_STATUS_INVALID", r["latched_faults"])
        self.manager = FaultManager(clock_id="TEST")
        pending = dict(camera(self.clock()), captured_ns=None, observation=None, frame_accepted=False)
        self.assertEqual(self.tick(result=pending)["state"], "DEGRADED")
        self.clock.t += 251_000_000
        self.assertIn("CAMERA_FRAME_TIMEOUT", self.tick(result=pending)["latched_faults"])

    def test_camera_envelope_failures(self):
        for changes, reason in ((dict(captured_ns=self.clock()-251_000_000), "CAMERA_STATUS_STALE_OR_INVALID"),
                                (dict(captured_ns=self.clock()+1), "CAMERA_STATUS_STALE_OR_INVALID"),
                                (dict(clock_id="OTHER"), "CAMERA_CLOCK_MISMATCH"),
                                (dict(frame_accepted=False), "CAMERA_STATUS_INVALID"),
                                (dict(observation={"valid": "yes"}), "CAMERA_OBSERVATION_INVALID")):
            self.manager = FaultManager(clock_id="TEST")
            self.assertIn(reason, self.tick(result=dict(camera(self.clock()), **changes))["latched_faults"])

    def test_serial_lost_timeout_bad_protocol_and_unverified(self):
        for state in ("LOST", "TIMEOUT", "PROTOCOL_ERROR", "UNVERIFIED"):
            self.manager = FaultManager(clock_id="TEST")
            r = self.tick(serial=state)
            self.assertIn("SERIAL_"+state, r["latched_faults"])
            self.assertFalse(r["serial_verified"])
            self.assertFalse(r["physical_stop_verified"])

    def test_stale_serial_and_malformed_status(self):
        for serial in (None, SerialStatus("OTHER", self.clock(), "OK"),
                       SerialStatus("TEST", self.clock()-501_000_000, "OK"),
                       SerialStatus("TEST", self.clock()+1, "OK"),
                       SerialStatus("TEST", self.clock(), [])):
            r = self.manager.update(camera(self.clock()), serial_status=serial,
                                   now_ns=self.clock(), clock_id="TEST")
            self.assertEqual(r["state"], "SAFE_STOP")

    def test_feedback_unavailable_does_not_invent_motor_failure(self):
        for feedback in (None, MotorFeedback("TEST", self.clock(), False, True),
                         MotorFeedback("TEST", self.clock()-1_000_000_000, True, True)):
            r = self.tick(motor=feedback)
            self.assertEqual(r["state"], "NORMAL")
            self.assertFalse(r["motor_feedback_verified"])

    def test_reliable_reported_motor_failure_latches(self):
        r = self.tick(motor=MotorFeedback("TEST", self.clock(), True, True))
        self.assertIn("REPORTED_MOTOR_FAILURE", r["latched_faults"])
        self.assertEqual(self.tick(motor=MotorFeedback("TEST", self.clock(), True, False))["state"], "SAFE_STOP")

    def test_required_feedback_loss_is_fault(self):
        self.manager = FaultManager(clock_id="TEST", require_motor_feedback=True)
        self.assertIn("MOTOR_FEEDBACK_UNAVAILABLE", self.tick()["latched_faults"])
        self.tick(motor=MotorFeedback("TEST", self.clock(), True, False))
        self.assertTrue(self.recover())

    def test_losing_feedback_cannot_clear_reported_motor_failure(self):
        self.tick(motor=MotorFeedback("TEST", self.clock(), True, True))
        self.tick()
        self.assertFalse(self.recover())
        self.assertIn("MOTOR_FEEDBACK_UNAVAILABLE", self.manager.snapshot()["active_faults"])
        self.tick(motor=MotorFeedback("TEST", self.clock(), True, False))
        self.assertTrue(self.recover())

    def test_stop_priority_and_reason_history(self):
        self.tick(serial="LOST", navigation=True)
        r = self.tick(result=dict(camera(self.clock()), stop_class="SAFETY", stop_requested=True,
                                  stop_reason="CAMERA_FAILURE"), serial="TIMEOUT")
        self.assertEqual(r["stop_class"], "SAFETY")
        self.assertEqual(set(r["latched_faults"]), {"SERIAL_LOST", "SERIAL_TIMEOUT", "CAMERA_FAILURE"})
        r["latched_faults"].clear()
        self.assertFalse(self.tick(navigation=False)["motion_allowed"])

    def test_recovery_requires_authorization_rest_and_current_health(self):
        self.tick(serial="LOST")
        self.assertFalse(self.recover())
        self.tick()
        self.assertFalse(self.recover(operator=False))
        self.assertFalse(self.recover(rest=False))
        self.assertEqual(self.manager.state, "SAFE_STOP")
        self.assertTrue(self.recover())
        self.assertEqual(self.manager.state, "NORMAL")

    def test_recovery_rejects_stale_or_unknown_inputs(self):
        self.tick(serial="LOST"); self.tick()
        self.clock.t += 1_000_000_000
        self.assertFalse(self.recover())
        self.tick("UNKNOWN")
        self.assertFalse(self.recover())

    def test_recovery_does_not_clear_navigation_stop(self):
        self.tick(serial="LOST"); self.tick(navigation=True)
        self.assertTrue(self.recover())
        r = self.manager.snapshot()
        self.assertEqual(r["stop_class"], "NAVIGATION")
        self.assertFalse(r["motion_allowed"])

    def test_clock_and_configuration_errors(self):
        self.tick()
        for now, domain in ((self.clock()-1, "TEST"), (self.clock(), "OTHER"), (True, "TEST")):
            r = self.manager.update(camera(self.clock()), serial_status=None, now_ns=now, clock_id=domain)
            self.assertIn("CONTROL_CLOCK_INVALID", r["latched_faults"])
        for kwargs in ({"clock_id": ""}, {"clock_id": "TEST", "serial_timeout_ns": 0}):
            with self.assertRaises(ValueError): FaultManager(**kwargs)


class FakePort:
    def __init__(self, clock, ack=b"OK Moteurs mis aux consignes : 0 0 0 0\r\n"):
        self.clock, self.ack = clock, bytearray(ack)
        self.data = bytearray()
        self.timeout = None
        self.write_timeout = None
        self.is_open = True
        self.pending_before = 0
        self.error = None
        self.partial = False
        self.byte_delay_ns = 0
        self.write_delay_ns = 0
        self.read_budgets = []
        self.write_budgets = []

    @property
    def in_waiting(self):
        return self.pending_before

    def write(self, data):
        self.write_budgets.append(self.write_timeout)
        if self.error: raise self.error
        self.clock.t += self.write_delay_ns
        if self.partial: return 0
        self.data.extend(data)
        return len(data)

    def read(self, count):
        self.read_budgets.append(self.timeout)
        if self.error: raise self.error
        if self.ack:
            self.clock.t += self.byte_delay_ns
            return bytes([self.ack.pop(0)])
        self.clock.t += round(min(self.timeout, .01)*1e9)
        return b""


class StopBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the legacy module's dependency outside patch.dict(sys.modules):
        # extension modules must not be removed and imported a second time.
        import numpy
        root = Path(__file__).resolve().parents[1]/"codes-robot-bluno/codes-robot-bluno/basic_motion"
        def load(name, file):
            spec = importlib.util.spec_from_file_location(name, file)
            mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
            return mod
        forbidden = lambda *a, **k: (_ for _ in ()).throw(AssertionError("hardware forbidden"))
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=forbidden)}):
            util = load("t9_legacy_util", root/"utilitaries.py")
            with patch.dict(sys.modules, {"utilitaries": util}):
                cls.legacy = load("t9_legacy_arduino", root/"arduino.py")

    def setUp(self):
        self.clock = Clock(); self.port = FakePort(self.clock)
        self.car = object.__new__(self.legacy.arduino)
        self.car.serialport = self.port
        self.link = BoundedBlunoStop(self.car, clock_id="TEST", now_ns=self.clock, timeout_s=.1)

    def stop(self):
        with contextlib.redirect_stdout(io.StringIO()): self.link.Stop()

    def assert_restored(self):
        self.assertIs(self.car.serialport, self.port)
        self.assertIsNone(self.port.timeout); self.assertIsNone(self.port.write_timeout)
        self.assertFalse(self.link.physical_stop_verified)

    def test_construction_has_no_io(self):
        self.assertEqual(self.port.data, b"")
        self.assertEqual(self.link.status.state, "UNVERIFIED")

    def test_exact_legacy_stop_and_ack_with_finite_io_budgets(self):
        self.stop()
        self.assertEqual(self.port.data, b"C"+bytes(8))
        self.assertTrue(self.link.stop_acknowledged)
        self.assertEqual(self.link.status.state, "OK")
        self.assertTrue(all(0 < t <= .1 for t in self.port.read_budgets+self.port.write_budgets))
        self.assert_restored()

    def test_existing_b_probe_checks_liveness_without_motor_command(self):
        self.port.ack = bytearray(b"OK\r\n")
        status = self.link.probe()
        self.assertEqual(self.port.data, b"B")
        self.assertEqual(status.state, "OK")
        self.assertFalse(self.link.stop_acknowledged)
        self.assert_restored()
        fm = FaultManager(clock_id="TEST")
        self.assertTrue(fm.update(camera(self.clock()), serial_status=status,
                                 now_ns=self.clock(), clock_id="TEST")["motion_allowed"])

    def test_probe_missing_ack_produces_link_timeout(self):
        self.port.ack.clear()
        with self.assertRaises(TimeoutError): self.link.probe()
        self.assertEqual(self.port.data, b"B")
        self.assertEqual(self.link.status.state, "TIMEOUT")
        self.assertFalse(self.link.stop_acknowledged)
        self.assert_restored()

    def test_probe_does_not_claim_an_old_stop_ack_is_current(self):
        self.stop()
        self.port.ack = bytearray(b"OK\r\n")
        self.link.probe()
        self.assertFalse(self.link.stop_acknowledged)

    def test_invalid_ack_length_and_clock(self):
        self.port.ack = bytearray(b"z"*128)
        with self.assertRaises(StopProtocolError): self.stop()
        self.setUp()
        original_write = self.port.write
        def backwards(data):
            result = original_write(data)
            self.clock.t -= 1
            return result
        self.port.write = backwards
        with self.assertRaises(StopProtocolError): self.stop()
        self.assertFalse(self.link.stop_acknowledged)

    def test_missing_ack_times_out_and_no_automatic_retry(self):
        self.port.ack.clear()
        with self.assertRaises(TimeoutError): self.stop()
        self.assertEqual(self.link.status.state, "TIMEOUT")
        self.assertFalse(self.link.stop_acknowledged)
        data = bytes(self.port.data)
        with self.assertRaises(StopProtocolError): self.stop()
        self.assertEqual(data, bytes(self.port.data))
        self.assert_restored()

    def test_negative_unrelated_or_partial_ack_not_success(self):
        for ack in (b"ER\r\n", b"OB\r\n", b"OK\r\n", b"OK Arduino connecte\r\n", b"OK Moteurs mis aux consignes : 0 0 0 0"):
            self.setUp(); self.port.ack = bytearray(ack)
            with self.assertRaises((StopProtocolError, TimeoutError)): self.stop()
            self.assertFalse(self.link.stop_acknowledged)
            self.assert_restored()

    def test_slow_trickle_cannot_extend_total_deadline(self):
        self.port.byte_delay_ns = 30_000_000
        with self.assertRaises(TimeoutError): self.stop()
        self.assertLessEqual(self.clock()-10_000_000_000, 130_000_000)
        self.assert_restored()

    def test_write_time_counts_toward_same_deadline(self):
        self.port.write_delay_ns = 30_000_000
        with self.assertRaises(TimeoutError): self.stop()
        self.assertEqual(self.link.status.state, "TIMEOUT")
        self.assert_restored()

    def test_closed_partial_disconnected_and_write_timeout(self):
        cases = ("closed", "partial", "lost", "write_timeout")
        for case in cases:
            self.setUp()
            if case == "closed": self.port.is_open = False
            if case == "partial": self.port.partial = True
            if case == "lost": self.port.error = OSError("mock unplug")
            if case == "write_timeout": self.port.error = type("SerialTimeoutException", (OSError,), {})("mock")
            with self.assertRaises(OSError): self.stop()
            self.assertEqual(self.link.status.state, "TIMEOUT" if case == "write_timeout" else "LOST")
            self.assertFalse(self.link.stop_acknowledged)
            self.assert_restored()

    def test_pending_input_refuses_uncorrelated_reply(self):
        self.port.pending_before = 4
        with self.assertRaises(StopProtocolError): self.stop()
        self.assertEqual(self.port.data, b"")
        self.assert_restored()

    def test_no_extra_or_nonzero_command_can_pass_bridge(self):
        self.car.Stop = lambda: self.car.serialport.write(b"C\1")
        with self.assertRaises(StopProtocolError): self.stop()
        self.assertEqual(self.port.data, b"")

    def test_link_failure_feeds_safe_stop_without_physical_success_claim(self):
        self.port.ack.clear()
        with self.assertRaises(TimeoutError): self.stop()
        fm = FaultManager(clock_id="TEST")
        r = fm.update(camera(self.clock()), serial_status=self.link.status,
                      now_ns=self.clock(), clock_id="TEST")
        self.assertIn("SERIAL_TIMEOUT", r["latched_faults"])
        self.assertTrue(r["stop_request_only"])
        self.assertFalse(r["physical_stop_verified"])

    def test_concurrent_calls_rejected_without_writes(self):
        self.link._lock.acquire()
        try:
            with self.assertRaises(RuntimeError): self.stop()
        finally:
            self.link._lock.release()
        self.assertEqual(self.port.data, b"")

    def test_actual_camera_adapter_faults_and_stop_bridge_integration(self):
        # Synthetic arrays exercise wiring only; no claim of real-camera testing.
        import cv2
        import numpy as np
        from intersections.physical_adapter import PhysicalIntersectionAdapter
        for kind in ("X", "T"):
            self.setUp()
            adapter = PhysicalIntersectionAdapter(run_id=kind, clock_id="TEST", now_ns=self.clock)
            fm = FaultManager(clock_id="TEST", camera_timeout_ns=adapter.max_age_ns)
            image = np.zeros((360,640,3),np.uint8)
            ends = [(80,180),(560,180),(320,350)]
            if kind == "X": ends.append((320,10))
            for end in ends: cv2.line(image, (320,180), end, (235,235,235), 8)
            events = []
            for seq in range(1,5):
                self.clock.t += 64_000_000
                result = adapter.process_frame(image, pixel_format="BGR", captured_ns=self.clock(), seq=seq, clock_id="TEST")
                r = fm.update(result, serial_status=SerialStatus("TEST",self.clock(),"OK"), now_ns=self.clock(), clock_id="TEST")
                if result["confirmed_event"]: events.append(result["confirmed_event"])
            self.assertEqual([e['kind'] for e in events], [kind])
            self.assertEqual(r["stop_class"], "NAVIGATION")
            with contextlib.redirect_stdout(io.StringIO()): self.assertTrue(adapter.apply_stop_request(self.link))
            self.assertFalse(adapter.apply_stop_request(self.link))
            self.assertEqual(self.port.data, b"C"+bytes(8))
            # Existing adapter owns UNKNOWN timeout, even after navigation stop.
            for seq in range(5,39):
                self.clock.t += 64_000_000
                result = adapter.process_frame(np.zeros_like(image), pixel_format="BGR", captured_ns=self.clock(), seq=seq, clock_id="TEST")
                r = fm.update(result, serial_status=SerialStatus("TEST",self.clock(),"OK"), now_ns=self.clock(), clock_id="TEST")
            self.assertIn("PERSISTENT_UNKNOWN", r["latched_faults"])
            self.assertEqual(r["stop_class"], "SAFETY")
            self.assertFalse(r["motion_allowed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
