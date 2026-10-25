"""验证演示接口的启动门禁、单次委托和完成记录；不接触硬件。"""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ican_arm_demo import ArmDemo, DemoError


def snapshot(used=False, active=False):
    return {"compatible": True, "active": active, "cycle_consumed": used,
            "can_request_start": not used and not active,
            "supported_actions": ["elbow-full"],
            "service": {"auth_ready": False}, "bench": {}}


class DemoTests(unittest.TestCase):
    def test_no_readiness_never_sends_or_queries(self):
        with patch("ican_arm_demo.query") as request, patch("ican_arm_demo.subprocess.run") as run:
            with self.assertRaises(DemoError):
                ArmDemo().start_curl()
            request.assert_not_called()
            run.assert_not_called()

    def test_consumed_boot_never_starts(self):
        with patch.object(ArmDemo, "status", return_value=snapshot(used=True)), \
                patch("ican_arm_demo.subprocess.run") as run:
            with self.assertRaises(DemoError):
                ArmDemo().start_curl(supported_and_clear=True)
            run.assert_not_called()

    def test_locked_flag_alone_does_not_mean_consumed(self):
        service = {"ok": True, "service": "arm-wifi-debug-v5", "m33_binding": True,
                   "motion_profile": "legacy-curl-j5-hold-j46-v5"}
        with patch("ican_arm_demo.query", side_effect=[service, {
            "ok": True, "state": 0, "active": False, "flags": 1, "trial_locked": True}]), \
                patch("ican_arm_demo.time.sleep"):
            self.assertTrue(ArmDemo().status()["can_request_start"])

    def test_old_profile_rejected(self):
        with patch("ican_arm_demo.query", side_effect=[{
            "ok": True, "service": "arm-wifi-debug-v5", "m33_binding": True,
            "motion_profile": "legacy-curl-j5-hold-j46-v4"}, {
            "ok": True, "state": 0, "active": False, "flags": 1}]), \
                patch("ican_arm_demo.time.sleep"):
            self.assertFalse(ArmDemo().status()["can_request_start"])

    def test_done_preserved_after_cleanup_cancel_and_log_saved(self):
        done = {"state": 13, "run_result": 0, "active": False, "stop_send_result": 0}
        cancel = {"state": 14, "run_result": -26}
        output = "trial_result: " + json.dumps(done) + "\nfinal_stop_request: " + json.dumps(cancel)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trial.log"
            with patch.object(ArmDemo, "status", side_effect=[snapshot(), snapshot(used=True)]), \
                    patch("ican_arm_demo.time.sleep"), patch("ican_arm_demo.subprocess.run",
                    return_value=subprocess.CompletedProcess([], 0, output, "")) as run:
                r = ArmDemo().start_curl(supported_and_clear=True, log_path=path)
                self.assertTrue(r["completed"])
                self.assertFalse(r["physical_stop_verified"])
                self.assertEqual(r["completed_cycle"], done)
                self.assertEqual(r["cleanup_stop_request"], cancel)
                self.assertEqual(path.read_text(encoding="utf-8"), output)
                run.assert_called_once()
                self.assertIn("--supported-and-clear", run.call_args.args[0])

    def test_existing_log_prevents_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "existing.log"
            path.write_text("evidence", encoding="utf-8")
            with patch.object(ArmDemo, "status", return_value=snapshot()), \
                    patch("ican_arm_demo.subprocess.run") as run:
                with self.assertRaises(FileExistsError):
                    ArmDemo().start_curl(supported_and_clear=True, log_path=path)
                run.assert_not_called()
                self.assertEqual(path.read_text(), "evidence")

    def test_timeout_sends_stop_once_and_never_restarts(self):
        with patch.object(ArmDemo, "status", return_value=snapshot()), \
                patch("ican_arm_demo.subprocess.run", side_effect=subprocess.TimeoutExpired([], 85)) as run, \
                patch.object(ArmDemo, "stop", return_value={"ok": True}) as stop:
            with self.assertRaises(DemoError) as error:
                ArmDemo().start_curl(supported_and_clear=True)
            self.assertTrue(error.exception.report["host_timeout"])
            run.assert_called_once()
            stop.assert_called_once()

    def test_failed_trial_never_retried(self):
        with patch.object(ArmDemo, "status", side_effect=[snapshot(), snapshot(used=True)]), \
                patch("ican_arm_demo.time.sleep"), patch("ican_arm_demo.subprocess.run",
                return_value=subprocess.CompletedProcess([], 1, "", "DAP unavailable")) as run:
            with self.assertRaises(DemoError):
                ArmDemo().start_curl(supported_and_clear=True)
            run.assert_called_once()

    def test_keyboard_interrupt_requests_stop(self):
        with patch.object(ArmDemo, "status", return_value=snapshot()), \
                patch("ican_arm_demo.subprocess.run", side_effect=KeyboardInterrupt), \
                patch.object(ArmDemo, "stop", return_value={"ok": True}) as stop:
            with self.assertRaises(DemoError):
                ArmDemo().start_curl(supported_and_clear=True)
            stop.assert_called_once()

    def test_stop_only_delegates_group_stop(self):
        with patch("ican_arm_demo.query", return_value={"ok": True}) as request:
            ArmDemo().stop()
            request.assert_called_once_with("192.168.5.182", "stop", None)

    def test_new_action_on_legacy_firmware_never_falls_back(self):
        with patch.object(ArmDemo, "status", return_value=snapshot()), \
                patch("ican_arm_demo.subprocess.run") as run:
            with self.assertRaises(DemoError):
                ArmDemo().start_action("elbow-medium", supported_and_clear=True)
            run.assert_not_called()

    def test_selected_action_delegated_once(self):
        before = snapshot()
        before["supported_actions"] = ["elbow-full", "elbow-small", "elbow-medium", "elbow-pause"]
        done = {"state": 13, "run_result": 0, "active": False, "stop_send_result": 0}
        with patch.object(ArmDemo, "status", side_effect=[before, snapshot(used=True)]), \
                patch("ican_arm_demo.time.sleep"), patch("ican_arm_demo.subprocess.run",
                return_value=subprocess.CompletedProcess([], 0, "trial_result: " + json.dumps(done), "")) as run:
            r = ArmDemo().start_action("elbow-pause", supported_and_clear=True)
            self.assertTrue(r["completed"])
            self.assertEqual(r["action"], "elbow-pause")
            command = run.call_args.args[0]
            self.assertEqual(command[command.index("--action")+1], "elbow-pause")
            run.assert_called_once()


if __name__ == "__main__":
    unittest.main()
