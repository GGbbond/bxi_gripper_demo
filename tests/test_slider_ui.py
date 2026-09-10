"""No hardware/socket access; verify actual UI command ordering and trace export."""
import csv
import os
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtWidgets import QApplication
from gripper_demo import GripperDemo


class SliderUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = GripperDemo()
        self.window.connected = True
        self.window.power_ready = True
        self.window.power_requested = True
        self.commands = []
        self.window.send_command = lambda command, quiet=False: self.commands.append(command) or True

    def tearDown(self):
        self.window.disconnect_socket()
        self.window.deleteLater()
        self.app.processEvents()

    def test_slider_takes_over_without_feedback_snap_command(self):
        self.window.action_running = True
        self.window.slider_changed(100)
        self.window.flush_slider()
        self.assertEqual(len(self.commands), 1)
        self.assertTrue(self.commands[0].startswith("CLAW_STREAM 10.00 "))
        self.assertFalse(self.window.action_running)

    def test_goto_cancels_pending_slider(self):
        self.window.slider_changed(100)
        self.window.target_spin.setValue(20)
        self.window.go_target()
        self.window.flush_slider()
        self.assertEqual(len(self.commands), 1)
        self.assertTrue(self.commands[0].startswith("CLAW_MOVE 20.00 "))

    def test_disable_discards_pending_slider(self):
        self.window.slider_changed(100)
        self.window.toggle_motor_enabled()
        self.window.flush_slider()
        self.assertEqual(self.commands, ["CLAW_DISABLE"])

    def test_zero_blocks_old_commands_and_resets_slider(self):
        self.window.slider_changed(120)
        self.window.handle_line("CLAW_ZERO_STARTED")
        self.window.flush_slider()
        self.assertEqual(self.commands, [])
        self.window.handle_line("CLAW_ZERO_COMPLETE")
        self.assertIsNone(self.window.slider_pending)
        self.assertEqual(self.window.slider.value(), 0)
        self.window.slider_changed(1)
        self.window.flush_slider()
        self.assertTrue(self.commands[0].startswith("CLAW_STREAM 0.10 "))

    def test_diagnostic_csv_contains_atomic_snapshot(self):
        self.window.handle_line("TRACE 1000 1.5 5 2 1.4 4.9 0.2 42 2 1 1")
        self.window.handle_line("TRACE broken")
        self.assertEqual(len(self.window.motion_trace), 1)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "trace.csv")
            with patch("gripper_demo.QFileDialog.getSaveFileName", return_value=(path, "")):
                self.window.export_motion_trace()
            with open(path, encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["command_position_deg"], "1.5")
            self.assertEqual(rows[0]["feedback_position_deg"], "1.4")
            self.assertEqual(rows[0]["feedback_sequence"], "42")


if __name__ == "__main__":
    unittest.main()
