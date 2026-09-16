"""Exercise Qt's real finished callback and close paths without hardware."""
import os
import sys
import subprocess
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtCore import QProcess
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import QApplication
from gripper_demo import GripperDemo


class ShutdownUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = GripperDemo()
        self.window.save_settings = Mock()
        self.addCleanup(self.window.deleteLater)

    def launch_owner_stub(self):
        process = QProcess(self.window)
        process.finished.connect(self.window.backend_finished)
        self.window.backend = process
        self.window.backend_owned = True
        process.start(sys.executable, ["-c", "import sys; sys.stdin.buffer.read()"])
        self.assertTrue(process.waitForStarted(2000))
        self.addCleanup(lambda: self.window.stop_backend())
        return process

    def test_disconnected_window_closes_pipe_and_waits(self):
        process = self.launch_owner_stub()
        event = QCloseEvent()
        self.window.closeEvent(event)
        self.assertTrue(event.isAccepted())
        self.assertEqual(process.state(), QProcess.NotRunning)
        self.assertIsNone(self.window.backend)

    def test_disconnect_then_close_and_repeat_cleanup(self):
        process = self.launch_owner_stub()
        self.window.safe_disconnect()
        self.assertEqual(process.state(), QProcess.NotRunning)
        event = QCloseEvent()
        self.window.closeEvent(event)
        self.assertTrue(event.isAccepted())
        self.assertTrue(self.window.stop_backend())

    def test_adopted_backend_receives_shutdown(self):
        self.window.connected = True
        self.window.backend_owned = False
        self.window.send_command = Mock(return_value=True)
        self.assertTrue(self.window.stop_backend())
        self.window.send_command.assert_called_once_with("SHUTDOWN", quiet=True)

    def test_shutdown_send_failure_still_closes_lifetime_pipe(self):
        process = self.launch_owner_stub()
        self.window.connected = True
        self.window.sock = Mock()
        self.window.sock.sendall.side_effect = OSError("lost TCP connection")
        self.assertTrue(self.window.stop_backend())
        self.assertEqual(process.state(), QProcess.NotRunning)

    def test_visible_log_window_is_closed(self):
        self.window.show_log_dialog()
        dialog = self.window.log_dialog
        self.assertTrue(dialog.isVisible())
        event = QCloseEvent()
        self.window.closeEvent(event)
        self.assertTrue(event.isAccepted())
        self.assertFalse(dialog.isVisible())
        self.assertIsNone(self.window.log_dialog)

    def test_timeout_stops_pending_backend(self):
        process = self.launch_owner_stub()
        self.window.connect_deadline = 1
        with patch.object(self.window, "open_socket", return_value=False), \
                patch("gripper_demo.QMessageBox.warning"):
            self.window.try_connect_after_start()
        self.assertEqual(process.state(), QProcess.NotRunning)
        self.assertFalse(self.window.connect_timer.isActive())

    def test_failed_cleanup_keeps_window_open(self):
        with patch.object(self.window, "stop_backend", return_value=False), \
                patch("gripper_demo.QMessageBox.warning"):
            event = QCloseEvent()
            self.window.closeEvent(event)
        self.assertFalse(event.isAccepted())
        self.assertFalse(self.window.closing)

    def test_fragmented_greeting_preserves_pending_telemetry(self):
        sock = Mock()
        sock.recv.side_effect = [b"HELLO BXI_", b"GRIPPER_DEMO 1\nTORQUE_LIM"]
        with patch("gripper_demo.socket.socket", return_value=sock):
            self.assertTrue(self.window.open_socket())
        self.assertEqual(self.window.rx_buffer, b"TORQUE_LIM")
        self.window.stop_backend()

    def test_wrong_backend_is_rejected(self):
        sock = Mock()
        sock.recv.return_value = b"HELLO MOTOR_BENCH 1\n"
        with patch("gripper_demo.socket.socket", return_value=sock):
            self.assertFalse(self.window.open_socket())
        sock.close.assert_called_once()
        sock.sendall.assert_not_called()

    def test_gui_event_loop_exits_with_log_dialog_and_backend(self):
        script = """
import sys
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import QApplication
from gripper_demo import GripperDemo
app = QApplication([])
w = GripperDemo()
w.save_settings = lambda: None
p = QProcess(w)
p.finished.connect(w.backend_finished)
w.backend = p
w.backend_owned = True
p.start(sys.executable, ['-c', 'import sys; sys.stdin.buffer.read()'])
assert p.waitForStarted(2000)
w.show()
w.show_log_dialog()
app.aboutToQuit.connect(w.stop_backend)
QTimer.singleShot(100, w.close)
assert app.exec_() == 0
assert w.backend is None
print('CLEAN_GUI_EXIT')
"""
        result = subprocess.run([sys.executable, "-c", script], capture_output=True,
                                timeout=8, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("CLEAN_GUI_EXIT", result.stdout)
        self.assertNotIn("Destroyed while process", result.stderr)

    def test_launch_arguments_preserve_lifetime_pipe(self):
        for uid in (0, 1000):
            with patch("gripper_demo.os.geteuid", return_value=uid), \
                    patch("gripper_demo.QProcess") as factory:
                factory.MergedChannels = QProcess.MergedChannels
                factory.NotRunning = QProcess.NotRunning
                process = factory.return_value
                self.window.start_backend()
                args = process.start.call_args.args
                self.assertIn("--managed", args[1])
                if uid:
                    self.assertEqual(args[0], "pkexec")
                self.window.connect_timer.stop()
                self.window.backend = None


if __name__ == "__main__":
    unittest.main()
