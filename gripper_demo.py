#!/usr/bin/env python3
import json
import math
import os
import socket
import sys
import time

from PyQt5.QtCore import QProcess, QSettings, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QIcon
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


APP_TITLE = "BXI 夹爪控制演示"
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 9999
POSITION_MIN = -360.0
POSITION_MAX = 360.0
PEAK_FACTOR = 1.875


def resource_path(*parts):
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, *parts)


BACKEND_PATH = resource_path("build", "bin", "gripper_backend")


class ValueCard(QFrame):
    def __init__(self, title, unit):
        super().__init__()
        self.unit = unit
        self.setObjectName("valueCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        label = QLabel(title)
        label.setProperty("role", "muted")
        self.value = QLabel(f"-- {unit}")
        self.value.setProperty("role", "value")
        layout.addWidget(label)
        layout.addWidget(self.value)

    def set_value(self, value, decimals=2):
        self.value.setText(f"{value:.{decimals}f} {self.unit}")

    def clear(self):
        self.value.setText(f"-- {self.unit}")


class GripperDemo(QMainWindow):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("BXI", "gripper_demo")
        self.sock = None
        self.rx_buffer = b""
        self.connected = False
        self.power_ready = False
        self.power_requested = False
        self.motor_disabled = False
        self.motor_enabling = False
        self.zero_in_progress = False
        self.backend = None
        self.backend_owned = False
        self.connect_deadline = 0.0
        self.feedback_position = 0.0
        self.command_position = 0.0
        self.slider_pending = None
        self.action_running = False
        self.action_paused = False
        self.action_index = 0
        self.action_points = []
        self.log_dialog = None
        self.log_dialog_output = None
        self.action_wait_timer = QTimer(self)
        self.action_wait_timer.setSingleShot(True)
        self.action_wait_timer.timeout.connect(self.advance_action)

        self.build_ui()
        self.load_settings()

        self.rx_timer = QTimer(self)
        self.rx_timer.setInterval(20)
        self.rx_timer.timeout.connect(self.receive_data)
        self.connect_timer = QTimer(self)
        self.connect_timer.setInterval(300)
        self.connect_timer.timeout.connect(self.try_connect_after_start)
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self.poll_status)
        self.slider_timer = QTimer(self)
        self.slider_timer.setInterval(20)
        self.slider_timer.timeout.connect(self.flush_slider)
        self.update_ui()

    def build_ui(self):
        self.setWindowTitle(APP_TITLE)
        self.setWindowIcon(QIcon(resource_path("assets", "gripper_demo.svg")))
        self.resize(1120, 760)
        self.setMinimumSize(900, 650)
        self.setStyleSheet("""
            QWidget { background: #111827; color: #e5e7eb; font-size: 14px; }
            QGroupBox { border: 1px solid #334155; border-radius: 8px;
                        margin-top: 12px; padding-top: 10px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
            QPushButton { background: #263449; border: 1px solid #43536b;
                          border-radius: 6px; padding: 7px 13px; }
            QPushButton:hover { background: #33445e; }
            QPushButton:pressed { background: #1d4ed8; }
            QPushButton:disabled { color: #64748b; background: #1e293b; }
            QPushButton[primary="true"] { background: #2563eb; border-color: #3b82f6; }
            QPushButton[danger="true"] { background: #b91c1c; border-color: #ef4444;
                                          font-size: 16px; font-weight: 700; }
            QPushButton[primary="true"]:disabled, QPushButton[danger="true"]:disabled {
                color: #64748b; background: #1e293b; border-color: #334155;
            }
            QLabel[role="muted"] { color: #94a3b8; }
            QLabel[role="value"] { font-size: 23px; font-weight: 700; color: #f8fafc; }
            QFrame#valueCard { background: #1e293b; border: 1px solid #334155;
                               border-radius: 8px; }
            QComboBox, QDoubleSpinBox { background: #0f172a; border: 1px solid #475569;
                                       border-radius: 5px; padding: 5px; min-height: 22px; }
            QPlainTextEdit, QTableWidget { background: #0b1220; border: 1px solid #334155;
                                          alternate-background-color: #131e30; }
            QHeaderView::section { background: #243146; color: #e5e7eb; padding: 6px;
                                   border: 0; border-right: 1px solid #334155; }
            QSlider::groove:horizontal { height: 6px; background: #334155; border-radius: 3px; }
            QSlider::handle:horizontal { width: 18px; margin: -6px 0; background: #60a5fa;
                                         border-radius: 9px; }
        """)

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(14, 12, 14, 12)

        top = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel(APP_TITLE)
        title.setStyleSheet("font-size: 24px; font-weight: 700;")
        self.connection_label = QLabel("● 未连接")
        self.connection_label.setProperty("role", "muted")
        title_box.addWidget(title)
        title_box.addWidget(self.connection_label)
        top.addLayout(title_box)
        top.addStretch()
        self.connect_button = QPushButton("启动并连接")
        self.connect_button.setProperty("primary", True)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.power_button = QPushButton("夹爪电机上电")
        self.power_button.clicked.connect(self.toggle_power)
        self.emergency_button = QPushButton("急停并下电")
        self.emergency_button.setProperty("danger", True)
        self.emergency_button.clicked.connect(self.emergency_stop)
        top.addWidget(self.connect_button)
        top.addWidget(self.power_button)
        top.addWidget(self.emergency_button)
        root_layout.addLayout(top)

        warning = QLabel("⚠ 操作前确认夹爪已固定、运动范围内无人且可随时切断电源。切换 CAN 或电机 ID 前必须下电。")
        warning.setWordWrap(True)
        warning.setStyleSheet("background:#422006; color:#fde68a; border:1px solid #92400e; "
                              "border-radius:6px; padding:8px;")
        root_layout.addWidget(warning)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.build_control_panel())
        splitter.addWidget(self.build_program_panel())
        splitter.setSizes([500, 600])

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(800)
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(7, 9, 7, 7)
        log_content = QHBoxLayout()
        expand_log_button = QPushButton("放大日志")
        expand_log_button.setToolTip("在独立窗口中查看完整运行日志")
        expand_log_button.clicked.connect(self.show_log_dialog)
        expand_log_button.setMaximumWidth(100)
        log_content.addWidget(self.log, 1)
        log_content.addWidget(expand_log_button)
        log_layout.addLayout(log_content)
        log_group.setMinimumHeight(75)

        vertical_splitter = QSplitter(Qt.Vertical)
        vertical_splitter.setChildrenCollapsible(False)
        vertical_splitter.addWidget(splitter)
        vertical_splitter.addWidget(log_group)
        vertical_splitter.setStretchFactor(0, 1)
        vertical_splitter.setStretchFactor(1, 0)
        vertical_splitter.setSizes([525, 95])
        vertical_splitter.setToolTip("拖动分隔线可调整运行日志区域大小")
        root_layout.addWidget(vertical_splitter, 1)

    def build_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 8, 0)

        device_group = QGroupBox("设备")
        device_form = QFormLayout(device_group)
        self.can_combo = QComboBox()
        for bus in range(7):
            self.can_combo.addItem(f"CAN{bus}", bus)
        self.id_combo = QComboBox()
        for motor_id in range(9):
            self.id_combo.addItem(str(motor_id), motor_id)
        self.can_combo.currentIndexChanged.connect(self.apply_device)
        self.id_combo.currentIndexChanged.connect(self.apply_device)
        device_form.addRow("CAN 通道", self.can_combo)
        device_form.addRow("电机 ID", self.id_combo)
        layout.addWidget(device_group)

        cards = QGridLayout()
        self.position_card = ValueCard("实时位置", "deg")
        self.velocity_card = ValueCard("实时速度", "deg/s")
        self.torque_card = ValueCard("反馈扭矩", "N·m")
        self.temperature_card = ValueCard("MOS / 转子温度", "°C")
        cards.addWidget(self.position_card, 0, 0)
        cards.addWidget(self.velocity_card, 0, 1)
        cards.addWidget(self.torque_card, 1, 0)
        cards.addWidget(self.temperature_card, 1, 1)
        layout.addLayout(cards)

        control_group = QGroupBox("位置控制")
        control = QVBoxLayout(control_group)
        form = QGridLayout()
        self.target_spin = self.spin(POSITION_MIN, POSITION_MAX, 0, 0.1, " deg", 2)
        self.speed_spin = self.spin(0.1, 5000, 180, 1, " deg/s", 1)
        self.kp_spin = self.spin(0, 500, 300, 1, "", 2)
        self.kd_spin = self.spin(0, 5, 5, 0.1, "", 2)
        form.addWidget(QLabel("目标位置"), 0, 0)
        form.addWidget(self.target_spin, 0, 1)
        form.addWidget(QLabel("峰值速度"), 1, 0)
        form.addWidget(self.speed_spin, 1, 1)
        form.addWidget(QLabel("Kp"), 2, 0)
        form.addWidget(self.kp_spin, 2, 1)
        form.addWidget(QLabel("Kd"), 3, 0)
        form.addWidget(self.kd_spin, 3, 1)
        control.addLayout(form)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(POSITION_MIN * 10), int(POSITION_MAX * 10))
        self.slider.valueChanged.connect(self.slider_changed)
        control.addWidget(self.slider)
        labels = QHBoxLayout()
        self.command_label = QLabel("命令: 0.00 deg")
        self.feedback_label = QLabel("实时: -- deg")
        self.command_label.setProperty("role", "muted")
        self.feedback_label.setProperty("role", "muted")
        labels.addWidget(self.command_label)
        labels.addStretch()
        labels.addWidget(self.feedback_label)
        control.addLayout(labels)

        buttons = QHBoxLayout()
        self.goto_button = QPushButton("前往位置")
        self.goto_button.setProperty("primary", True)
        self.goto_button.clicked.connect(self.go_target)
        self.gains_button = QPushButton("应用 Kp/Kd")
        self.gains_button.clicked.connect(self.apply_gains)
        self.zero_button = QPushButton("位置置零")
        self.zero_button.clicked.connect(self.zero_position)
        self.disable_button = QPushButton("电机失能")
        self.disable_button.clicked.connect(self.toggle_motor_enabled)
        buttons.addWidget(self.goto_button)
        buttons.addWidget(self.gains_button)
        buttons.addWidget(self.zero_button)
        buttons.addWidget(self.disable_button)
        control.addLayout(buttons)
        layout.addWidget(control_group)
        layout.addStretch()
        return panel

    def build_program_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 0, 0, 0)
        group = QGroupBox("动作程序")
        group_layout = QVBoxLayout(group)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(
            ["位置 (deg)", "峰值速度 (deg/s)", "段时间 (s)", "到达后等待 (s)"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.itemChanged.connect(self.table_item_changed)
        group_layout.addWidget(self.table, 1)

        edit_buttons = QHBoxLayout()
        for text, slot in (
            ("添加实时位置", self.add_feedback_point),
            ("添加目标位置", self.add_target_point),
            ("删除选中", self.remove_point),
            ("清空", self.clear_points),
        ):
            button = QPushButton(text)
            button.clicked.connect(slot)
            edit_buttons.addWidget(button)
        group_layout.addLayout(edit_buttons)

        file_buttons = QHBoxLayout()
        save = QPushButton("导出动作程序")
        load = QPushButton("导入动作程序")
        save.clicked.connect(self.save_program)
        load.clicked.connect(self.load_program)
        file_buttons.addWidget(save)
        file_buttons.addWidget(load)
        file_buttons.addStretch()
        group_layout.addLayout(file_buttons)

        playback = QHBoxLayout()
        self.start_button = QPushButton("开始")
        self.start_button.setProperty("primary", True)
        self.pause_button = QPushButton("暂停")
        self.stop_button = QPushButton("停止")
        self.loop_check = QCheckBox("循环播放")
        self.return_speed = self.spin(0.1, 5000, 180, 1, " deg/s", 1)
        self.return_speed.setMaximumWidth(130)
        self.start_button.clicked.connect(self.start_actions)
        self.pause_button.clicked.connect(self.pause_actions)
        self.stop_button.clicked.connect(self.stop_actions)
        playback.addWidget(self.start_button)
        playback.addWidget(self.pause_button)
        playback.addWidget(self.stop_button)
        playback.addWidget(self.loop_check)
        playback.addWidget(QLabel("回起点速度"))
        playback.addWidget(self.return_speed)
        group_layout.addLayout(playback)
        self.action_status = QLabel("动作待机")
        self.action_status.setProperty("role", "muted")
        group_layout.addWidget(self.action_status)
        hint = QLabel("速度和段时间使用五次 S 曲线关系自动换算；第一行是动作起点。"
                      "停止后电机保持当前位置，只有“电机失能”或下电会释放。")
        hint.setWordWrap(True)
        hint.setProperty("role", "muted")
        group_layout.addWidget(hint)
        layout.addWidget(group)
        return panel

    @staticmethod
    def spin(low, high, value, step, suffix, decimals):
        widget = QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setSingleStep(step)
        widget.setSuffix(suffix)
        widget.setDecimals(decimals)
        return widget

    def append_log(self, text):
        stamp = time.strftime("%H:%M:%S")
        message = f"[{stamp}] {text}"
        self.log.appendPlainText(message)
        self.scroll_log_to_bottom(self.log)
        if self.log_dialog_output:
            self.log_dialog_output.appendPlainText(message)
            self.scroll_log_to_bottom(self.log_dialog_output)

    @staticmethod
    def scroll_log_to_bottom(output):
        bar = output.verticalScrollBar()
        bar.setValue(bar.maximum())

    def clear_logs(self):
        self.log.clear()
        if self.log_dialog_output:
            self.log_dialog_output.clear()

    def show_log_dialog(self):
        if self.log_dialog and self.log_dialog.isVisible():
            self.log_dialog.showNormal()
            self.log_dialog.raise_()
            self.log_dialog.activateWindow()
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("BXI 夹爪控制演示 - 运行日志")
        dialog.resize(1080, 720)
        dialog.setAttribute(Qt.WA_DeleteOnClose, True)
        layout = QVBoxLayout(dialog)
        output = QPlainTextEdit()
        output.setReadOnly(True)
        output.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        output.document().setMaximumBlockCount(4000)
        output.setPlainText(self.log.toPlainText())
        self.scroll_log_to_bottom(output)
        buttons = QHBoxLayout()
        buttons.addStretch()
        clear_button = QPushButton("清空日志")
        close_button = QPushButton("关闭")
        clear_button.clicked.connect(self.clear_logs)
        close_button.clicked.connect(dialog.close)
        buttons.addWidget(clear_button)
        buttons.addWidget(close_button)
        layout.addWidget(output, 1)
        layout.addLayout(buttons)
        self.log_dialog = dialog
        self.log_dialog_output = output
        dialog.finished.connect(self.log_dialog_closed)
        dialog.show()

    def log_dialog_closed(self, _result=None):
        self.log_dialog = None
        self.log_dialog_output = None

    def load_settings(self):
        can_bus = int(self.settings.value("can_bus", 2))
        motor_id = int(self.settings.value("motor_id", 1))
        self.can_combo.setCurrentIndex(max(0, self.can_combo.findData(can_bus)))
        self.id_combo.setCurrentIndex(max(0, self.id_combo.findData(motor_id)))
        self.speed_spin.setValue(float(self.settings.value("speed", 180.0)))
        self.kp_spin.setValue(float(self.settings.value("kp", 300.0)))
        self.kd_spin.setValue(float(self.settings.value("kd", 5.0)))
        self.return_speed.setValue(float(self.settings.value("return_speed", 180.0)))

    def save_settings(self):
        self.settings.setValue("can_bus", self.can_combo.currentData())
        self.settings.setValue("motor_id", self.id_combo.currentData())
        self.settings.setValue("speed", self.speed_spin.value())
        self.settings.setValue("kp", self.kp_spin.value())
        self.settings.setValue("kd", self.kd_spin.value())
        self.settings.setValue("return_speed", self.return_speed.value())

    def toggle_connection(self):
        if self.connected:
            self.safe_disconnect()
            return
        self.connect_button.setEnabled(False)
        self.append_log("正在连接本机夹爪后端…")
        if self.open_socket():
            return
        self.start_backend()

    def start_backend(self):
        if not os.path.isfile(BACKEND_PATH):
            self.append_log(f"未找到后端：{BACKEND_PATH}")
            QMessageBox.critical(
                self, "后端未编译",
                "未找到夹爪后端程序。请先在软件目录执行：\n\nbash scripts/build_backend.sh",
            )
            self.connect_button.setEnabled(True)
            return
        self.backend = QProcess(self)
        self.backend.setProcessChannelMode(QProcess.MergedChannels)
        self.backend.readyReadStandardOutput.connect(self.read_backend_output)
        self.backend.finished.connect(self.backend_finished)
        if os.geteuid() == 0:
            self.backend.start(BACKEND_PATH, [])
        else:
            self.append_log("硬件访问需要系统授权，请在弹窗中确认")
            self.backend.start(
                "pkexec",
                ["env", "PATH=/usr/sbin:/usr/bin:/sbin:/bin", BACKEND_PATH],
            )
        self.backend_owned = True
        self.connect_deadline = time.monotonic() + 20.0
        self.connect_timer.start()

    def read_backend_output(self):
        if not self.backend:
            return
        text = bytes(self.backend.readAllStandardOutput()).decode("utf-8", "replace")
        for line in text.splitlines():
            self.append_log(f"后端: {line}")

    def backend_finished(self, code, _status):
        self.connect_timer.stop()
        if not self.connected and self.connect_deadline:
            self.append_log(f"后端已退出，退出码 {code}")
            self.connect_button.setEnabled(True)
        self.backend = None
        self.backend_owned = False

    def try_connect_after_start(self):
        if self.open_socket():
            self.connect_timer.stop()
        elif time.monotonic() >= self.connect_deadline:
            self.connect_timer.stop()
            self.connect_button.setEnabled(True)
            self.append_log("连接超时，请检查授权结果、PCI 驱动和硬件连接")
            QMessageBox.warning(self, "连接超时", "无法连接夹爪后端，请查看下方日志。")

    def open_socket(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.25)
            sock.connect((SERVER_HOST, SERVER_PORT))
            greeting = sock.recv(1024)
            if b"HELLO BXI_GRIPPER_DEMO 1" not in greeting:
                sock.close()
                return False
            sock.setblocking(False)
        except OSError:
            try:
                sock.close()
            except (UnboundLocalError, OSError):
                pass
            return False
        self.sock = sock
        self.connected = True
        self.rx_buffer = b""
        self.rx_timer.start()
        self.status_timer.start()
        self.connect_button.setEnabled(True)
        self.append_log("已连接夹爪后端")
        for raw_line in greeting.split(b"\n"):
            line = raw_line.decode("utf-8", "replace").strip()
            if line and not line.startswith("HELLO "):
                self.handle_line(line)
        self.apply_device()
        self.send_command(f"SET_GAINS {self.kp_spin.value():.2f} {self.kd_spin.value():.2f}")
        self.update_ui()
        return True

    def safe_disconnect(self):
        if self.connected:
            self.send_command("CLAW_DISABLE")
            self.send_command("MOTOR_POWER_OFF")
        self.disconnect_socket()

    def disconnect_socket(self):
        self.rx_timer.stop()
        self.status_timer.stop()
        self.slider_timer.stop()
        self.action_wait_timer.stop()
        self.action_running = False
        self.action_paused = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected = False
        self.power_ready = False
        self.power_requested = False
        self.motor_disabled = False
        self.motor_enabling = False
        self.zero_in_progress = False
        self.rx_buffer = b""
        self.append_log("已断开连接")
        for card in (self.position_card, self.velocity_card, self.torque_card,
                     self.temperature_card):
            card.clear()
        self.feedback_label.setText("实时: -- deg")
        self.update_ui()

    def send_command(self, command, quiet=False):
        if not self.connected or not self.sock:
            if not quiet:
                self.append_log("命令未发送：后端未连接")
            return False
        try:
            self.sock.sendall((command.rstrip() + "\n").encode("utf-8"))
            if not quiet and not command.startswith("MOTOR_POWER_STATUS"):
                self.append_log(f"> {command}")
            return True
        except OSError as exc:
            self.append_log(f"发送失败：{exc}")
            self.disconnect_socket()
            return False

    def receive_data(self):
        if not self.sock:
            return
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    self.append_log("后端关闭了连接")
                    self.disconnect_socket()
                    return
                self.rx_buffer += data
                lines = self.rx_buffer.split(b"\n")
                self.rx_buffer = lines.pop()
                for line in lines:
                    self.handle_line(line.decode("utf-8", "replace").strip())
        except BlockingIOError:
            pass
        except OSError as exc:
            self.append_log(f"连接中断：{exc}")
            self.disconnect_socket()

    def handle_line(self, line):
        if not line:
            return
        parts = line.split()
        key = parts[0]
        try:
            if key == "POS":
                self.feedback_position = float(parts[1])
                self.position_card.set_value(self.feedback_position)
                self.feedback_label.setText(f"实时: {self.feedback_position:.2f} deg")
                return
            if key == "VEL":
                self.velocity_card.set_value(float(parts[1]))
                return
            if key == "TORQUE":
                self.torque_card.set_value(float(parts[1]))
                return
            if key == "TEMP_MOS":
                self.last_mos_temp = float(parts[1])
                return
            if key == "TEMP_ROTOR":
                rotor = float(parts[1])
                mos = getattr(self, "last_mos_temp", math.nan)
                self.temperature_card.value.setText(f"{mos:.1f} / {rotor:.1f} °C")
                return
        except (ValueError, IndexError):
            return

        if key == "MOTOR_POWERING_ON":
            self.power_requested = True
            self.motor_disabled = False
            self.motor_enabling = False
            self.zero_in_progress = False
            self.action_status.setText("电机启动中，等待反馈…")
        elif key == "MOTOR_POWER_READY":
            self.power_ready = len(parts) > 1 and parts[1] == "1"
            self.power_requested = self.power_ready or self.power_requested
            if self.power_ready:
                self.motor_disabled = False
                self.motor_enabling = False
                self.action_status.setText("电机已上电并保持当前位置")
        elif key == "MOTOR_POWER_OFF_COMPLETE":
            self.power_ready = False
            self.power_requested = False
            self.motor_disabled = False
            self.motor_enabling = False
            self.zero_in_progress = False
            self.stop_actions(send_stop=False)
            self.action_status.setText("电机已下电")
        elif key == "CLAW_MOVE_COMPLETE":
            if self.action_running:
                self.action_reached()
            else:
                self.action_status.setText("已到达目标位置并保持")
        elif key == "CLAW_ZERO_COMPLETE":
            self.zero_in_progress = False
            self.power_ready = True
            self.motor_disabled = False
            self.motor_enabling = False
            self.set_command_position(0.0, move_slider=True)
            self.action_status.setText("位置置零完成")
        elif key == "CLAW_ZERO_STARTED":
            self.zero_in_progress = True
            self.action_status.setText("正在执行失能 → 置零 → 重新使能")
        elif key == "CLAW_ZERO_RETRY":
            attempt = parts[1] if len(parts) > 1 else "下一次"
            self.action_status.setText(f"首次反馈尚未稳定，正在自动重试置零（第 {attempt} 次）")
        elif key == "CLAW_DISABLED":
            self.power_ready = False
            self.motor_disabled = True
            self.motor_enabling = False
            self.stop_actions(send_stop=False)
            self.action_status.setText("电机已失能；硬件电源仍开启")
        elif key == "CLAW_ENABLING":
            self.motor_disabled = True
            self.motor_enabling = True
            self.action_status.setText("正在重新使能，等待电机反馈…")
        elif key == "CLAW_HOLDING":
            self.action_status.setText("已停止，保持当前位置")
        elif key == "GAINS_SET":
            self.action_status.setText(f"Kp/Kd 已应用：{parts[1]} / {parts[2]}")
        elif key == "ERROR":
            backend_message = " ".join(parts[1:])
            if backend_message in {
                "claw zero command failed",
                "claw zero verification failed",
                "claw zero feedback timeout",
            }:
                self.zero_in_progress = False
                self.power_ready = False
                self.motor_disabled = True
                self.motor_enabling = False
            elif backend_message == "claw zero requires motor power":
                self.zero_in_progress = False
            message = self.translate_error(backend_message)
            self.action_status.setText(f"错误：{message}")
            self.append_log(f"! {message}")
        elif key == "LOG":
            self.append_log("< " + line[4:])
        elif key not in {"PONG", "HELLO", "CLAW_MOVE_STARTED", "CLAW_CAN_SET", "MOTOR_ID_SET"}:
            self.append_log("< " + line)
        self.update_ui()

    @staticmethod
    def translate_error(message):
        return {
            "motor is not ready": "电机尚未准备好",
            "claw zero requires motor power": "请先给夹爪电机上电",
            "claw enable requires motor power": "硬件电源未开启，无法使能电机",
            "claw zero command failed": "置零报文发送失败",
            "claw zero verification failed": "置零后位置反馈超出允许范围，电机已失能",
            "claw zero feedback timeout": "置零后未收到反馈，电机已失能",
            "motor power on failed": "硬件上电失败",
            "power off before changing CAN": "请先下电再切换 CAN 通道",
            "power off before changing motor ID": "请先下电再切换电机 ID",
        }.get(message, message)

    def poll_status(self):
        self.send_command("MOTOR_POWER_STATUS", quiet=True)

    def apply_device(self):
        if not self.connected:
            return
        self.send_command(f"SET_CLAW_CAN {self.can_combo.currentData()}")
        self.send_command(f"SET_MOTOR_ID {self.id_combo.currentData()}")

    def toggle_power(self):
        if not self.connected:
            return
        if self.power_requested or self.power_ready:
            self.stop_actions()
            self.send_command("MOTOR_POWER_OFF")
        else:
            self.send_command("MOTOR_POWER_ON")

    def emergency_stop(self):
        self.slider_pending = None
        self.slider_timer.stop()
        self.stop_actions(send_stop=False)
        if self.connected:
            self.send_command("CLAW_DISABLE")
            self.send_command("MOTOR_POWER_OFF")
        self.action_status.setText("已执行急停并下电")

    def toggle_motor_enabled(self):
        if self.motor_disabled:
            if self.send_command("CLAW_ENABLE"):
                self.motor_enabling = True
                self.action_status.setText("正在重新使能，等待电机反馈…")
                self.update_ui()
            return
        self.stop_actions(send_stop=False)
        self.send_command("CLAW_DISABLE")

    def zero_position(self):
        if self.zero_in_progress:
            return
        answer = QMessageBox.question(
            self, "确认位置置零",
            "置零会短暂失能电机，并把当前位置设为 0°。确认机械位置安全吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.slider_pending = None
            self.slider_timer.stop()
            self.stop_actions()
            if self.send_command("CLAW_ZERO"):
                self.zero_in_progress = True
                self.action_status.setText("正在执行失能 → 置零 → 重新使能")
                self.update_ui()

    def apply_gains(self):
        self.send_command(f"SET_GAINS {self.kp_spin.value():.2f} {self.kd_spin.value():.2f}")

    def send_move(self, position, speed, stream=False):
        name = "CLAW_STREAM" if stream else "CLAW_MOVE"
        return self.send_command(
            f"{name} {position:.2f} {speed:.2f} "
            f"{self.kp_spin.value():.2f} {self.kd_spin.value():.2f}",
            quiet=stream,
        )

    def go_target(self):
        self.stop_actions()
        target = self.target_spin.value()
        self.set_command_position(target)
        if self.send_move(target, self.speed_spin.value()):
            self.action_status.setText(f"正在前往 {target:.2f}°")

    def set_command_position(self, position, move_slider=False):
        self.command_position = position
        self.target_spin.blockSignals(True)
        self.target_spin.setValue(position)
        self.target_spin.blockSignals(False)
        if move_slider:
            self.slider.blockSignals(True)
            self.slider.setValue(round(position * 10))
            self.slider.blockSignals(False)
        self.command_label.setText(f"命令: {position:.2f} deg")

    def slider_changed(self, value):
        if self.action_running or self.action_paused:
            self.stop_actions()
        position = value / 10.0
        self.set_command_position(position)
        self.slider_pending = position
        if not self.slider_timer.isActive():
            self.slider_timer.start()

    def flush_slider(self):
        if self.slider_pending is None:
            self.slider_timer.stop()
            return
        position = self.slider_pending
        self.slider_pending = None
        self.send_move(position, self.speed_spin.value(), stream=True)

    def add_feedback_point(self):
        self.add_point(self.feedback_position)

    def add_target_point(self):
        self.add_point(self.target_spin.value())

    def add_point(self, position, speed=None, duration=None, wait=0.0, mode="speed"):
        row = self.table.rowCount()
        self.table.blockSignals(True)
        self.table.insertRow(row)
        self.set_cell(row, 0, position)
        self.set_cell(row, 3, wait)
        if row == 0:
            for column in (1, 2):
                item = QTableWidgetItem("起点")
                item.setTextAlignment(Qt.AlignCenter)
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                item.setForeground(QColor("#94a3b8"))
                self.table.setItem(row, column, item)
        else:
            if speed is None:
                speed = self.speed_spin.value()
            self.set_cell(row, 1, speed)
            previous = self.cell_value(row - 1, 0)
            if duration is None:
                duration = abs(position - previous) * PEAK_FACTOR / max(speed, 0.001)
            self.set_cell(row, 2, duration)
            self.table.item(row, 0).setData(Qt.UserRole, mode)
            self.color_calculated_cell(row)
        self.table.setVerticalHeaderItem(row, QTableWidgetItem(f"点 {row + 1}"))
        self.table.blockSignals(False)
        self.table.selectRow(row)

    def set_cell(self, row, column, value):
        item = QTableWidgetItem(f"{float(value):.2f}")
        item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, column, item)

    def cell_value(self, row, column):
        try:
            return float(self.table.item(row, column).text())
        except (AttributeError, ValueError):
            return None

    def row_mode(self, row):
        item = self.table.item(row, 0)
        mode = item.data(Qt.UserRole) if item else None
        return mode if mode in ("speed", "time") else "speed"

    def color_calculated_cell(self, row):
        mode = self.row_mode(row)
        for column in (1, 2):
            item = self.table.item(row, column)
            if item:
                calculated = (mode == "speed" and column == 2) or (mode == "time" and column == 1)
                item.setForeground(QColor("#60a5fa" if calculated else "#e5e7eb"))

    def recalculate_row(self, row):
        if row <= 0 or row >= self.table.rowCount():
            return False
        previous = self.cell_value(row - 1, 0)
        position = self.cell_value(row, 0)
        if previous is None or position is None:
            return False
        distance = abs(position - previous)
        mode = self.row_mode(row)
        if mode == "time":
            duration = self.cell_value(row, 2)
            if duration is None or duration <= 0:
                return False
            self.set_cell(row, 1, distance * PEAK_FACTOR / duration)
        else:
            speed = self.cell_value(row, 1)
            if speed is None or speed <= 0:
                return False
            self.set_cell(row, 2, distance * PEAK_FACTOR / speed)
        self.color_calculated_cell(row)
        return True

    def table_item_changed(self, item):
        row, column = item.row(), item.column()
        if row == 0 and column in (1, 2):
            return
        value = self.cell_value(row, column)
        if value is None or not math.isfinite(value):
            self.action_status.setText("动作表中存在无效数字")
            return
        if column == 0 and not POSITION_MIN <= value <= POSITION_MAX:
            self.action_status.setText("位置必须在 -360° 到 360° 之间")
            return
        if column == 3 and value < 0:
            self.action_status.setText("等待时间不能小于 0")
            return
        self.table.blockSignals(True)
        if row > 0 and column in (1, 2):
            self.table.item(row, 0).setData(Qt.UserRole, "speed" if column == 1 else "time")
        valid = self.recalculate_row(row) if row > 0 else True
        if column == 0 and row + 1 < self.table.rowCount():
            valid = self.recalculate_row(row + 1) and valid
        self.table.blockSignals(False)
        self.action_status.setText("动作点已更新" if valid else "请检查速度或段时间")

    def remove_point(self):
        row = self.table.currentRow()
        if row < 0:
            return
        self.table.blockSignals(True)
        self.table.removeRow(row)
        for index in range(self.table.rowCount()):
            self.table.setVerticalHeaderItem(index, QTableWidgetItem(f"点 {index + 1}"))
        if self.table.rowCount():
            first = self.table.item(0, 0)
            if first:
                first.setData(Qt.UserRole, "speed")
            for column in (1, 2):
                item = QTableWidgetItem("起点")
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(0, column, item)
            if row > 0 and row < self.table.rowCount():
                self.recalculate_row(row)
        self.table.blockSignals(False)

    def clear_points(self):
        self.stop_actions()
        self.table.setRowCount(0)

    def read_points(self):
        points = []
        for row in range(self.table.rowCount()):
            position = self.cell_value(row, 0)
            wait = self.cell_value(row, 3)
            if position is None or not POSITION_MIN <= position <= POSITION_MAX or wait is None or wait < 0:
                return None
            speed = self.return_speed.value() if row == 0 else self.cell_value(row, 1)
            duration = 0.0 if row == 0 else self.cell_value(row, 2)
            if speed is None or speed <= 0 or duration is None or duration < 0:
                return None
            points.append({"position": position, "speed": speed, "duration": duration,
                           "wait": wait, "mode": self.row_mode(row)})
        return points

    def start_actions(self):
        if self.action_paused:
            self.action_paused = False
            self.action_running = True
            self.command_action_point()
            return
        points = self.read_points()
        if not points or len(points) < 2:
            self.action_status.setText("至少需要两个有效动作点")
            return
        if not self.power_ready:
            self.action_status.setText("请先等待电机上电并收到反馈")
            return
        self.action_points = points
        self.action_index = 0
        self.action_running = True
        self.action_paused = False
        self.command_action_point()

    def command_action_point(self):
        point = self.action_points[self.action_index]
        speed = point["speed"]
        if self.action_index == 0 and self.loop_check.isChecked():
            speed = self.return_speed.value()
        self.set_command_position(point["position"], move_slider=True)
        if self.send_move(point["position"], speed):
            self.action_status.setText(
                f"动作点 {self.action_index + 1}/{len(self.action_points)}："
                f"前往 {point['position']:.2f}°")
        else:
            self.stop_actions(send_stop=False)

    def action_reached(self):
        point = self.action_points[self.action_index]
        wait_ms = round(point["wait"] * 1000)
        if wait_ms > 0:
            self.action_status.setText(
                f"动作点 {self.action_index + 1} 已到达，等待 {point['wait']:.2f} 秒")
            self.action_wait_timer.start(wait_ms)
        else:
            self.advance_action()

    def advance_action(self):
        if not self.action_running:
            return
        self.action_index += 1
        if self.action_index >= len(self.action_points):
            if self.loop_check.isChecked():
                self.action_index = 0
            else:
                self.action_running = False
                self.action_status.setText("动作程序完成，电机保持当前位置")
                self.update_ui()
                return
        self.command_action_point()

    def pause_actions(self):
        if not self.action_running:
            return
        self.action_wait_timer.stop()
        self.action_running = False
        self.action_paused = True
        self.send_command("CLAW_STOP")
        self.action_status.setText("动作已暂停，电机保持当前位置")
        self.update_ui()

    def stop_actions(self, send_stop=True):
        was_active = self.action_running or self.action_paused
        self.action_wait_timer.stop()
        self.action_running = False
        self.action_paused = False
        self.action_index = 0
        if was_active and send_stop and self.connected:
            self.send_command("CLAW_STOP")
        if was_active:
            self.action_status.setText("动作已停止，电机保持当前位置")
        self.update_ui()

    def save_program(self):
        points = self.read_points()
        if not points:
            QMessageBox.warning(self, "无法导出", "请先添加有效的动作点。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出动作程序", "gripper_action.json",
                                              "JSON 文件 (*.json)")
        if not path:
            return
        document = {"format": "bxi-gripper-action", "version": 1, "points": points,
                    "return_speed": self.return_speed.value()}
        try:
            with open(path, "w", encoding="utf-8") as file:
                json.dump(document, file, ensure_ascii=False, indent=2)
            self.append_log(f"动作程序已导出：{path}")
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    def load_program(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入动作程序", "", "JSON 文件 (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as file:
                document = json.load(file)
            if document.get("format") != "bxi-gripper-action" or document.get("version") != 1:
                raise ValueError("不是受支持的 BXI 夹爪动作程序")
            points = document["points"]
            if not isinstance(points, list) or len(points) < 1:
                raise ValueError("动作点为空")
            self.table.setRowCount(0)
            for point in points:
                self.add_point(float(point["position"]), float(point["speed"]),
                               float(point["duration"]), float(point["wait"]),
                               str(point.get("mode", "speed")))
            self.return_speed.setValue(float(document.get("return_speed", 180.0)))
            if not self.read_points():
                raise ValueError("动作程序参数超出允许范围")
            self.append_log(f"动作程序已导入：{path}")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.table.setRowCount(0)
            QMessageBox.critical(self, "导入失败", str(exc))

    def update_ui(self):
        self.connection_label.setText("● 已连接" if self.connected else "● 未连接")
        self.connection_label.setStyleSheet("color:#4ade80" if self.connected else "color:#94a3b8")
        self.connect_button.setText("断开连接" if self.connected else "启动并连接")
        self.power_button.setText("夹爪电机下电" if (self.power_ready or self.power_requested)
                                  else "夹爪电机上电")
        self.disable_button.setText(
            "使能中…" if self.motor_enabling
            else "重新使能" if self.motor_disabled
            else "电机失能"
        )
        self.power_button.setEnabled(self.connected)
        self.emergency_button.setEnabled(self.connected)
        can_change = self.connected and not (self.power_ready or self.power_requested)
        self.can_combo.setEnabled(can_change)
        self.id_combo.setEnabled(can_change)
        controls_ready = (
            self.connected and self.power_ready and not self.zero_in_progress
        )
        for widget in (self.goto_button, self.gains_button, self.zero_button,
                       self.start_button):
            widget.setEnabled(controls_ready)
        self.disable_button.setEnabled(
            not self.zero_in_progress and self.connected and (
                self.power_ready or (self.motor_disabled and not self.motor_enabling)
            )
        )
        self.slider.setEnabled(controls_ready)
        self.pause_button.setEnabled(self.action_running)
        self.stop_button.setEnabled(self.action_running or self.action_paused)

    def closeEvent(self, event):
        self.save_settings()
        if self.connected:
            self.send_command("CLAW_DISABLE", quiet=True)
            self.send_command("MOTOR_POWER_OFF", quiet=True)
            if self.backend_owned:
                self.send_command("SHUTDOWN", quiet=True)
        self.disconnect_socket()
        if self.backend and self.backend.state() != QProcess.NotRunning:
            self.backend.waitForFinished(1500)
            if self.backend.state() != QProcess.NotRunning:
                self.backend.terminate()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("BXI Gripper Demo")
    app.setOrganizationName("BXI")
    font = QFont()
    font.setFamilies(["Noto Sans CJK SC", "Noto Sans SC", "Ubuntu", "DejaVu Sans"])
    font.setPointSize(10)
    app.setFont(font)
    window = GripperDemo()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
