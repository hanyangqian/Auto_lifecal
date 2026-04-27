# -*- coding: utf-8 -*-
import argparse
import sys
import time

import serial
from serial.tools import list_ports
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


DEFAULT_BAUD = 115200
READ_TIMEOUT_S = 0.4
WRITE_TIMEOUT_S = 0.4


class UTG9005CIIController:
    """Serial controller for UNI-T UTG9000C-II UCI commands."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.ser: serial.Serial | None = None

    def open(self):
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            timeout=READ_TIMEOUT_S,
            write_timeout=WRITE_TIMEOUT_S,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, command: str):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未连接")
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()
        time.sleep(0.05)

    def query_text(self, command: str) -> str:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未连接")
        self.ser.reset_input_buffer()
        self.send(command)
        deadline = time.time() + READ_TIMEOUT_S
        chunks: list[bytes] = []
        while time.time() < deadline:
            waiting = self.ser.in_waiting
            if waiting:
                chunks.append(self.ser.read(waiting))
                time.sleep(0.03)
            else:
                time.sleep(0.01)
        return b"".join(chunks).decode("ascii", errors="ignore").strip("\x00\r\n ")

    def idn(self) -> str:
        return self.query_text("IDN?")

    def set_remote(self, remote: bool):
        self.send(f"local:{0 if remote else 1};")

    def set_output(self, enable: bool, channel: int = 0, rear_output: bool = False):
        if channel not in (0, 1):
            raise ValueError("channel 只能是 0(CH1) 或 1(CH2)")
        value = 2 if enable and rear_output else 1 if enable else 0
        self.send(f"wp@CH:{channel}@addr:0x8001@v:{value};")


class UTG9005CIIWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ctrl: UTG9005CIIController | None = None
        self.setWindowTitle("UTG9005C-II OUTPUT 控制")
        self.resize(430, 250)
        self._build_ui()
        self.refresh_ports()

    def _build_ui(self):
        root = QWidget()
        main_layout = QVBoxLayout(root)

        form = QFormLayout()
        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.clicked.connect(self.refresh_ports)
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.refresh_button)
        form.addRow("串口", port_row)

        self.baud_edit = QLineEdit(str(DEFAULT_BAUD))
        form.addRow("波特率", self.baud_edit)

        self.channel_combo = QComboBox()
        self.channel_combo.addItems(["CH1", "CH2"])
        form.addRow("通道", self.channel_combo)
        main_layout.addLayout(form)

        self.remote_check = QCheckBox("连接后进入 Remote")
        self.remote_check.setChecked(True)
        self.rear_check = QCheckBox("OUTPUT ON 使用后面板端子")
        main_layout.addWidget(self.remote_check)
        main_layout.addWidget(self.rear_check)

        self.connect_button = QPushButton("连接 / 识别")
        self.output_on_button = QPushButton("OUTPUT ON")
        self.output_off_button = QPushButton("OUTPUT OFF")
        self.disconnect_button = QPushButton("断开")
        self.connect_button.clicked.connect(self.connect_device)
        self.output_on_button.clicked.connect(lambda: self.set_output(True))
        self.output_off_button.clicked.connect(lambda: self.set_output(False))
        self.disconnect_button.clicked.connect(self.disconnect_device)

        main_layout.addWidget(self.connect_button)
        main_layout.addWidget(self.output_on_button)
        main_layout.addWidget(self.output_off_button)
        main_layout.addWidget(self.disconnect_button)

        self.status_label = QLabel("未连接")
        self.status_label.setWordWrap(True)
        main_layout.addWidget(self.status_label)

        self.setCentralWidget(root)

    def refresh_ports(self):
        current = self.port_combo.currentText()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        self.status_label.setText(f"发现串口: {', '.join(ports) if ports else '无'}")

    def connect_device(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "缺少串口", "请先选择 UTG9005C-II 对应的 COM 口。")
            return
        try:
            self.disconnect_device(show_status=False)
            self.ctrl = UTG9005CIIController(port, int(self.baud_edit.text().strip()))
            self.ctrl.open()
            if self.remote_check.isChecked():
                self.ctrl.set_remote(True)
            idn = self.ctrl.idn() or "未返回 IDN"
            self.status_label.setText(f"已连接 {port}: {idn}")
        except Exception as ex:
            self.ctrl = None
            QMessageBox.critical(self, "连接失败", str(ex))
            self.status_label.setText("连接失败")

    def disconnect_device(self, show_status: bool = True):
        if self.ctrl:
            self.ctrl.close()
            self.ctrl = None
        if show_status:
            self.status_label.setText("已断开")

    def set_output(self, enable: bool):
        if not self.ctrl:
            self.connect_device()
        if not self.ctrl:
            return
        channel = 0 if self.channel_combo.currentText() == "CH1" else 1
        try:
            self.ctrl.set_output(enable, channel=channel, rear_output=self.rear_check.isChecked())
            state = "ON" if enable else "OFF"
            self.status_label.setText(f"{self.channel_combo.currentText()} OUTPUT {state} 命令已发送")
        except Exception as ex:
            QMessageBox.critical(self, "OUTPUT 控制失败", str(ex))

    def closeEvent(self, event):
        self.disconnect_device(show_status=False)
        super().closeEvent(event)


def run_cli(args):
    ctrl = UTG9005CIIController(args.port, args.baud)
    try:
        ctrl.open()
        if args.remote:
            ctrl.set_remote(True)
        if args.idn:
            print(ctrl.idn())
        ctrl.set_output(args.output == "on", channel=args.channel - 1, rear_output=args.rear)
        print(f"CH{args.channel} OUTPUT {args.output.upper()} command sent")
    finally:
        ctrl.close()


def main():
    parser = argparse.ArgumentParser(description="UNI-T UTG9005C-II OUTPUT controller")
    parser.add_argument("--port", help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--channel", type=int, choices=(1, 2), default=1)
    parser.add_argument("--output", choices=("on", "off"))
    parser.add_argument("--rear", action="store_true", help="Use rear output terminal when enabling output")
    parser.add_argument("--remote", action="store_true", help="Send local:0; before OUTPUT command")
    parser.add_argument("--idn", action="store_true", help="Print IDN? response before OUTPUT command")
    args = parser.parse_args()

    if args.port and args.output:
        run_cli(args)
        return

    app = QApplication(sys.argv)
    window = UTG9005CIIWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
