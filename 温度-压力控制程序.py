from __future__ import annotations

import csv
import os
import re
import sys
import time
import traceback
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from PyQt5.QtCore import QSettings, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


BAUD_CANDIDATES = (9600, 115200)
QUERY_WAIT_S = 0.35
TEMP_SLAVE = 1
PRESS_SLAVE = 2


def log_stamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def dir_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_axis_path(text: str, name: str) -> list[dict]:
    text = text.strip()
    if not text:
        return []

    targets = []
    segments = [part.strip() for part in re.split(r"[;；\n]+", text) if part.strip()]
    for segment in segments:
        match = re.match(
            r"^\s*([-+]?\d+(?:\.\d+)?)\s*(?:->|→|-|,|:|：)\s*(h|\d+(?:\.\d+)?)\s*(?:->|→|-|,|:|：)\s*([-+]?\d+(?:\.\d+)?)\s*$",
            segment,
            re.IGNORECASE,
        )
        if not match:
            raise RuntimeError(f"{name}路径格式错误，请使用: 起点-速率-终点 或 起点-h-终点")

        start_text, mode_text, end_text = match.groups()
        start = float(start_text)
        end = float(end_text)
        if mode_text.lower() == "h":
            mode = "HOLD"
            rate = None
        else:
            mode = "RAMP"
            rate = float(mode_text)
            if rate <= 0:
                raise RuntimeError(f"{name}路径速率必须大于 0（当前: {rate}）")

        if targets and abs(targets[-1]["value"] - start) > 1e-9:
            raise RuntimeError(
                f"{name}路径不连续：上一段终点 {targets[-1]['value']:.6g}，下一段起点 {start:.6g}"
            )
        if not targets:
            targets.append({"value": start, "rate": rate, "mode": mode})
        targets.append({"value": end, "rate": rate, "mode": mode})

    return targets


def build_stages(temp_targets: list[dict], press_targets: list[dict]) -> list[dict]:
    count = max(len(temp_targets), len(press_targets))
    stages = []
    for idx in range(count):
        temp = temp_targets[idx] if idx < len(temp_targets) else None
        press = press_targets[idx] if idx < len(press_targets) else None
        stages.append(
            {
                "temp": temp,
                "press": press,
            }
        )
    return stages


def extract_float_list(resp: str) -> list[float]:
    if not resp:
        return []
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)
    return [float(x) for x in nums]


def extract_rt_channels(resp: str):
    if not resp:
        return None, None, None, None, None, None

    frames = re.findall(r"MK:\d+:[^\r\n]*", resp, flags=re.IGNORECASE)
    by_slave = {}
    for fr in frames:
        parts = fr.split(":")
        if len(parts) < 10:
            continue
        try:
            sx = int(float(parts[1]))
        except Exception:
            continue
        by_slave[sx] = parts

    tc = pk = tf = pf = temp_status = press_status = None
    p1 = by_slave.get(TEMP_SLAVE)
    if p1 is not None:
        try:
            tc = float(p1[2])
        except Exception:
            pass
        try:
            tf = float(p1[4])
        except Exception:
            pass
        try:
            temp_status = int(float(p1[8]))
        except Exception:
            pass

    p2 = by_slave.get(PRESS_SLAVE)
    if p2 is not None:
        try:
            pk = float(p2[2])
        except Exception:
            pass
        try:
            pf = float(p2[4])
        except Exception:
            pass
        try:
            press_status = int(float(p2[8]))
        except Exception:
            pass

    return tc, pk, tf, pf, temp_status, press_status


class HotplateClient:
    def __init__(self, port: str):
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        self.port = port
        self.ser = None
        self.baud = None

    def connect(self):
        errors = []
        for baud in BAUD_CANDIDATES:
            ser = None
            try:
                ser = serial.Serial(
                    self.port,
                    baudrate=baud,
                    timeout=0.12,
                    write_timeout=0.4,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.08)
                self.ser = ser
                self.baud = baud
                if self.query("*IDN?") or self.query("TEMPerature:RTINformation?"):
                    return
            except Exception as ex:
                errors.append(f"{baud}bps -> {ex}")
            finally:
                if self.ser is None and ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
        raise RuntimeError(f"标定台连接失败: {self.port}\n" + "\n".join(errors))

    def close(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        finally:
            self.ser = None

    def send(self, command: str):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("标定台串口未连接")
        self.ser.write((command + "\r").encode("ascii"))
        self.ser.flush()
        time.sleep(0.05)

    def query(self, command: str) -> str:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("标定台串口未连接")
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r").encode("ascii"))
        self.ser.flush()

        deadline = time.time() + QUERY_WAIT_S
        chunks = []
        last_recv = 0.0
        while time.time() < deadline:
            n = self.ser.in_waiting
            if n > 0:
                chunks.append(self.ser.read(n))
                last_recv = time.time()
            if chunks and (time.time() - last_recv) > 0.08:
                break
            time.sleep(0.01)
        return b"".join(chunks).decode("ascii", errors="ignore").strip("\x00\r\n ")

    def read_state(self) -> dict:
        rt_resp = self.query("TEMPerature:RTINformation?")
        tc_rt, pk_rt, tf_rt, pf_rt, temp_status, press_status = extract_rt_channels(rt_resp)

        ct_resp = self.query("TEMPerature:CTEMperature?")
        ct_vals = extract_float_list(ct_resp)
        tc = ct_vals[0] if len(ct_vals) >= 1 else tc_rt
        pk = ct_vals[1] if len(ct_vals) >= 2 else pk_rt

        return {
            "timestamp": log_stamp(),
            "tc": tc,
            "pk": pk,
            "tf": tf_rt,
            "pf": pf_rt,
            "temp_status": temp_status,
            "press_status": press_status,
            "baud": self.baud,
            "rt_resp": rt_resp,
            "ct_resp": ct_resp,
        }


class ControlThread(QThread):
    state_ready = pyqtSignal(dict)
    message = pyqtSignal(str)
    failed = pyqtSignal(str)
    finished_ok = pyqtSignal()

    def __init__(
        self,
        port: str,
        stages: list[dict],
        poll_interval_s: float,
        temp_tol: float,
        temp_hold_s: float,
        press_tol: float,
        press_hold_s: float,
        sensor_log_path: str,
        parent=None,
    ):
        super().__init__(parent)
        self.port = port
        self.stages = stages
        self.poll_interval_s = poll_interval_s
        self.temp_tol = temp_tol
        self.temp_hold_s = temp_hold_s
        self.press_tol = press_tol
        self.press_hold_s = press_hold_s
        self.sensor_log_path = sensor_log_path
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        client = None
        try:
            client = HotplateClient(self.port)
            client.connect()
            self.message.emit(f"[标定台] 已连接 {self.port} @ {client.baud}bps")

            with open(self.sensor_log_path, "a", newline="", encoding="utf-8-sig") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "timestamp",
                        "stage",
                        "tc_C",
                        "pk_kPa",
                        "tf",
                        "pf",
                        "temp_status",
                        "press_status",
                        "temp_stable_s",
                        "press_stable_s",
                        "rt_resp",
                        "ct_resp",
                    ],
                )
                if f.tell() == 0:
                    writer.writeheader()

                for stage_index, stage in enumerate(self.stages, start=1):
                    if self._stop:
                        break
                    self.apply_stage(client, stage_index, stage)
                    temp_since = None
                    press_since = None

                    while not self._stop:
                        state = client.read_state()
                        now = time.monotonic()
                        temp_target = stage["temp"]["value"] if stage.get("temp") else None
                        press_target = stage["press"]["value"] if stage.get("press") else None

                        if temp_target is None:
                            temp_ok = True
                        else:
                            temp_ok = state["tc"] is not None and abs(float(state["tc"]) - float(temp_target)) <= self.temp_tol
                        if press_target is None:
                            press_ok = True
                        else:
                            press_ok = state["pk"] is not None and abs(float(state["pk"]) - float(press_target)) <= self.press_tol

                        if temp_ok:
                            if temp_since is None:
                                temp_since = now
                        else:
                            temp_since = None

                        if press_ok:
                            if press_since is None:
                                press_since = now
                        else:
                            press_since = None

                        temp_stable_s = 0.0 if temp_since is None else now - temp_since
                        press_stable_s = 0.0 if press_since is None else now - press_since
                        state["stage"] = stage_index
                        state["temp_stable_s"] = temp_stable_s
                        state["press_stable_s"] = press_stable_s
                        writer.writerow(
                            {
                                "timestamp": state["timestamp"],
                                "stage": stage_index,
                                "tc_C": state["tc"],
                                "pk_kPa": state["pk"],
                                "tf": state["tf"],
                                "pf": state["pf"],
                                "temp_status": state["temp_status"],
                                "press_status": state["press_status"],
                                "temp_stable_s": f"{temp_stable_s:.3f}",
                                "press_stable_s": f"{press_stable_s:.3f}",
                                "rt_resp": state["rt_resp"],
                                "ct_resp": state["ct_resp"],
                            }
                        )
                        f.flush()
                        self.state_ready.emit(state)

                        temp_done = temp_target is None or self.temp_hold_s <= 0 or temp_stable_s >= self.temp_hold_s
                        press_done = press_target is None or self.press_hold_s <= 0 or press_stable_s >= self.press_hold_s
                        if temp_done and press_done:
                            self.message.emit(f"[阶段 {stage_index}/{len(self.stages)}] 已稳定，进入下一阶段")
                            break

                        slept = 0.0
                        while not self._stop and slept < self.poll_interval_s:
                            time.sleep(min(0.05, self.poll_interval_s - slept))
                            slept += 0.05

            if not self._stop:
                self.message.emit("[任务] 温度压力路径执行完成")
                self.finished_ok.emit()
        except Exception:
            self.failed.emit(traceback.format_exc())
        finally:
            if client is not None:
                client.close()
            self.message.emit("[标定台] 控制线程已停止")

    def apply_stage(self, client: HotplateClient, stage_index: int, stage: dict):
        temp = stage.get("temp")
        press = stage.get("press")
        total = len(self.stages)
        if temp is not None:
            target = float(temp["value"])
            mode = temp.get("mode") or "RAMP"
            client.send(f"TEMPerature:OPSLave {TEMP_SLAVE}")
            if mode == "HOLD":
                client.send(f"TEMPerature:HOLD {target}")
                self.message.emit(f"[阶段 {stage_index}/{total}] 温度 HOLD -> {target:.2f} ℃")
            else:
                rate = float(temp.get("rate") or 5.0)
                client.send(f"TEMPerature:RAMP {target},{rate}")
                self.message.emit(f"[阶段 {stage_index}/{total}] 温度 RAMP -> {target:.2f} ℃ @ {rate:.2f} ℃/min")

        if press is not None:
            target = float(press["value"])
            mode = press.get("mode") or "RAMP"
            client.send(f"TEMPerature:OPSLave {PRESS_SLAVE}")
            if mode == "HOLD":
                client.send(f"TEMPerature:HOLD {target}")
                self.message.emit(f"[阶段 {stage_index}/{total}] 压力 HOLD -> {target:.3f} kPa")
            else:
                rate = float(press.get("rate") or 5.0)
                client.send(f"TEMPerature:RAMP {target},{rate}")
                self.message.emit(f"[阶段 {stage_index}/{total}] 压力 RAMP -> {target:.3f} kPa @ {rate:.3f} kPa/min")


class ControlWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("温度-压力控制程序")
        self.resize(980, 720)
        self.settings = QSettings("AutoLifeCal", "TempPressureControl")
        self.event_log_path = ""
        self.worker = None

        self.build_ui()
        self.load_settings()
        self.refresh_ports()

    def build_ui(self):
        layout = QVBoxLayout(self)
        grid = QGridLayout()

        self.save_dir_edit = QLineEdit()
        self.save_dir_btn = QPushButton("选择保存文件夹")
        self.save_dir_btn.clicked.connect(self.choose_save_dir)
        grid.addWidget(QLabel("保存文件夹"), 0, 0)
        grid.addWidget(self.save_dir_edit, 0, 1)
        grid.addWidget(self.save_dir_btn, 0, 2)

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("刷新COM")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        grid.addWidget(QLabel("标定台COM"), 1, 0)
        grid.addWidget(self.port_combo, 1, 1)
        grid.addWidget(self.refresh_btn, 1, 2)

        self.poll_spin = QDoubleSpinBox()
        self.poll_spin.setRange(0.2, 60.0)
        self.poll_spin.setDecimals(1)
        self.poll_spin.setValue(1.0)
        self.poll_spin.setSuffix(" s")
        grid.addWidget(QLabel("读取间隔"), 2, 0)
        grid.addWidget(self.poll_spin, 2, 1)

        self.temp_tol_spin = QDoubleSpinBox()
        self.temp_tol_spin.setRange(0.0, 1000.0)
        self.temp_tol_spin.setDecimals(3)
        self.temp_tol_spin.setValue(0.2)
        self.temp_tol_spin.setSuffix(" ℃")
        self.temp_hold_spin = QDoubleSpinBox()
        self.temp_hold_spin.setRange(0.0, 86400.0)
        self.temp_hold_spin.setDecimals(1)
        self.temp_hold_spin.setValue(300.0)
        self.temp_hold_spin.setSuffix(" s")
        grid.addWidget(QLabel("温度波动阈值"), 3, 0)
        grid.addWidget(self.temp_tol_spin, 3, 1)
        grid.addWidget(QLabel("温度稳定时间"), 4, 0)
        grid.addWidget(self.temp_hold_spin, 4, 1)

        self.press_tol_spin = QDoubleSpinBox()
        self.press_tol_spin.setRange(0.0, 100000.0)
        self.press_tol_spin.setDecimals(3)
        self.press_tol_spin.setValue(0.5)
        self.press_tol_spin.setSuffix(" kPa")
        self.press_hold_spin = QDoubleSpinBox()
        self.press_hold_spin.setRange(0.0, 86400.0)
        self.press_hold_spin.setDecimals(1)
        self.press_hold_spin.setValue(300.0)
        self.press_hold_spin.setSuffix(" s")
        grid.addWidget(QLabel("压力波动阈值"), 5, 0)
        grid.addWidget(self.press_tol_spin, 5, 1)
        grid.addWidget(QLabel("压力稳定时间"), 6, 0)
        grid.addWidget(self.press_hold_spin, 6, 1)

        self.current_temp_label = QLabel("-- ℃")
        self.current_press_label = QLabel("-- kPa")
        self.stage_label = QLabel("--")
        grid.addWidget(QLabel("当前温度"), 7, 0)
        grid.addWidget(self.current_temp_label, 7, 1)
        grid.addWidget(QLabel("当前压力"), 8, 0)
        grid.addWidget(self.current_press_label, 8, 1)
        grid.addWidget(QLabel("当前阶段"), 9, 0)
        grid.addWidget(self.stage_label, 9, 1)

        layout.addLayout(grid)

        layout.addWidget(QLabel("温度路径（示例：25-5-80;80-h-80;80-2-25）"))
        self.temp_path_edit = QTextEdit()
        self.temp_path_edit.setFixedHeight(90)
        layout.addWidget(self.temp_path_edit)

        layout.addWidget(QLabel("压力路径（示例：101.3-20-800;800-h-800;800-50-101.3）"))
        self.press_path_edit = QTextEdit()
        self.press_path_edit.setFixedHeight(90)
        layout.addWidget(self.press_path_edit)

        layout.addWidget(QLabel("执行策略：温度路径和压力路径按同序号阶段同步发送；路径较短的通道后续保持当前设定，仅继续记录。"))

        btns = QHBoxLayout()
        self.check_btn = QPushButton("检查路径")
        self.start_btn = QPushButton("开始")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)
        self.check_btn.clicked.connect(self.check_paths)
        self.start_btn.clicked.connect(self.start_control)
        self.stop_btn.clicked.connect(self.stop_control)
        btns.addWidget(self.check_btn)
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        layout.addLayout(btns)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QTextEdit.NoWrap)
        layout.addWidget(self.log_text, 1)

    def load_settings(self):
        self.save_dir_edit.setText(self.settings.value("save_dir", "", type=str))
        self.temp_path_edit.setPlainText(self.settings.value("temp_path", "25-5-80", type=str))
        self.press_path_edit.setPlainText(self.settings.value("press_path", "101.3-20-800", type=str))
        self.poll_spin.setValue(float(self.settings.value("poll_interval_s", 1.0)))
        self.temp_tol_spin.setValue(float(self.settings.value("temp_tol", 0.2)))
        self.temp_hold_spin.setValue(float(self.settings.value("temp_hold_s", 300.0)))
        self.press_tol_spin.setValue(float(self.settings.value("press_tol", 0.5)))
        self.press_hold_spin.setValue(float(self.settings.value("press_hold_s", 300.0)))

    def save_settings(self):
        self.settings.setValue("save_dir", self.save_dir_edit.text().strip())
        self.settings.setValue("temp_path", self.temp_path_edit.toPlainText().strip())
        self.settings.setValue("press_path", self.press_path_edit.toPlainText().strip())
        self.settings.setValue("poll_interval_s", self.poll_spin.value())
        self.settings.setValue("temp_tol", self.temp_tol_spin.value())
        self.settings.setValue("temp_hold_s", self.temp_hold_spin.value())
        self.settings.setValue("press_tol", self.press_tol_spin.value())
        self.settings.setValue("press_hold_s", self.press_hold_spin.value())

    def refresh_ports(self):
        current = self.port_combo.currentText().strip()
        self.port_combo.clear()
        ports = []
        if list_ports is not None:
            ports = sorted(p.device for p in list_ports.comports())
        self.port_combo.addItems(ports)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def choose_save_dir(self):
        folder = QFileDialog.getExistingDirectory(self, "选择保存文件夹", self.save_dir_edit.text().strip())
        if folder:
            self.save_dir_edit.setText(folder)
            self.save_settings()

    def append_log(self, text: str):
        line = f"[{log_stamp()}] {text}"
        self.log_text.append(line)
        if self.event_log_path:
            try:
                with open(self.event_log_path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass

    def build_runtime_stages(self):
        temp_targets = parse_axis_path(self.temp_path_edit.toPlainText(), "温度")
        press_targets = parse_axis_path(self.press_path_edit.toPlainText(), "压力")
        if not temp_targets and not press_targets:
            raise RuntimeError("请至少填写温度路径或压力路径")
        return build_stages(temp_targets, press_targets), temp_targets, press_targets

    def check_paths(self):
        try:
            stages, temp_targets, press_targets = self.build_runtime_stages()
        except Exception as ex:
            QMessageBox.warning(self, "路径错误", str(ex))
            return
        self.append_log(f"[路径] 温度目标数={len(temp_targets)}，压力目标数={len(press_targets)}，执行阶段={len(stages)}")
        QMessageBox.information(self, "路径检查通过", f"温度目标数={len(temp_targets)}\n压力目标数={len(press_targets)}\n执行阶段={len(stages)}")

    def start_control(self):
        try:
            root_dir = self.save_dir_edit.text().strip()
            if not root_dir:
                raise RuntimeError("请先选择保存文件夹")
            port = self.port_combo.currentText().strip()
            if not port:
                raise RuntimeError("请选择标定台 COM 口")
            stages, temp_targets, press_targets = self.build_runtime_stages()
            os.makedirs(root_dir, exist_ok=True)
        except Exception as ex:
            QMessageBox.warning(self, "无法开始", str(ex))
            return

        self.save_settings()
        run_dir = os.path.join(root_dir, f"temp_press_control_{dir_stamp()}")
        os.makedirs(run_dir, exist_ok=True)
        self.event_log_path = os.path.join(run_dir, "运行事件日志.log")
        sensor_log_path = os.path.join(run_dir, "温压状态日志.csv")
        self.append_log(f"[任务] 保存目录: {run_dir}")
        self.append_log(f"[路径] 温度目标数={len(temp_targets)}，压力目标数={len(press_targets)}，执行阶段={len(stages)}")
        self.append_log(
            f"[稳定条件] 温度 ±{self.temp_tol_spin.value():g}℃ 持续 {self.temp_hold_spin.value():g}s；"
            f"压力 ±{self.press_tol_spin.value():g}kPa 持续 {self.press_hold_spin.value():g}s"
        )

        self.worker = ControlThread(
            port=port,
            stages=stages,
            poll_interval_s=float(self.poll_spin.value()),
            temp_tol=float(self.temp_tol_spin.value()),
            temp_hold_s=float(self.temp_hold_spin.value()),
            press_tol=float(self.press_tol_spin.value()),
            press_hold_s=float(self.press_hold_spin.value()),
            sensor_log_path=sensor_log_path,
            parent=self,
        )
        self.worker.message.connect(self.append_log)
        self.worker.state_ready.connect(self.on_state_ready)
        self.worker.failed.connect(self.on_failed)
        self.worker.finished_ok.connect(self.on_finished_ok)
        self.worker.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_control(self):
        if self.worker is not None:
            self.worker.stop()
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.append_log("[任务] 已请求停止")

    def on_state_ready(self, state: dict):
        if state.get("tc") is not None:
            self.current_temp_label.setText(f"{state['tc']:.2f} ℃")
        if state.get("pk") is not None:
            self.current_press_label.setText(f"{state['pk']:.3f} kPa")
        self.stage_label.setText(str(state.get("stage", "--")))

    def on_failed(self, text: str):
        self.append_log("[错误] " + text)
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        QMessageBox.critical(self, "控制失败", text)

    def on_finished_ok(self):
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def closeEvent(self, event):
        self.stop_control()
        if self.worker is not None and self.worker.isRunning():
            self.worker.wait(2500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = ControlWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
