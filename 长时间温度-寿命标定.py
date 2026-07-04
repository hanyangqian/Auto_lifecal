from __future__ import annotations

import csv
import os
import re
import sys
import time
import traceback
from datetime import datetime

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from PyQt5.QtCore import QSettings, QThread, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QDoubleSpinBox,
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

from main_window import (
    SERIAL_LOCK,
    clear_session,
    discover_scope_resource_robust,
    build_tcpip_resource,
    extract_last_value,
    open_scope,
    normalize_acq_mode,
    normalize_acq_state,
    normalize_stopafter,
    read_waveform,
    save_waveform_file,
    _w,
    _q,
)


HOTPLATE_BAUD_CANDIDATES = (9600, 115200)
HOTPLATE_QUERY_WAIT_S = 0.35
PRESS_CONTROL_SLAVE = 2


def timestamp_for_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def timestamp_for_log() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def parse_channels(text: str) -> list[str]:
    channels = []
    for part in re.split(r"[,，\s]+", text.upper().strip()):
        if not part:
            continue
        if not re.fullmatch(r"CH[1-4]", part):
            raise ValueError(f"无效通道: {part}，请使用 CH1~CH4")
        if part not in channels:
            channels.append(part)
    if not channels:
        raise ValueError("至少选择一个示波器通道")
    return channels


def extract_float_list(resp: str) -> list[float]:
    if not resp:
        return []
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)
    return [float(x) for x in nums]


def extract_rt_channels(resp: str, control_slave: int = 1):
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

    tc = pk = tf = tt = status = err = None
    p1 = by_slave.get(control_slave)
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
            tt = float(p1[5])
        except Exception:
            pass
        try:
            status = int(float(p1[8]))
        except Exception:
            pass
        try:
            err = int(float(p1[10].split(",")[-1])) if len(p1) > 10 else None
        except Exception:
            pass

    p2 = by_slave.get(PRESS_CONTROL_SLAVE)
    if p2 is not None:
        try:
            pk = float(p2[2])
        except Exception:
            pass

    return tc, pk, tf, tt, status, err


def parse_idn_parts(idn: str):
    parts = [p.strip() for p in idn.strip().split(",")]
    return {
        "idn": idn.strip(),
        "maker": parts[0] if len(parts) > 0 else "",
        "model": parts[1] if len(parts) > 1 else "",
        "serial": parts[2] if len(parts) > 2 else "",
        "firmware": parts[3] if len(parts) > 3 else "",
    }


class HotplateClient:
    def __init__(self, port: str):
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        self.port = port
        self.ser = None
        self.baud = None

    def connect(self):
        errors = []
        for baud in HOTPLATE_BAUD_CANDIDATES:
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

    def query(self, command: str) -> str:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("标定台串口未连接")
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r").encode("ascii"))
        self.ser.flush()

        deadline = time.time() + HOTPLATE_QUERY_WAIT_S
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
        tc_rt, pk_rt, tf_rt, tt_rt, status, err = extract_rt_channels(rt_resp)

        ct_resp = self.query("TEMPerature:CTEMperature?")
        ct_vals = extract_float_list(ct_resp)
        tc = ct_vals[0] if len(ct_vals) >= 1 else tc_rt
        pk = ct_vals[1] if len(ct_vals) >= 2 else pk_rt

        return {
            "timestamp": timestamp_for_log(),
            "tc": tc,
            "pk": pk,
            "tf": tf_rt,
            "tt": tt_rt,
            "status": status,
            "err": err,
            "baud": self.baud,
            "rt_resp": rt_resp,
            "ct_resp": ct_resp,
        }


class HotplateLoggerThread(QThread):
    state_ready = pyqtSignal(dict)
    message = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, port: str, interval_s: float, log_path: str, parent=None):
        super().__init__(parent)
        self.port = port
        self.interval_s = interval_s
        self.log_path = log_path
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        client = None
        try:
            client = HotplateClient(self.port)
            client.connect()
            self.message.emit(f"[标定台] 已连接 {self.port} @ {client.baud}bps")
            with open(self.log_path, "a", newline="", encoding="utf-8-sig") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "timestamp",
                        "tc_C",
                        "pk_kPa",
                        "tf",
                        "tt",
                        "status",
                        "err",
                        "baud",
                        "rt_resp",
                        "ct_resp",
                    ],
                )
                if f.tell() == 0:
                    writer.writeheader()
                next_tick = time.monotonic()
                while not self._stop:
                    state = client.read_state()
                    writer.writerow(
                        {
                            "timestamp": state["timestamp"],
                            "tc_C": state["tc"],
                            "pk_kPa": state["pk"],
                            "tf": state["tf"],
                            "tt": state["tt"],
                            "status": state["status"],
                            "err": state["err"],
                            "baud": state["baud"],
                            "rt_resp": state["rt_resp"],
                            "ct_resp": state["ct_resp"],
                        }
                    )
                    f.flush()
                    self.state_ready.emit(state)
                    next_tick += self.interval_s
                    while not self._stop and time.monotonic() < next_tick:
                        time.sleep(min(0.05, next_tick - time.monotonic()))
        except Exception:
            self.failed.emit(traceback.format_exc())
        finally:
            if client is not None:
                client.close()
            self.message.emit("[标定台] 日志线程已停止")


class ScopeCaptureThread(QThread):
    saved = pyqtSignal(str)
    message = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(
        self,
        resource: str,
        save_dir: str,
        channels: list[str],
        interval_s: float,
        save_format: str,
        avg_enabled: bool,
        avg_count: int,
        capture_mode: str,
        parent=None,
    ):
        super().__init__(parent)
        self.resource = resource
        self.save_dir = save_dir
        self.channels = channels
        self.interval_s = interval_s
        self.save_format = save_format
        self.avg_enabled = avg_enabled
        self.avg_count = avg_count
        self.capture_mode = capture_mode
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        rm = scope = None
        try:
            rm, scope, idn = open_scope(self.resource)
            clear_session(scope)
            self.message.emit(f"[示波器] 已连接: {idn.strip()}")
            self.apply_acquire_settings(scope)
            idx = 0
            next_tick = time.monotonic()
            while not self._stop:
                while not self._stop and time.monotonic() < next_tick:
                    time.sleep(min(0.05, next_tick - time.monotonic()))
                if self._stop:
                    break

                capture_ts = timestamp_for_log()
                file_ts = timestamp_for_name()
                suffix = "".join(sorted(self.channels))
                filename = f"{file_ts}_T{idx:04d}_{suffix}.{self.save_format}"
                save_path = os.path.join(self.save_dir, filename)

                self.run_capture_once(scope, idx + 1)
                _w(scope, scope.profile.acq_stop_cmd)
                time.sleep(0.25)

                waveform_data = {}
                waveform_meta = {
                    "timestamp": capture_ts,
                    "sync_strategy": "示波器文件名时间戳 + 温压CSV时间戳，按系统时钟对齐",
                    "channels": self.channels,
                    "scope": getattr(scope.profile, "name", ""),
                    "resource": self.resource,
                }
                base_time = None
                for ch in self.channels:
                    t, v, meta = read_waveform(scope, ch)
                    if base_time is None:
                        base_time = t
                        waveform_data["TIME"] = t
                    elif len(t) != len(base_time) or not np.allclose(t, base_time, rtol=0.0, atol=1e-15):
                        raise RuntimeError(f"{ch} 的时间轴与其他通道不一致，已停止保存")
                    waveform_data[ch] = v
                    waveform_meta[ch] = meta

                save_waveform_file(save_path, waveform_data, save_format=self.save_format, meta=waveform_meta)
                self.saved.emit(save_path)
                idx += 1
                next_tick += self.interval_s
        except Exception:
            self.failed.emit(traceback.format_exc())
        finally:
            try:
                if scope is not None:
                    scope.close()
            except Exception:
                pass
            try:
                if rm is not None:
                    rm.close()
            except Exception:
                pass
            self.message.emit("[示波器] 保存线程已停止")

    def apply_acquire_settings(self, scope):
        profile = scope.profile
        _w(scope, profile.stopafter_run_cmd)
        if self.avg_enabled:
            _w(scope, profile.acquire_mode_set_cmd.format(mode="AVERAGE"))
            _w(scope, profile.acquire_numavg_set_cmd.format(numavg=int(self.avg_count)))
            self.message.emit(f"[示波器] {profile.name} Acquire模式: AVERAGE, 平均次数: {int(self.avg_count)}")
        else:
            _w(scope, profile.acquire_mode_set_cmd.format(mode="SAMPLE"))
            self.message.emit(f"[示波器] {profile.name} Acquire模式: SAMPLE")
        self.message.emit(f"[示波器] 采样模式: {self.capture_mode}")

    def query_acquire_status(self, scope) -> dict:
        profile = scope.profile
        return {
            "mode": normalize_acq_mode(_q(scope, profile.acquire_mode_query)),
            "state": normalize_acq_state(_q(scope, profile.acquire_state_query)),
            "numavg": int(float(extract_last_value(_q(scope, profile.acquire_numavg_query)))),
            "numacq": int(float(extract_last_value(_q(scope, profile.acquire_numacq_query)))),
            "stopafter": normalize_stopafter(_q(scope, profile.acquire_stopafter_query)),
            "trig_state": extract_last_value(_q(scope, profile.trigger_state_query)).upper(),
        }

    def wait_scope_run_started(self, scope, cycle_no: int, timeout_s: float = 1.2):
        deadline = time.time() + timeout_s
        last_state = ""
        last_trig = ""
        while time.time() < deadline and not self._stop:
            try:
                acq = self.query_acquire_status(scope)
                last_state = acq.get("state", "")
                last_trig = acq.get("trig_state", "")
                if last_state == "1" and last_trig != "SAVE":
                    time.sleep(0.08)
                    return
            except Exception:
                pass
            time.sleep(0.05)
        self.message.emit(
            f"[示波器][循环 {cycle_no}] RUN确认超时，继续等待采集完成 "
            f"(state={last_state}, trig={last_trig})"
        )

    def wait_scope_average_ready(self, scope, cycle_no: int, timeout_s: float = 30.0):
        if not self.avg_enabled:
            time.sleep(0.2)
            return

        target = int(self.avg_count)
        deadline = time.time() + timeout_s
        last_numacq = -1
        while time.time() < deadline and not self._stop:
            acq = self.query_acquire_status(scope)
            last_numacq = int(acq.get("numacq", 0))
            if last_numacq >= target:
                time.sleep(0.08)
                return
            time.sleep(0.15)
        if self._stop:
            return
        raise RuntimeError(f"第 {cycle_no} 次循环等待平均完成超时: {last_numacq}/{target}")

    def run_capture_once(self, scope, cycle_no: int):
        profile = scope.profile
        _w(scope, profile.stopafter_run_cmd)
        _w(scope, profile.acq_run_cmd)
        self.wait_scope_run_started(scope, cycle_no)

        if self.capture_mode == "CONTINUOUS":
            self.wait_scope_average_ready(scope, cycle_no)
            return

        _w(scope, profile.stopafter_single_cmd)
        _w(scope, profile.acq_run_cmd)
        self.wait_scope_run_started(scope, cycle_no)

        deadline = time.time() + 30.0
        last_state = None
        while time.time() < deadline and not self._stop:
            acq = self.query_acquire_status(scope)
            last_state = acq
            is_running = acq["state"] == "1" and acq["trig_state"] != "SAVE"
            if not is_running:
                return
            time.sleep(0.15)
        if self._stop:
            return
        raise RuntimeError(f"第 {cycle_no} 次循环等待 SINGLE 完成超时: {last_state}")


class LongRunWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("长时间温度-寿命测量")
        self.resize(860, 560)
        self.settings = QSettings("AutoLifeCal", "LongTemperatureLifetime")
        self.scope_resource = ""
        self.scope_connection_mode = "USB"
        self.scope_manual_resource = ""
        self.run_dir = ""
        self.event_log_path = ""
        self.run_started_monotonic = None
        self.scope_thread = None
        self.hotplate_thread = None

        self._build_ui()
        self._load_settings()
        self._refresh_ports()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        grid = QGridLayout()
        row = 0

        self.save_dir_edit = QLineEdit()
        self.save_dir_edit.setReadOnly(True)
        self.save_dir_btn = QPushButton("选择保存文件夹")
        self.save_dir_btn.clicked.connect(self.choose_save_dir)
        grid.addWidget(QLabel("保存文件夹"), row, 0)
        grid.addWidget(self.save_dir_edit, row, 1)
        grid.addWidget(self.save_dir_btn, row, 2)
        row += 1

        self.scope_mode_combo = QComboBox()
        self.scope_mode_combo.addItem("USB", "USB")
        self.scope_mode_combo.addItem("网口/IP", "TCPIP")
        self.scope_mode_combo.addItem("自动", "AUTO")
        self.scope_resource_edit = QLineEdit()
        self.scope_resource_edit.setPlaceholderText("网口模式输入 10.5.5.88 / TCPIP0::...::INSTR")
        self.scope_mode_combo.currentIndexChanged.connect(self.on_scope_mode_changed)
        grid.addWidget(QLabel("示波器连接方式"), row, 0)
        grid.addWidget(self.scope_mode_combo, row, 1)
        row += 1
        grid.addWidget(QLabel("示波器 IP / VISA"), row, 0)
        grid.addWidget(self.scope_resource_edit, row, 1)
        row += 1

        self.scope_label = QLabel("未锁定")
        self.lock_scope_btn = QPushButton("锁定示波器")
        self.lock_scope_btn.clicked.connect(self.lock_scope)
        grid.addWidget(QLabel("示波器"), row, 0)
        grid.addWidget(self.scope_label, row, 1)
        grid.addWidget(self.lock_scope_btn, row, 2)
        row += 1

        self.hotplate_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("刷新COM")
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)
        grid.addWidget(QLabel("标定台COM"), row, 0)
        grid.addWidget(self.hotplate_combo, row, 1)
        grid.addWidget(self.refresh_ports_btn, row, 2)
        row += 1

        self.channels_edit = QLineEdit("CH2")
        self.scope_interval_spin = QDoubleSpinBox()
        self.scope_interval_spin.setRange(1.0, 86400.0)
        self.scope_interval_spin.setDecimals(1)
        self.scope_interval_spin.setValue(60.0)
        self.scope_interval_spin.setSuffix(" s")
        self.hotplate_interval_spin = QDoubleSpinBox()
        self.hotplate_interval_spin.setRange(0.2, 3600.0)
        self.hotplate_interval_spin.setDecimals(1)
        self.hotplate_interval_spin.setValue(1.0)
        self.hotplate_interval_spin.setSuffix(" s")
        grid.addWidget(QLabel("示波器通道"), row, 0)
        grid.addWidget(self.channels_edit, row, 1)
        row += 1
        grid.addWidget(QLabel("示波器保存间隔"), row, 0)
        grid.addWidget(self.scope_interval_spin, row, 1)
        row += 1
        grid.addWidget(QLabel("温压读取间隔"), row, 0)
        grid.addWidget(self.hotplate_interval_spin, row, 1)
        row += 1

        self.format_combo = QComboBox()
        self.format_combo.addItems(["CSV", "NPZ"])
        self.capture_mode_combo = QComboBox()
        self.capture_mode_combo.addItem("连续采样", "CONTINUOUS")
        self.capture_mode_combo.addItem("单次采样", "SINGLE")
        self.avg_checkbox = QCheckBox("启用平均")
        self.avg_count_combo = QComboBox()
        for count in [2, 4, 8, 16, 32, 64, 128, 256, 512]:
            self.avg_count_combo.addItem(str(count), count)
        self.avg_checkbox.toggled.connect(self.avg_count_combo.setEnabled)
        self.avg_count_combo.setEnabled(False)
        self.auto_lock_check = QCheckBox("开始时自动锁定示波器")
        self.auto_lock_check.setChecked(True)
        grid.addWidget(QLabel("保存格式"), row, 0)
        grid.addWidget(self.format_combo, row, 1)
        grid.addWidget(self.auto_lock_check, row, 2)
        row += 1
        grid.addWidget(QLabel("采样模式"), row, 0)
        grid.addWidget(self.capture_mode_combo, row, 1)
        row += 1
        grid.addWidget(self.avg_checkbox, row, 0)
        grid.addWidget(self.avg_count_combo, row, 1)
        row += 1

        self.sync_label = QLabel("同步策略：示波器文件名时间戳 + 温压CSV时间戳，按系统时钟对齐。")
        self.sync_label.setWordWrap(True)
        grid.addWidget(QLabel("同步策略"), row, 0)
        grid.addWidget(self.sync_label, row, 1, 1, 2)
        row += 1

        self.current_temp_label = QLabel("-- ℃")
        self.current_press_label = QLabel("-- kPa")
        self.lifetime_label = QLabel("00:00:00")
        grid.addWidget(QLabel("当前温度"), row, 0)
        grid.addWidget(self.current_temp_label, row, 1)
        row += 1
        grid.addWidget(QLabel("当前压力"), row, 0)
        grid.addWidget(self.current_press_label, row, 1)
        row += 1
        grid.addWidget(QLabel("测量寿命"), row, 0)
        grid.addWidget(self.lifetime_label, row, 1)

        layout.addLayout(grid)

        buttons = QHBoxLayout()
        self.start_btn = QPushButton("开始")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_run)
        self.stop_btn.clicked.connect(self.stop_run)
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.stop_btn)
        layout.addLayout(buttons)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QTextEdit.NoWrap)
        layout.addWidget(self.log_text, 1)

        self.elapsed_timer = self.startTimer(1000)

    def _load_settings(self):
        save_dir = self.settings.value("save_dir", "", type=str)
        if save_dir and os.path.isdir(save_dir):
            self.save_dir_edit.setText(save_dir)
        mode = self.settings.value("scope_connection_mode", "USB", type=str).upper()
        mode_idx = self.scope_mode_combo.findData(mode)
        self.scope_mode_combo.setCurrentIndex(mode_idx if mode_idx >= 0 else 0)
        self.scope_resource_edit.setText(self.settings.value("scope_manual_resource", "", type=str).strip())
        self.channels_edit.setText(self.settings.value("channels", "CH2", type=str))
        self.scope_interval_spin.setValue(float(self.settings.value("scope_interval_s", 60.0)))
        self.hotplate_interval_spin.setValue(float(self.settings.value("hotplate_interval_s", 1.0)))
        fmt = self.settings.value("save_format", "CSV", type=str)
        idx = self.format_combo.findText(fmt)
        if idx >= 0:
            self.format_combo.setCurrentIndex(idx)
        capture_mode = self.settings.value("scope_capture_mode", "CONTINUOUS", type=str)
        capture_idx = self.capture_mode_combo.findData(capture_mode)
        if capture_idx >= 0:
            self.capture_mode_combo.setCurrentIndex(capture_idx)
        avg_enabled = self.settings.value("scope_avg_enabled", False, type=bool)
        avg_count = int(self.settings.value("scope_avg_count", 64))
        avg_idx = self.avg_count_combo.findData(avg_count)
        if avg_idx >= 0:
            self.avg_count_combo.setCurrentIndex(avg_idx)
        self.avg_checkbox.setChecked(avg_enabled)
        self.avg_count_combo.setEnabled(avg_enabled)
        self.on_scope_mode_changed()

    def _save_settings(self):
        self.settings.setValue("save_dir", self.save_dir_edit.text().strip())
        self.settings.setValue("scope_connection_mode", self.scope_mode_combo.currentData() or "USB")
        self.settings.setValue("scope_manual_resource", self.scope_resource_edit.text().strip())
        self.settings.setValue("channels", self.channels_edit.text().strip())
        self.settings.setValue("scope_interval_s", self.scope_interval_spin.value())
        self.settings.setValue("hotplate_interval_s", self.hotplate_interval_spin.value())
        self.settings.setValue("save_format", self.format_combo.currentText())
        self.settings.setValue("scope_capture_mode", self.capture_mode_combo.currentData() or "CONTINUOUS")
        self.settings.setValue("scope_avg_enabled", self.avg_checkbox.isChecked())
        self.settings.setValue("scope_avg_count", int(self.avg_count_combo.currentData() or 64))

    def timerEvent(self, event):
        if event.timerId() != self.elapsed_timer or self.run_started_monotonic is None:
            return
        elapsed = int(time.monotonic() - self.run_started_monotonic)
        h = elapsed // 3600
        m = (elapsed % 3600) // 60
        s = elapsed % 60
        self.lifetime_label.setText(f"{h:02d}:{m:02d}:{s:02d}")

    def append_log(self, text: str):
        line = f"[{timestamp_for_log()}] {text}"
        self.log_text.append(line)
        if self.event_log_path:
            try:
                with open(self.event_log_path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass

    def choose_save_dir(self):
        folder = QFileDialog.getExistingDirectory(self, "选择保存文件夹", self.save_dir_edit.text().strip())
        if folder:
            self.save_dir_edit.setText(folder)
            self._save_settings()

    def _refresh_ports(self):
        current = self.hotplate_combo.currentText().strip()
        self.hotplate_combo.clear()
        ports = []
        if list_ports is not None:
            ports = sorted(p.device for p in list_ports.comports())
        self.hotplate_combo.addItems(ports)
        if current:
            idx = self.hotplate_combo.findText(current)
            if idx >= 0:
                self.hotplate_combo.setCurrentIndex(idx)

    def on_scope_mode_changed(self):
        self.scope_resource_edit.setEnabled(self.scope_mode_combo.currentData() != "USB")
        self.scope_resource = ""
        self.scope_label.setText("未锁定")

    def lock_scope(self):
        try:
            self.scope_connection_mode = self.scope_mode_combo.currentData() or "USB"
            self.scope_manual_resource = self.scope_resource_edit.text().strip()
            if self.scope_connection_mode == "TCPIP" and self.scope_manual_resource:
                resource = build_tcpip_resource(self.scope_manual_resource)
                rm = scope = None
                try:
                    rm, scope, idn = open_scope(resource)
                    info = parse_idn_parts(idn)
                    info["resource"] = resource
                finally:
                    try:
                        if scope is not None:
                            scope.close()
                    except Exception:
                        pass
                    try:
                        if rm is not None:
                            rm.close()
                    except Exception:
                        pass
            else:
                info = discover_scope_resource_robust(
                    SERIAL_LOCK,
                    cached_resource=self.scope_resource,
                    manual_resource=self.scope_manual_resource,
                    connection_mode=self.scope_connection_mode,
                )
            self.scope_resource = info["resource"]
            self.scope_label.setText(f"{info['model']} / {info['serial']} / {self.scope_resource}")
            self.append_log(f"[示波器] 已锁定: 方式={self.scope_connection_mode}, IDN={info['idn']}")
        except Exception as ex:
            self.scope_resource = ""
            self.scope_label.setText("锁定失败")
            QMessageBox.critical(self, "示波器锁定失败", str(ex))

    def _validate_start(self):
        root_dir = self.save_dir_edit.text().strip()
        if not root_dir:
            raise ValueError("请先选择保存文件夹")
        os.makedirs(root_dir, exist_ok=True)
        if self.auto_lock_check.isChecked() and not self.scope_resource:
            self.lock_scope()
        if not self.scope_resource:
            raise ValueError("示波器未锁定")
        hotplate_port = self.hotplate_combo.currentText().strip()
        if not hotplate_port:
            raise ValueError("请选择标定台 COM 口")
        channels = parse_channels(self.channels_edit.text())
        return root_dir, hotplate_port, channels

    def start_run(self):
        try:
            root_dir, hotplate_port, channels = self._validate_start()
        except Exception as ex:
            QMessageBox.warning(self, "无法开始", str(ex))
            return

        self._save_settings()
        self.run_dir = os.path.join(root_dir, f"longrun_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(self.run_dir, exist_ok=True)
        hotplate_log_path = os.path.join(self.run_dir, "温压状态日志.csv")
        self.event_log_path = os.path.join(self.run_dir, "运行事件日志.log")

        self.append_log(f"[任务] 保存目录: {self.run_dir}")
        self.append_log("[任务] 同步策略: 示波器文件名时间戳 + 温压CSV时间戳，按系统时钟对齐")

        self.hotplate_thread = HotplateLoggerThread(
            hotplate_port,
            float(self.hotplate_interval_spin.value()),
            hotplate_log_path,
            self,
        )
        self.hotplate_thread.state_ready.connect(self.on_hotplate_state)
        self.hotplate_thread.message.connect(self.append_log)
        self.hotplate_thread.failed.connect(self.on_thread_failed)

        self.scope_thread = ScopeCaptureThread(
            self.scope_resource,
            self.run_dir,
            channels,
            float(self.scope_interval_spin.value()),
            self.format_combo.currentText(),
            self.avg_checkbox.isChecked(),
            int(self.avg_count_combo.currentData() or 64),
            self.capture_mode_combo.currentData() or "CONTINUOUS",
            self,
        )
        self.scope_thread.saved.connect(lambda path: self.append_log(f"[示波器] 已保存: {path}"))
        self.scope_thread.message.connect(self.append_log)
        self.scope_thread.failed.connect(self.on_thread_failed)

        self.run_started_monotonic = time.monotonic()
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.scope_thread.start()
        self.hotplate_thread.start()

    def stop_run(self):
        if self.scope_thread is not None:
            self.scope_thread.stop()
        if self.hotplate_thread is not None:
            self.hotplate_thread.stop()
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.append_log("[任务] 已请求停止")

    def on_hotplate_state(self, state: dict):
        if state.get("tc") is not None:
            self.current_temp_label.setText(f"{state['tc']:.2f} ℃")
        if state.get("pk") is not None:
            self.current_press_label.setText(f"{state['pk']:.3f} kPa")
        self.append_log(
            f"[温压] TC={state.get('tc')} ℃, PK={state.get('pk')} kPa, "
            f"status={state.get('status')}, err={state.get('err')}"
        )

    def on_thread_failed(self, text: str):
        self.append_log("[错误] " + text)
        self.stop_run()

    def closeEvent(self, event):
        self.stop_run()
        for thread in (self.scope_thread, self.hotplate_thread):
            if thread is not None and thread.isRunning():
                thread.wait(2500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = LongRunWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
