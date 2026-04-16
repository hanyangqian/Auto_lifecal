import os
import re
import sys
import time
import traceback
import threading
from contextlib import contextmanager
from typing import Set

import numpy as np
import pandas as pd
import pyvisa
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None
from PyQt5.QtCore import QSettings, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QDialog,
    QDialogButtonBox,
    QCheckBox,
    QVBoxLayout,
    QPushButton,
    QMessageBox,
    QComboBox,
    QGridLayout,
    QLabel,
)

from laser_lifetime_calibration_ui import LaserLifetimeCalibrationUI


SERIAL_LOCK = "C030795"
DEFAULT_TIMEOUT_MS = 20000
CURVE_TIMEOUT_MS = 3000
DISCOVERY_TIMEOUT_MS = 1500
HOTPLATE_BAUD_CANDIDATES = (9600, 115200)
HOTPLATE_QUERY_WAIT_S = 0.35
TARGET_TOL_C = 0.5
TARGET_HOLD_S = 20.0
MAX_RAMP_RECOVERIES = 2
SERIAL_PROBE_TIMEOUT_S = 0.1
FNAME_RE = re.compile(r"^T(\\d{4})(CH[1-4](?:CH[1-4])*)?\\.CSV$", re.IGNORECASE)


class ChannelDialog(QDialog):
    def __init__(self, selected: Set[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择保存通道（可多选）")
        self.selected = set(selected)

        layout = QVBoxLayout(self)
        self.checks = {}
        for i in range(1, 5):
            name = f"CH{i}"
            cb = QCheckBox(name)
            cb.setChecked(name in self.selected)
            self.checks[name] = cb
            layout.addWidget(cb)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_selected(self) -> Set[str]:
        return {name for name, cb in self.checks.items() if cb.isChecked()}


def _parse_idn(idn: str):
    parts = [p.strip() for p in idn.strip().split(",")]
    maker = parts[0] if len(parts) > 0 else ""
    model = parts[1] if len(parts) > 1 else ""
    serial = parts[2] if len(parts) > 2 else ""
    firmware = parts[3] if len(parts) > 3 else ""
    return maker, model, serial, firmware


def _resource_priority(resource: str):
    upper = resource.upper()
    if upper.startswith("USB"):
        return 0
    if upper.startswith("TCPIP"):
        return 1
    if upper.startswith("GPIB"):
        return 2
    if upper.startswith("ASRL"):
        return 3
    return 4


def _probe_scope_identity(rm, resource: str):
    scope = None
    try:
        scope = rm.open_resource(resource)
        scope.timeout = DISCOVERY_TIMEOUT_MS
        scope.read_termination = "\n"
        scope.write_termination = "\n"
        idn = scope.query("*IDN?").strip()
        maker, model, serial, firmware = _parse_idn(idn)
        return {
            "resource": resource,
            "idn": idn,
            "maker": maker,
            "model": model,
            "serial": serial,
            "firmware": firmware,
        }
    finally:
        if scope is not None:
            try:
                scope.close()
            except Exception:
                pass


def discover_scope_resource_robust(serial_lock: str):
    rm = pyvisa.ResourceManager()
    try:
        resources = list(rm.list_resources())
        if not resources:
            raise RuntimeError("未发现任何 VISA 资源，请确认示波器已连接并安装 NI-VISA。")

        def sort_key(resource: str):
            upper = resource.upper()
            return (
                0 if serial_lock and serial_lock.upper() in upper else 1,
                0 if "0X0699" in upper or "TEK" in upper else 1,
                _resource_priority(resource),
                resource,
            )

        matches = []
        probe_errors = []
        for resource in sorted(resources, key=sort_key):
            upper = resource.upper()
            if "::INSTR" not in upper:
                continue
            try:
                info = _probe_scope_identity(rm, resource)
            except Exception as ex:
                probe_errors.append(f"{resource} -> {ex}")
                continue

            maker_upper = info["maker"].upper()
            idn_upper = info["idn"].upper()
            serial_upper = info["serial"].upper()
            is_tek = "TEKTRONIX" in maker_upper or "TEKTRONIX" in idn_upper
            if not is_tek:
                continue

            matches.append(info)
            if serial_lock and serial_upper == serial_lock.upper():
                return info

        if len(matches) == 1:
            return matches[0]

        if len(matches) > 1:
            options = "\n".join(f"{item['resource']} -> {item['idn']}" for item in matches)
            raise RuntimeError(
                f"发现多个 Tek 设备，但没有找到序列号 {serial_lock} 的目标设备：\n{options}"
            )

        resource_list = "\n".join(resources)
        error_text = "\n".join(probe_errors[:8]) if probe_errors else "(无可用探测结果)"
        raise RuntimeError(
            f"未找到可用 Tek 示波器。当前 VISA 资源：\n{resource_list}\n\n探测结果：\n{error_text}"
        )
    finally:
        rm.close()


def open_scope(resource_str: str):
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(resource_str)
    scope.timeout = DEFAULT_TIMEOUT_MS
    scope.read_termination = "\n"
    scope.write_termination = "\n"
    idn = scope.query("*IDN?")
    return rm, scope, idn


def clear_session(scope):
    try:
        scope.clear()
    except Exception:
        pass
    try:
        scope.write("*CLS")
    except Exception:
        pass


def drain_input_buffer(scope, timeout_ms=100):
    old_timeout = scope.timeout
    old_rt = scope.read_termination
    scope.timeout = timeout_ms
    scope.read_termination = None
    try:
        while True:
            scope.read_raw()
    except Exception:
        pass
    finally:
        scope.timeout = old_timeout
        scope.read_termination = old_rt

    try:
        scope.write("*CLS")
    except Exception:
        pass


def _q(scope, cmd: str, retry=3, sleep_s=0.12) -> str:
    for i in range(retry):
        try:
            return scope.query(cmd)
        except pyvisa.errors.VisaIOError:
            if i == retry - 1:
                raise
            time.sleep(sleep_s)
    raise RuntimeError("unreachable")


def _w(scope, cmd: str, retry=3, sleep_s=0.12):
    for i in range(retry):
        try:
            scope.write(cmd)
            return
        except pyvisa.errors.VisaIOError:
            if i == retry - 1:
                raise
            time.sleep(sleep_s)


def _get_record_length(scope) -> int:
    try:
        return int(float(_q(scope, "WFMOUTPRE:RECORDLENGTH?")))
    except Exception:
        return int(float(_q(scope, "HORIZONTAL:RECORDLENGTH?")))


@contextmanager
def temporary_timeout(scope, timeout_ms: int):
    old_timeout = scope.timeout
    scope.timeout = timeout_ms
    try:
        yield
    finally:
        scope.timeout = old_timeout


@contextmanager
def binary_transfer_mode(scope, chunk_size=1024 * 1024):
    old_rt = scope.read_termination
    old_chunk = getattr(scope, "chunk_size", None)
    scope.read_termination = None
    if old_chunk is not None:
        scope.chunk_size = chunk_size
    try:
        yield
    finally:
        scope.read_termination = old_rt
        if old_chunk is not None:
            scope.chunk_size = old_chunk


def _select_composition_and_width(scope):
    comp = _q(scope, "DATA:COMPOSITION:AVAILABLE?").strip().upper()

    if "SINGULAR_YT" in comp:
        _w(scope, "DATA:COMPOSITION SINGULAR_YT")
        _w(scope, "DATA:WIDTH 2")
        return 2, "H", "SINGULAR_YT"

    _w(scope, "DATA:COMPOSITION COMPOSITE_YT")
    _w(scope, "DATA:WIDTH 1")
    return 1, "B", "COMPOSITE_YT"


def read_waveform(scope, ch_name: str):
    _w(scope, f"DATA:SOURCE {ch_name}")
    time.sleep(0.12)

    _w(scope, "DATA:ENC RPB")
    try:
        _w(scope, "DATA:RESOLUTION FULL")
    except Exception:
        pass

    width, dtype, comp_used = _select_composition_and_width(scope)

    preamble = "WFMOUTPRE"
    ymult = float(_q(scope, f"{preamble}:YMULT?"))
    yzero = float(_q(scope, f"{preamble}:YZERO?"))
    yoff = float(_q(scope, f"{preamble}:YOFF?"))
    xincr = float(_q(scope, f"{preamble}:XINCR?"))
    xzero = float(_q(scope, f"{preamble}:XZERO?"))

    data_enc = _q(scope, "DATA:ENC?").strip()
    byt_nr = _q(scope, f"{preamble}:BYT_NR?").strip()
    bn_fmt = _q(scope, f"{preamble}:BN_FMT?").strip()
    byt_or = _q(scope, f"{preamble}:BYT_OR?").strip().upper()

    recl = _get_record_length(scope)
    _w(scope, "DATA:START 1")
    _w(scope, f"DATA:STOP {recl}")

    is_big_endian = width == 2 and "MSB" in byt_or
    with temporary_timeout(scope, CURVE_TIMEOUT_MS):
        with binary_transfer_mode(scope):
            raw = scope.query_binary_values(
                "CURVE?",
                datatype=dtype,
                container=np.array,
                is_big_endian=is_big_endian,
                expect_termination=False,
                chunk_size=1024 * 1024,
            )
    drain_input_buffer(scope)

    if len(raw) != recl:
        recl = len(raw)

    v = (raw - yoff) * ymult + yzero
    t = xzero + np.arange(recl) * xincr

    return t, v, {
        "width": width,
        "dtype": dtype,
        "composition": comp_used,
        "recl": recl,
        "data_enc": data_enc,
        "byt_nr": byt_nr,
        "bn_fmt": bn_fmt,
        "byt_or": byt_or,
        "ymult": ymult,
        "yoff": yoff,
        "yzero": yzero,
        "xincr": xincr,
        "xzero": xzero,
    }


def next_index(save_dir: str) -> int:
    max_idx = -1
    if not os.path.isdir(save_dir):
        return 0
    for fn in os.listdir(save_dir):
        m = FNAME_RE.match(fn)
        if not m:
            continue
        idx = int(m.group(1))
        max_idx = max(max_idx, idx)
    return max_idx + 1


def make_filename(idx: int, channels):
    suffix = "".join(sorted(channels))
    return f"T{idx:04d}{suffix}.CSV"


def save_csv(save_path: str, data: dict):
    pd.DataFrame(data).to_csv(save_path, index=False)


class MainWindow(LaserLifetimeCalibrationUI):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("AutoLifeCal", "ScopeSaver")
        self.selected_channels = {"CH1"}
        self.resource_str = None
        self.idn = None
        self.allowed_avg_counts = [2, 4, 8, 16, 32, 64, 128, 256, 512]
        self.control_slave = 1
        self.ramp_target_temp = None
        self.ramp_rate_cpm = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        self.last_hotplate_s_status = None
        self.ramp_recovery_count = 0
        self.hotplate_guard_timer = QTimer(self)
        self.hotplate_guard_timer.setInterval(1200)
        self.hotplate_guard_timer.timeout.connect(self.check_hotplate_ramp_guard)
        self.detect_thread = None

        self.channel_btn = QPushButton()
        self._update_channel_button_text()
        self.channel_btn.clicked.connect(self.choose_channels)

        parent_layout = self.channel_combo.parentWidget().layout()
        parent_layout.replaceWidget(self.channel_combo, self.channel_btn)
        self.channel_combo.deleteLater()

        self.avg_count_combo = QComboBox()
        for count in self.allowed_avg_counts:
            self.avg_count_combo.addItem(str(count), count)
        default_avg = int(self.avg_times_spin.value())
        if default_avg not in self.allowed_avg_counts:
            default_avg = 64
        idx = self.avg_count_combo.findData(default_avg)
        if idx >= 0:
            self.avg_count_combo.setCurrentIndex(idx)
        avg_layout = self.avg_times_spin.parentWidget().layout()
        avg_layout.replaceWidget(self.avg_times_spin, self.avg_count_combo)
        self.avg_times_spin.deleteLater()
        self.avg_count_combo.setStyleSheet(
            "QComboBox:disabled {"
            "background: #e9edf2;"
            "color: #8f99a5;"
            "border: 1px dashed #bcc6d1;"
            "}"
        )

        self.browse_btn.clicked.connect(self.choose_path)
        self.init_btn.clicked.connect(self.initialize_system)
        self.start_btn.clicked.connect(self.do_save)
        self.avg_checkbox.toggled.connect(self.on_avg_toggled)
        self.hotplate_hold_btn.clicked.connect(self.send_hotplate_hold)
        self.hotplate_ramp_btn.clicked.connect(self.send_hotplate_ramp)
        self.hotplate_stop_btn.clicked.connect(self.send_hotplate_stop)

        self.on_avg_toggled(self.avg_checkbox.isChecked())

        self.load_saved_path()
        self.settings_button.hide()
        self.append_status("[系统] 已加载主窗口")

    def append_status(self, text: str):
        self._append_log(self.status_log_text, text)

    def append_data(self, text: str):
        self._append_log(self.sensor_log_text, text)

    def _append_log(self, widget, text: str, max_lines: int = 500):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {text}"
        widget.append(line)
        doc = widget.document()
        while doc.blockCount() > max_lines:
            cursor = widget.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _update_channel_button_text(self):
        self.channel_btn.setText("、".join(sorted(self.selected_channels)))

    def load_saved_path(self):
        saved = self.settings.value("save_dir", "", type=str)
        if saved and os.path.isdir(saved):
            self.save_path_edit.setText(saved)

    def persist_save_path(self):
        save_dir = self.save_path_edit.text().strip()
        if save_dir and os.path.isdir(save_dir):
            self.settings.setValue("save_dir", save_dir)
        else:
            self.settings.remove("save_dir")

    def initialize_system(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("初始化设置")
        dialog.setMinimumWidth(420)

        layout = QVBoxLayout(dialog)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)

        grid.addWidget(QLabel("示波器 COM口:"), 0, 0)
        scope_combo = QComboBox()
        grid.addWidget(scope_combo, 0, 1)

        grid.addWidget(QLabel("加热台 COM口:"), 1, 0)
        hotplate_combo = QComboBox()
        grid.addWidget(hotplate_combo, 1, 1)

        status_label = QLabel("可手动选择，也可自动检测(100ms/端口询问)")
        grid.addWidget(status_label, 2, 0, 1, 2)
        layout.addLayout(grid)

        tools_layout = QGridLayout()
        refresh_btn = QPushButton("刷新端口")
        auto_btn = QPushButton("自动检测(100ms)")
        tools_layout.addWidget(refresh_btn, 0, 0)
        tools_layout.addWidget(auto_btn, 0, 1)
        layout.addLayout(tools_layout)

        def fill_port_combos():
            scope_combo.clear()
            hotplate_combo.clear()
            for label, value in self.get_available_com_ports():
                scope_combo.addItem(label, value)
                hotplate_combo.addItem(label, value)

            def ensure_current(combo, current_value):
                idx = combo.findData(current_value)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                elif current_value:
                    combo.addItem(f"{current_value} (当前)", current_value)
                    combo.setCurrentIndex(combo.count() - 1)

            ensure_current(scope_combo, self.scope_com_port)
            ensure_current(hotplate_combo, self.hotplate_com_port)

        def on_detect_done(result):
            auto_btn.setEnabled(True)
            scope_val = result.get("scope_com")
            hotplate_val = result.get("hotplate_com")
            if scope_val:
                idx = scope_combo.findData(scope_val)
                if idx < 0:
                    scope_combo.addItem(f"{scope_val} (自动检测)", scope_val)
                    idx = scope_combo.count() - 1
                scope_combo.setCurrentIndex(idx)
            if hotplate_val:
                idx = hotplate_combo.findData(hotplate_val)
                if idx < 0:
                    hotplate_combo.addItem(f"{hotplate_val} (自动检测)", hotplate_val)
                    idx = hotplate_combo.count() - 1
                hotplate_combo.setCurrentIndex(idx)
            status_label.setText(result.get("message", "自动检测完成"))

        def start_auto_detect():
            if self.detect_thread and self.detect_thread.is_alive():
                return
            if serial is None:
                QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先安装: pip install pyserial")
                return
            auto_btn.setEnabled(False)
            status_label.setText("正在自动检测串口设备...")

            def worker():
                result = self._scan_serial_devices()
                QTimer.singleShot(0, lambda r=result: on_detect_done(r))

            self.detect_thread = threading.Thread(target=worker, daemon=True)
            self.detect_thread.start()

        fill_port_combos()
        refresh_btn.clicked.connect(fill_port_combos)
        auto_btn.clicked.connect(start_auto_detect)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec_() != QDialog.Accepted:
            return

        self.scope_com_port = scope_combo.currentData() or self.scope_com_port
        self.hotplate_com_port = hotplate_combo.currentData() or self.hotplate_com_port
        self.update_settings_tooltip()
        self.append_status(f"[初始化] 示波器COM: {self.scope_com_port}, 加热台COM: {self.hotplate_com_port}")
        self.try_lock_scope()

    def _probe_port_once(self, port, baud, cmd):
        ser = None
        try:
            ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=SERIAL_PROBE_TIMEOUT_S,
                write_timeout=SERIAL_PROBE_TIMEOUT_S,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            ser.reset_input_buffer()
            ser.write((cmd + "\r").encode("ascii"))
            ser.flush()
            deadline = time.time() + SERIAL_PROBE_TIMEOUT_S
            chunks = []
            while time.time() < deadline:
                n = ser.in_waiting
                if n > 0:
                    chunks.append(ser.read(n))
                    break
                time.sleep(0.005)
            return b"".join(chunks).decode("ascii", errors="ignore").strip("\x00\r\n ")
        except Exception:
            return ""
        finally:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

    def _scan_serial_devices(self):
        if serial is None:
            return {"scope_com": None, "hotplate_com": None, "message": "未安装pyserial"}
        try:
            ports = [p.device for p in list_ports.comports()] if list_ports else []
        except Exception as ex:
            return {"scope_com": None, "hotplate_com": None, "message": f"串口枚举失败: {ex}"}

        scope_com = None
        hotplate_com = None
        for port in ports:
            for baud in HOTPLATE_BAUD_CANDIDATES:
                idn = self._probe_port_once(port, baud, "*IDN?")
                idn_l = idn.lower()
                if ("tektronix" in idn_l or SERIAL_LOCK.lower() in idn_l) and scope_com is None:
                    scope_com = port
                if "instec,mk2000b" in idn_l and hotplate_com is None:
                    hotplate_com = port
                if hotplate_com is None:
                    rt = self._probe_port_once(port, baud, "TEMPerature:RTINformation?")
                    if "MK:" in rt.upper():
                        hotplate_com = port
                if scope_com and hotplate_com:
                    break
            if scope_com and hotplate_com:
                break

        msg = f"自动检测完成: 示波器={scope_com or '未找到'}, 加热台={hotplate_com or '未找到'}"
        return {"scope_com": scope_com, "hotplate_com": hotplate_com, "message": msg}

    def choose_path(self):
        d = QFileDialog.getExistingDirectory(self, "选择保存路径", self.save_path_edit.text().strip())
        if d:
            self.save_path_edit.setText(d)
            self.persist_save_path()
            self.append_status(f"[示波器] 保存路径: {d}")

    def choose_channels(self):
        dlg = ChannelDialog(self.selected_channels, self)
        if dlg.exec_() == QDialog.Accepted:
            sel = dlg.get_selected()
            if not sel:
                QMessageBox.warning(self, "提示", "至少选择一个通道")
                return
            self.selected_channels = sel
            self._update_channel_button_text()
            self.append_status(f"[示波器] 保存通道: {','.join(sorted(sel))}")

    def on_avg_toggled(self, checked: bool):
        self.avg_count_combo.setEnabled(checked)
        if checked:
            self.avg_count_combo.setToolTip("")
            self.avg_count_label.setText("平均次数:")
            self.avg_count_label.setStyleSheet("")
        else:
            self.avg_count_combo.setToolTip("仅在启用平均时生效")
            self.avg_count_label.setText("平均次数(已禁用):")
            self.avg_count_label.setStyleSheet("color: #8f99a5;")

    def get_avg_count(self) -> int:
        value = self.avg_count_combo.currentData()
        if value is None:
            return 64
        return int(value)

    def get_capture_mode(self) -> str:
        if self.single_mode_btn.isChecked():
            return "SINGLE"
        return "CONTINUOUS"

    def _extract_value(self, resp: str) -> str:
        text = resp.strip()
        if " " in text:
            return text.split()[-1]
        return text

    def query_acquire_status(self, scope) -> dict:
        mode = self._extract_value(_q(scope, "ACQ:MODE?")).upper()
        state = self._extract_value(_q(scope, "ACQ:STATE?"))
        numavg = int(float(self._extract_value(_q(scope, "ACQ:NUMAVG?"))))
        numacq = int(float(self._extract_value(_q(scope, "ACQ:NUMACQ?"))))
        stopafter = self._extract_value(_q(scope, "ACQ:STOPAFTER?")).upper()
        trig_state = self._extract_value(_q(scope, "TRIG:STATE?")).upper()
        return {
            "mode": mode,
            "state": state,
            "numavg": numavg,
            "numacq": numacq,
            "stopafter": stopafter,
            "trig_state": trig_state,
        }

    def apply_acquire_settings(self, scope):
        avg_enabled = self.avg_checkbox.isChecked()
        numavg = self.get_avg_count()
        capture_mode = self.get_capture_mode()

        _w(scope, "ACQ:STOPAFTER RUNST")
        if avg_enabled:
            _w(scope, "ACQ:MODE AVERAGE")
            _w(scope, f"ACQ:NUMAVG {numavg}")
            self.append_status(f"[示波器] Acquire模式: AVERAGE, 平均次数: {numavg}")
        else:
            _w(scope, "ACQ:MODE SAMPLE")
            self.append_status("[示波器] Acquire模式: SAMPLE")
        self.append_status(f"[示波器] 采样模式: {capture_mode}")

    def run_capture_once(self, scope, cycle_idx: int, total_cycles: int):
        capture_mode = self.get_capture_mode()
        _w(scope, "ACQ:STOPAFTER RUNST")
        _w(scope, "ACQ:STATE RUN")
        time.sleep(0.15)

        if capture_mode == "CONTINUOUS":
            self.append_status(f"[循环 {cycle_idx}/{total_cycles}] 连续采样运行")
            return

        _w(scope, "ACQ:STOPAFTER SEQ")
        _w(scope, "ACQ:STATE RUN")

        deadline = time.time() + 30.0
        while time.time() < deadline:
            acq = self.query_acquire_status(scope)
            is_running = acq["state"] == "1" and acq["trig_state"] != "SAVE"
            if not is_running:
                self.append_status(f"[循环 {cycle_idx}/{total_cycles}] SINGLE完成, 触发状态={acq['trig_state']}")
                return
            time.sleep(0.15)
        raise RuntimeError(f"第 {cycle_idx} 次循环等待 SINGLE 完成超时")

    def with_scope(self):
        if not self.resource_str:
            raise RuntimeError("示波器未锁定")
        rm, scope, _ = open_scope(self.resource_str)
        return rm, scope

    def try_lock_scope(self):
        try:
            device_info = discover_scope_resource_robust(SERIAL_LOCK)
            self.resource_str = device_info["resource"]
            self.idn = device_info["idn"].strip()
            self.append_status(f"[示波器] 已锁定 [{SERIAL_LOCK}]")
            self.append_status(f"[示波器] 资源: {self.resource_str}")
            self.append_status(f"[示波器] IDN: {self.idn}")
        except Exception as e:
            self.resource_str = None
            self.idn = None
            QMessageBox.critical(self, "连接失败", f"无法锁定示波器（序列号 {SERIAL_LOCK}）\n\n{e}")

    def do_save(self):
        save_dir = self.save_path_edit.text().strip()
        if save_dir and not os.path.isdir(save_dir):
            try:
                os.makedirs(save_dir, exist_ok=True)
            except Exception:
                QMessageBox.warning(self, "提示", "保存路径无效，请重新选择")
                return

        if not save_dir:
            QMessageBox.warning(self, "提示", "请先选择保存路径")
            return
        if not self.resource_str:
            QMessageBox.warning(self, "提示", "示波器未锁定，无法保存")
            return

        try:
            self.wait_for_temp_press_stable()
        except Exception as ex:
            QMessageBox.critical(self, "稳定等待失败", str(ex))
            return

        rm = scope = None
        try:
            rm, scope, _ = open_scope(self.resource_str)
            clear_session(scope)
            self.apply_acquire_settings(scope)

            self.persist_save_path()
            start_idx = next_index(save_dir)
            cycles = int(self.cycles_spin.value())
            if cycles < 1:
                raise RuntimeError("循环次数必须大于等于 1")

            capture_mode = self.get_capture_mode()
            if capture_mode == "SINGLE":
                self.append_status(f"[任务] 开始执行 RUN-SINGLE-SAVE，共 {cycles} 次")
            else:
                self.append_status(f"[任务] 开始执行 RUN-SAVE，共 {cycles} 次")
            saved_files = []

            for n in range(cycles):
                cycle_no = n + 1
                if capture_mode == "SINGLE":
                    self.append_status(f"[循环 {cycle_no}/{cycles}] RUN -> SINGLE")
                else:
                    self.append_status(f"[循环 {cycle_no}/{cycles}] RUN")
                self.run_capture_once(scope, cycle_no, cycles)
                _w(scope, "ACQ:STATE STOP")
                time.sleep(0.25)

                csv_data = {}
                base_time = None
                idx = start_idx + n
                csv_path = os.path.join(save_dir, make_filename(idx, self.selected_channels))

                for ch in sorted(self.selected_channels):
                    t, v, meta = read_waveform(scope, ch)
                    if base_time is None:
                        base_time = t
                        csv_data["TIME"] = t
                    else:
                        if len(t) != len(base_time) or not np.allclose(t, base_time, rtol=0.0, atol=1e-15):
                            raise RuntimeError(f"{ch} 的时间轴与其他通道不一致，已停止合并保存")

                    csv_data[ch] = v
                    self.append_status(
                        f"[循环{cycle_no}][{ch}] {meta['composition']} width={meta['width']} dtype={meta['dtype']} "
                        f"BYT_OR={meta['byt_or']} BN_FMT={meta['bn_fmt']} ENC={meta['data_enc']}"
                    )

                save_csv(csv_path, csv_data)
                saved_files.append(csv_path)
                self.append_status(f"[循环 {cycle_no}/{cycles}] SAVE完成: {csv_path}")

            summary = "\n".join(saved_files[:6])
            if len(saved_files) > 6:
                summary += f"\n... 共 {len(saved_files)} 个文件"
            if capture_mode == "SINGLE":
                QMessageBox.information(self, "保存完成", f"已完成 {cycles} 次 RUN-SINGLE-SAVE\n{summary}")
            else:
                QMessageBox.information(self, "保存完成", f"已完成 {cycles} 次 RUN-SAVE\n{summary}")

        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"{e}\n\n{traceback.format_exc()}")
        finally:
            try:
                if scope:
                    scope.close()
            except Exception:
                pass
            try:
                if rm:
                    rm.close()
            except Exception:
                pass

    def _send_hotplate_commands(self, commands):
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.hotplate_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置加热台 COM 口，请先在 menu-settings 中设置")

        errors = []
        for baud in HOTPLATE_BAUD_CANDIDATES:
            ser = None
            try:
                ser = serial.Serial(
                    port,
                    baudrate=baud,
                    timeout=0.25,
                    write_timeout=0.5,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.08)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                for cmd in commands:
                    ser.write((cmd + "\r").encode("ascii"))
                    ser.flush()
                    time.sleep(0.05)
                return baud
            except Exception as ex:
                errors.append(f"{baud}bps -> {ex}")
            finally:
                try:
                    if ser and ser.is_open:
                        ser.close()
                except Exception:
                    pass
        raise RuntimeError(f"加热台命令发送失败: {port}\n" + "\n".join(errors))

    def _query_hotplate(self, command: str):
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.hotplate_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置加热台 COM 口，请先在 menu-settings 中设置")

        errors = []
        for baud in HOTPLATE_BAUD_CANDIDATES:
            ser = None
            try:
                ser = serial.Serial(
                    port,
                    baudrate=baud,
                    timeout=0.12,
                    write_timeout=0.4,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.06)
                ser.reset_input_buffer()
                ser.write((command + "\r").encode("ascii"))
                ser.flush()

                deadline = time.time() + HOTPLATE_QUERY_WAIT_S
                chunks = []
                last_recv = 0.0
                while time.time() < deadline:
                    n = ser.in_waiting
                    if n > 0:
                        chunks.append(ser.read(n))
                        last_recv = time.time()
                    if chunks and (time.time() - last_recv) > 0.08:
                        break
                    time.sleep(0.01)

                resp = b"".join(chunks).decode("ascii", errors="ignore").strip("\x00\r\n ")
                return resp, baud
            except Exception as ex:
                errors.append(f"{baud}bps -> {ex}")
            finally:
                try:
                    if ser and ser.is_open:
                        ser.close()
                except Exception:
                    pass
        raise RuntimeError(f"加热台查询失败: {port}\n" + "\n".join(errors))

    @staticmethod
    def _extract_rt_channels(resp: str, control_slave: int = 1):
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

        tc1 = tf1 = tt1 = pk2 = None
        s_status = err_code = None

        p1 = by_slave.get(control_slave)
        if p1 is not None:
            try:
                tc1 = float(p1[2])
            except Exception:
                pass
            try:
                tf1 = float(p1[4])
            except Exception:
                pass
            try:
                tt1 = float(p1[5])
            except Exception:
                pass
            try:
                s_status = int(float(p1[8]))
            except Exception:
                pass
            try:
                err_code = int(float(p1[10].split(",")[-1])) if len(p1) > 10 else None
            except Exception:
                pass

        p2 = by_slave.get(2)
        if p2 is not None:
            try:
                pk2 = float(p2[2])
            except Exception:
                pass

        return tc1, pk2, tf1, tt1, s_status, err_code

    @staticmethod
    def _extract_temp_pressure_fallback(resp: str):
        if not resp:
            return None, None
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)
        if len(nums) >= 2:
            try:
                return float(nums[0]), float(nums[1])
            except Exception:
                return None, None
        if len(nums) == 1:
            try:
                return float(nums[0]), None
            except Exception:
                return None, None
        return None, None

    def wait_for_temp_press_stable(self):
        need_temp = self.temp_cal_btn.isChecked()
        need_press = self.press_cal_btn.isChecked()
        if not need_temp and not need_press:
            return

        temp_target = float(self.ref_temp_spin.value())
        press_target = float(self.ref_press_spin.value())
        temp_tol = float(self.temp_fluc_spin.value())
        press_tol = float(self.press_fluc_spin.value())
        temp_hold_s = int(self.temp_stable_time_spin.value())
        press_hold_s = int(self.press_stable_time_spin.value())

        temp_since = None
        press_since = None
        last_log_ts = 0.0

        self.append_status(
            f"[稳定等待] 启动: 温度目标={temp_target:.2f}±{temp_tol:.2f}, 压力目标={press_target:.2f}±{press_tol:.2f}"
        )

        while True:
            rt_resp, _ = self._query_hotplate("TEMPerature:RTINformation?")
            tc, pk, _, _, _, _ = self._extract_rt_channels(rt_resp, self.control_slave)
            if tc is None or pk is None:
                ftc, fpk = self._extract_temp_pressure_fallback(rt_resp)
                if tc is None:
                    tc = ftc
                if pk is None:
                    pk = fpk

            now = time.time()
            if need_temp and tc is not None and abs(tc - temp_target) <= temp_tol:
                if temp_since is None:
                    temp_since = now
            elif need_temp:
                temp_since = None

            if need_press and pk is not None and abs(pk - press_target) <= press_tol:
                if press_since is None:
                    press_since = now
            elif need_press:
                press_since = None

            temp_ok = (not need_temp) or (temp_hold_s <= 0) or (temp_since is not None and (now - temp_since) >= temp_hold_s)
            press_ok = (not need_press) or (press_hold_s <= 0) or (press_since is not None and (now - press_since) >= press_hold_s)

            if need_temp:
                self.current_temp_display.setText("稳定" if temp_ok else "待稳")
            else:
                self.current_temp_display.setText("已关闭")
            if need_press:
                self.current_press_display.setText("稳定" if press_ok else "待稳")
            else:
                self.current_press_display.setText("已关闭")

            if now - last_log_ts >= 1.0:
                temp_prog = 0.0 if temp_since is None else max(0.0, now - temp_since)
                press_prog = 0.0 if press_since is None else max(0.0, now - press_since)
                self.append_data(
                    f"温度={tc if tc is not None else 'NA'} °C, 压力={pk if pk is not None else 'NA'} kPa, "
                    f"温稳进度={temp_prog:.1f}/{temp_hold_s}s, 压稳进度={press_prog:.1f}/{press_hold_s}s"
                )
                last_log_ts = now

            if temp_ok and press_ok:
                self.append_status("[稳定等待] 温度、压力均已满足稳定时间，开始示波器保存流程")
                return

            QApplication.processEvents()
            time.sleep(0.2)

    def _reset_ramp_guard_state(self):
        self.ramp_target_temp = None
        self.ramp_rate_cpm = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        self.last_hotplate_s_status = None
        self.ramp_recovery_count = 0
        self.hotplate_guard_timer.stop()

    def _update_reach_judge(self, tc_now, s_status):
        if self.ramp_target_temp is None or tc_now is None:
            return

        target = self.ramp_target_temp
        diff = abs(tc_now - target)
        now = time.time()

        if s_status == 2:
            if diff <= TARGET_TOL_C:
                if self.reach_inband_since is None:
                    self.reach_inband_since = now
                inband_sec = now - self.reach_inband_since
            else:
                self.reach_inband_since = None
                inband_sec = 0.0

            if (not self.reach_confirmed) and inband_sec >= TARGET_HOLD_S:
                self.reach_confirmed = True
                self.append_status(
                    f"[加热台] RAMP达标: |TC-TF| <= {TARGET_TOL_C:.1f}℃ 持续 {TARGET_HOLD_S:.0f}s"
                )

    def check_hotplate_ramp_guard(self):
        if self.ramp_target_temp is None:
            self.hotplate_guard_timer.stop()
            return
        try:
            rt_resp, baud = self._query_hotplate("TEMPerature:RTINformation?")
            tc, _, _, _, s_status, err_code = self._extract_rt_channels(rt_resp, self.control_slave)
            self._update_reach_judge(tc, s_status)

            if (
                self.last_hotplate_s_status == 2
                and s_status == 0
                and not self.reach_confirmed
            ):
                if self.ramp_recovery_count < MAX_RAMP_RECOVERIES and self.ramp_rate_cpm is not None:
                    self.ramp_recovery_count += 1
                    self._send_hotplate_commands(
                        [
                            f"TEMPerature:OPSLave {self.control_slave}",
                            f"TEMPerature:RAMP {self.ramp_target_temp},{self.ramp_rate_cpm}",
                        ]
                    )
                    self.append_status(
                        f"[加热台] 检测到RAMP中断(未达标), 自动重发RAMP "
                        f"({self.ramp_recovery_count}/{MAX_RAMP_RECOVERIES}) [{baud}bps]"
                    )
                else:
                    self.append_status(
                        f"[加热台] 警告: RAMP中断且自动恢复已达上限, err={err_code}"
                    )
            self.last_hotplate_s_status = s_status

            if self.reach_confirmed and s_status in (0, 1):
                self.append_status("[加热台] 达标后已进入稳定/停止状态，停止RAMP监控")
                self.hotplate_guard_timer.stop()

        except Exception as ex:
            self.append_status(f"[加热台] 监控查询失败: {ex}")

    def send_hotplate_hold(self):
        tf = float(self.hotplate_tf_spin.value())
        try:
            baud = self._send_hotplate_commands([f"TEMPerature:HOLD {tf}"])
            self._reset_ramp_guard_state()
            self.append_status(f"[加热台] HOLD 已发送: TF={tf:.2f} ℃ ({self.hotplate_com_port}, {baud}bps)")
        except Exception as ex:
            QMessageBox.critical(self, "加热台控制失败", str(ex))

    def send_hotplate_ramp(self):
        tf = float(self.hotplate_tf_spin.value())
        rate = float(self.hotplate_rate_spin.value())
        if rate <= 0:
            QMessageBox.warning(self, "加热台参数错误", "Rate 必须大于 0")
            return
        try:
            baud = self._send_hotplate_commands(
                [
                    "TEMPerature:OPSLave 1",
                    f"TEMPerature:RAMP {tf},{rate}",
                ]
            )
            self.ramp_target_temp = tf
            self.ramp_rate_cpm = rate
            self.reach_inband_since = None
            self.reach_confirmed = False
            self.last_hotplate_s_status = None
            self.ramp_recovery_count = 0
            self.hotplate_guard_timer.start()
            self.append_status(
                f"[加热台] RAMP 已发送: TF={tf:.2f} ℃, Rate={rate:.2f} ℃/min ({self.hotplate_com_port}, {baud}bps)"
            )
        except Exception as ex:
            QMessageBox.critical(self, "加热台控制失败", str(ex))

    def send_hotplate_stop(self):
        try:
            baud = self._send_hotplate_commands(["TEMPerature:STOP"])
            self._reset_ramp_guard_state()
            self.append_status(f"[加热台] STOP 已发送 ({self.hotplate_com_port}, {baud}bps)")
        except Exception as ex:
            QMessageBox.critical(self, "加热台控制失败", str(ex))


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
