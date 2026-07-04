from __future__ import annotations
import json
import os
import re
import sys
import time
import traceback
import threading
import gc
from contextlib import contextmanager
from datetime import datetime
from decimal import Decimal, ROUND_HALF_UP
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
from PyQt5.QtCore import Qt, QSettings, QTimer
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
    QLineEdit,
    QGridLayout,
    QLabel,
    QHBoxLayout,
    QWidget,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QStyledItemDelegate,
    QSpinBox,
)

from laser_lifetime_calibration_ui import Figure, FigureCanvas, LaserLifetimeCalibrationUI


SERIAL_LOCK = "C030795"
DEFAULT_TIMEOUT_MS = 20000
CURVE_TIMEOUT_MS = 12000
MDO34_CURVE_TIMEOUT_MS = 120000
SCREENSHOT_TIMEOUT_MS = 20000
DISCOVERY_TIMEOUT_MS = 1500
SCOPE_LOCK_DISCOVERY_TIMEOUT_MS = 2500
SCOPE_RECOVERY_CLEAR_TIMEOUT_MS = 1000
DPO2024B_SAVE_SCREENSHOT = False
SCOPE_HEARTBEAT_BEFORE_CYCLE = True
SCOPE_LOCK_ATTEMPTS = 4
HOTPLATE_BAUD_CANDIDATES = (9600, 115200)
HOTPLATE_QUERY_WAIT_S = 0.35
PLOT_TICK_S = 0.2
SIGGEN_BAUD = 115200
SIGGEN_COMMAND_DELAY_S = 0.2
SIGGEN_LIFETIME_FREQ_HZ = 10.0
SIGGEN_LIFETIME_AMPLITUDE_VPP = 5.0
SIGGEN_LIFETIME_OFFSET_V = 2.5
SIGGEN_LIFETIME_DUTY_PERCENT = 50.0
SIGGEN_SPECTRUM_DC_V = 5.0
TARGET_HOLD_S = 20.0
MAX_RAMP_RATE = 20.0
SERIAL_PROBE_TIMEOUT_S = 0.1
PRESS_CONTROL_SLAVE = 2
TEST_MODE = False
TEST_SIM_SPEEDUP = 120.0
TEST_MAX_STABLE_HOLD_S = 2.0
TEST_MAX_LIGHT_STABLE_S = 1.0
DEFAULT_TEMP_TOL = 0.20
DEFAULT_PRESS_TOL = 0.50
DEFAULT_TEMP_HOLD_S = 300.0
DEFAULT_PRESS_HOLD_S = 300.0
TEST_CAPTURE_POINTS = 1000
LOG_HOTPLATE_COMMANDS = False
LOG_CAPTURE_DETAIL = False
LOG_STAGE_DETAIL_LIMIT = 4
CURRENT_SCOPE_SAVE_FORMAT = "CSV"
FNAME_RE = re.compile(r"^T(\d+)(?:[_-]?(CH[1-4](?:CH[1-4])*))?\.(?:CSV|NPZ|PNG)(?:\.NPZ)?$", re.IGNORECASE)
TEST_PNG_BYTES = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
    b"\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc\xf8\xff\xff?"
    b"\x00\x05\xfe\x02\xfeA\xe2!\xbc\x00\x00\x00\x00IEND\xaeB`\x82"
)
SPECTRUM_DATA_START_MARKER = ">>>>>Begin Spectral Data<<<<<"
SPECTRUM_WAVELENGTH_QUANTUM = Decimal("0.001")


class ScopeProfile:
    def __init__(
        self,
        name: str,
        idn_keywords,
        stopafter_run_cmd: str = "ACQ:STOPAFTER RUNST",
        stopafter_single_cmd: str = "ACQ:STOPAFTER SEQ",
        acq_run_cmd: str = "ACQ:STATE RUN",
        acq_stop_cmd: str = "ACQ:STATE STOP",
        acquire_mode_query: str = "ACQ:MODE?",
        acquire_state_query: str = "ACQ:STATE?",
        acquire_numavg_query: str = "ACQ:NUMAVG?",
        acquire_numacq_query: str = "ACQ:NUMACQ?",
        acquire_stopafter_query: str = "ACQ:STOPAFTER?",
        trigger_state_query: str = "TRIG:STATE?",
        acquire_mode_set_cmd: str = "ACQ:MODE {mode}",
        acquire_numavg_set_cmd: str = "ACQ:NUMAVG {numavg}",
    ):
        self.name = name
        self.idn_keywords = [str(x).upper() for x in idn_keywords]
        self.stopafter_run_cmd = stopafter_run_cmd
        self.stopafter_single_cmd = stopafter_single_cmd
        self.acq_run_cmd = acq_run_cmd
        self.acq_stop_cmd = acq_stop_cmd
        self.acquire_mode_query = acquire_mode_query
        self.acquire_state_query = acquire_state_query
        self.acquire_numavg_query = acquire_numavg_query
        self.acquire_numacq_query = acquire_numacq_query
        self.acquire_stopafter_query = acquire_stopafter_query
        self.trigger_state_query = trigger_state_query
        self.acquire_mode_set_cmd = acquire_mode_set_cmd
        self.acquire_numavg_set_cmd = acquire_numavg_set_cmd


PROFILE_DPO2024B = ScopeProfile("DPO2024B", ["DPO2024B"])
PROFILE_MDO34 = ScopeProfile("MDO34", ["MDO34"])
SUPPORTED_SCOPE_PROFILES = [PROFILE_DPO2024B, PROFILE_MDO34]


def profile_from_idn(idn: str) -> ScopeProfile:
    idn_upper = str(idn).upper()
    for profile in SUPPORTED_SCOPE_PROFILES:
        if any(keyword in idn_upper for keyword in profile.idn_keywords):
            return profile
    raise RuntimeError(
        "未识别到支持的示波器型号，已停止操作以避免错误 SCPI 阻塞通信。\n\n"
        f"IDN: {idn}\n\n"
        "当前仅支持 DPO2024B 和 MDO34。"
    )


def extract_last_value(resp: str) -> str:
    text = str(resp).strip()
    if " " in text:
        return text.split()[-1]
    return text


def normalize_acq_mode(resp: str) -> str:
    text = extract_last_value(resp).upper()
    if text in ("AVE", "AVER", "AVERAGE"):
        return "AVERAGE"
    if text in ("SAM", "SAMPLE"):
        return "SAMPLE"
    return text


def normalize_acq_state(resp: str) -> str:
    text = extract_last_value(resp).upper()
    if text in ("1", "ON", "RUN", "RUNNING"):
        return "1"
    if text in ("0", "OFF", "STOP", "STOPPED"):
        return "0"
    return text


def normalize_stopafter(resp: str) -> str:
    text = extract_last_value(resp).upper()
    if text in ("RUNSTOP", "RUNST", "RUN/STOP"):
        return "RUNST"
    if text in ("SEQ", "SEQUENCE", "SINGLE"):
        return "SEQ"
    return text


class TaskStoppedError(Exception):
    pass


class TestScope:
    profile = PROFILE_DPO2024B

    def close(self):
        pass

    def clear(self):
        pass


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


class SpectrumPreviewDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("光谱实时预览")
        self.resize(760, 460)
        self.wavelengths = None
        self.intensities = None

        layout = QVBoxLayout(self)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("积分时间(ms):"))
        self.integration_spin = QSpinBox()
        self.integration_spin.setRange(1, 600000)
        self.integration_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.integration_spin.setValue(int(getattr(parent, "spectrum_integration_ms", 100)))
        self.integration_spin.setEnabled(False)
        controls.addWidget(self.integration_spin)

        controls.addWidget(QLabel("平均次数:"))
        self.average_spin = QSpinBox()
        self.average_spin.setRange(1, 100)
        self.average_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.average_spin.setValue(int(getattr(parent, "spectrum_average_count", 1)))
        self.average_spin.setEnabled(False)
        controls.addWidget(self.average_spin)

        controls.addWidget(QLabel("刷新间隔(ms):"))
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(1000, 60000)
        self.interval_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.interval_spin.setValue(3000)
        controls.addWidget(self.interval_spin)

        self.start_button = QPushButton("开始")
        self.stop_button = QPushButton("停止")
        self.stop_button.setEnabled(False)
        controls.addWidget(self.start_button)
        controls.addWidget(self.stop_button)
        layout.addLayout(controls)

        self.status_label = QLabel("Ready")
        if Figure is None or FigureCanvas is None:
            self.canvas = None
            self.line = None
            layout.addWidget(QLabel("未检测到 matplotlib，无法显示实时曲线。"))
        else:
            self.figure = Figure(figsize=(7.4, 3.8), dpi=100)
            self.ax = self.figure.add_subplot(111)
            self.ax.set_xlabel("Wavelength (nm)")
            self.ax.set_ylabel("Counts")
            self.ax.grid(color="#d9e2ec", linestyle="-", linewidth=0.8)
            self.ax.set_ylim(0, 63000)
            self.line, = self.ax.plot([], [], linewidth=1.0)
            self.canvas = FigureCanvas(self.figure)
            layout.addWidget(self.canvas, 1)
        layout.addWidget(self.status_label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_once)
        self.start_button.clicked.connect(self.start_preview)
        self.stop_button.clicked.connect(self.stop_preview)

    def start_preview(self):
        if self.canvas is None:
            self.status_label.setText("缺少 matplotlib")
            return

        self.read_once()
        self.timer.start(int(self.interval_spin.value()))
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.interval_spin.setEnabled(False)

    def read_once(self):
        if self.canvas is None:
            return
        spectrometer = None
        try:
            import seabreeze.spectrometers as sb

            integration_ms = int(getattr(self.parent(), "spectrum_integration_ms", self.integration_spin.value()))
            average_count = int(getattr(self.parent(), "spectrum_average_count", self.average_spin.value()))
            self.integration_spin.setValue(integration_ms)
            self.average_spin.setValue(average_count)
            spectrometer = sb.Spectrometer.from_first_available()
            spectrometer.integration_time_micros(integration_ms * 1000)
            wavelengths, intensities = read_averaged_spectrum(spectrometer, average_count)
            self.wavelengths = np.asarray(wavelengths, dtype=float)
            self.intensities = np.asarray(intensities, dtype=float)
            self.line.set_data(self.wavelengths, self.intensities)
            self.ax.set_xlim(float(np.nanmin(self.wavelengths)), float(np.nanmax(self.wavelengths)))
            y_max = float(np.nanmax(self.intensities))
            self.ax.set_ylim(0, 63000)
            self.canvas.draw_idle()
            self.status_label.setText(
                f"读取成功: {len(self.wavelengths)} 点, max={y_max:.0f}, 积分={self.integration_spin.value()} ms, 平均={self.average_spin.value()}"
            )
        except ImportError:
            self.status_label.setText("未安装 seabreeze")
            self.stop_preview()
        except Exception as ex:
            self.status_label.setText(f"读取失败: {ex}")
            self.stop_preview()
        finally:
            try:
                if spectrometer is not None:
                    spectrometer.close()
            except Exception:
                pass

    def stop_preview(self):
        self.timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.interval_spin.setEnabled(True)

    def closeEvent(self, event):
        self.stop_preview()
        event.accept()


class AxisModeDelegate(QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        combo = QComboBox(parent)
        combo.addItem("Ramp", "RAMP")
        combo.addItem("Hold", "HOLD")
        return combo

    def setEditorData(self, editor, index):
        mode = (index.data(Qt.EditRole) or index.data(Qt.DisplayRole) or "Ramp").strip().upper()
        editor.setCurrentIndex(1 if mode == "HOLD" else 0)

    def setModelData(self, editor, model, index):
        model.setData(index, editor.currentText(), Qt.EditRole)


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


def build_tcpip_resource(address_or_resource: str) -> str:
    text = (address_or_resource or "").strip()
    if not text:
        return ""
    if "::" in text:
        return text
    return f"TCPIP0::{text}::inst0::INSTR"


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


def discover_scope_resource_robust(
    serial_lock: str,
    cached_resource: str = "",
    manual_resource: str = "",
    connection_mode: str = "AUTO",
):
    rm = pyvisa.ResourceManager()
    try:
        resources = list(rm.list_resources())
        usb_resources = [
            r for r in resources
            if r.upper().startswith("USB") and "::INSTR" in r.upper()
        ]
        tcpip_resources = [r for r in resources if r.upper().startswith("TCPIP")]

        connection_mode = (connection_mode or "AUTO").upper()
        use_usb = connection_mode in ("AUTO", "USB")
        use_tcpip = connection_mode in ("AUTO", "TCPIP")

        search_order = []
        manual_resource = build_tcpip_resource(manual_resource) if use_tcpip else ""
        cached_is_allowed = (
            not cached_resource
            or connection_mode == "AUTO"
            or (use_usb and cached_resource.upper().startswith("USB"))
            or (use_tcpip and cached_resource.upper().startswith("TCPIP"))
        )
        if manual_resource:
            search_order.append(manual_resource)
        if cached_resource and cached_is_allowed and cached_resource not in search_order:
            search_order.append(cached_resource)
        if use_usb and serial_lock:
            for r in usb_resources:
                if serial_lock.upper() in r.upper() and r not in search_order:
                    search_order.append(r)
        tek_usb = [r for r in usb_resources if "0X0699" in r.upper() or "0699" in r.upper()]
        if use_usb:
            for r in tek_usb + usb_resources:
                if r not in search_order:
                    search_order.append(r)
        if use_tcpip:
            for r in tcpip_resources:
                if r not in search_order:
                    search_order.append(r)

        if not search_order:
            raise RuntimeError(
                "未发现任何可尝试的 VISA 资源。请确认示波器已开机、NI-VISA 已安装，"
                "并检查初始化窗口里的示波器连接方式。"
            )

        matches = []
        probe_errors = []
        for resource in search_order:
            scope = None
            try:
                scope = rm.open_resource(resource)
                scope.timeout = SCOPE_LOCK_DISCOVERY_TIMEOUT_MS
                scope.read_termination = "\n"
                scope.write_termination = "\n"
                try:
                    scope.clear()
                except Exception:
                    pass
                idn = scope.query("*IDN?").strip()
                maker, model, serial, firmware = _parse_idn(idn)
                profile = profile_from_idn(idn)
                info = {
                    "resource": resource,
                    "idn": idn,
                    "maker": maker,
                    "model": model,
                    "serial": serial,
                    "firmware": firmware,
                    "profile": profile,
                }
                if serial_lock and serial.upper() == serial_lock.upper():
                    return info
                matches.append(info)
            except Exception as ex:
                probe_errors.append(f"{resource} -> {ex}")
            finally:
                if scope is not None:
                    try:
                        scope.close()
                    except Exception:
                        pass

        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            options = "\n".join(f"{item['resource']} -> {item['idn']}" for item in matches)
            raise RuntimeError(f"发现多个支持的 Tek USB 示波器，但无法自动决定目标：\n{options}")

        error_text = "\n".join(probe_errors[:12]) if probe_errors else "(无可用探测结果)"
        raise RuntimeError(
            "未找到支持的 Tek 示波器。当前 USB 资源：\n"
            + ("\n".join(usb_resources) if usb_resources else "(无)")
            + "\n\n当前 TCPIP 资源：\n"
            + ("\n".join(tcpip_resources) if tcpip_resources else "(无)")
            + "\n\n探测结果：\n"
            + error_text
        )
    finally:
        rm.close()


def open_scope(resource_str: str):
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(resource_str)
    scope.timeout = DEFAULT_TIMEOUT_MS
    scope.read_termination = "\n"
    scope.write_termination = "\n"
    idn = scope.query("*IDN?").strip()
    scope.profile = profile_from_idn(idn)
    return rm, scope, idn


def force_clear_scope_resource(resource_str: str):
    if not resource_str:
        return None

    rm = scope = None
    try:
        rm = pyvisa.ResourceManager()
        scope = rm.open_resource(resource_str)
        scope.timeout = SCOPE_RECOVERY_CLEAR_TIMEOUT_MS
        scope.read_termination = "\n"
        scope.write_termination = "\n"
        try:
            scope.clear()
        except Exception:
            pass
        try:
            scope.write("*CLS")
        except Exception:
            pass
        try:
            drain_input_buffer(scope, timeout_ms=80)
        except Exception:
            pass
        return None
    except Exception as ex:
        return str(ex)
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
        gc.collect()


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


def _get_record_length_dpo2024b(scope) -> int:
    try:
        return int(float(_q(scope, "WFMOUTPRE:RECORDLENGTH?")))
    except Exception:
        return int(float(_q(scope, "HORIZONTAL:RECORDLENGTH?")))


def _get_record_length_mdo34(scope) -> int:
    return int(float(extract_last_value(_q(scope, "HORIZONTAL:RECORDLENGTH?"))))


def _get_transfer_points_mdo34(scope) -> int:
    return int(float(extract_last_value(_q(scope, "WFMOUTPRE:NR_PT?"))))


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


def _select_composition_and_width_dpo2024b(scope):
    comp = _q(scope, "DATA:COMPOSITION:AVAILABLE?").strip().upper()
    if "SINGULAR_YT" in comp:
        _w(scope, "DATA:COMPOSITION SINGULAR_YT")
        _w(scope, "DATA:WIDTH 2")
        return 2, "H", "SINGULAR_YT"
    _w(scope, "DATA:COMPOSITION COMPOSITE_YT")
    _w(scope, "DATA:WIDTH 1")
    return 1, "B", "COMPOSITE_YT"


def read_waveform(scope, ch_name: str):
    if scope.profile.name == "DPO2024B":
        return read_waveform_dpo2024b(scope, ch_name)
    if scope.profile.name == "MDO34":
        return read_waveform_mdo34(scope, ch_name)
    raise RuntimeError(f"不支持的示波器型号：{scope.profile.name}")


def read_waveform_dpo2024b(scope, ch_name: str):
    _w(scope, f"DATA:SOURCE {ch_name}")
    time.sleep(0.12)

    _w(scope, "DATA:ENC RPB")
    try:
        _w(scope, "DATA:RESOLUTION FULL")
    except Exception:
        pass

    width, dtype, comp_used = _select_composition_and_width_dpo2024b(scope)
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

    recl = _get_record_length_dpo2024b(scope)
    _w(scope, "DATA:START 1")
    _w(scope, f"DATA:STOP {recl}")

    is_big_endian = width == 2 and "MSB" in byt_or
    raw = None
    transfer_error = None
    for attempt in range(1, 3):
        try:
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
            transfer_error = None
            break
        except Exception as ex:
            transfer_error = ex
            try:
                scope.clear()
                scope.write("*CLS")
                drain_input_buffer(scope, timeout_ms=80)
            except Exception:
                pass
            if attempt < 2:
                time.sleep(0.18)

    if transfer_error is not None or raw is None:
        raise RuntimeError(f"{ch_name} 波形读取失败（CURVE传输超时/中断）: {transfer_error}")

    try:
        drain_input_buffer(scope, timeout_ms=80)
    except Exception:
        pass

    if len(raw) != recl:
        recl = len(raw)
    v = (raw - yoff) * ymult + yzero
    t = xzero + np.arange(recl) * xincr
    return t, v, {
        "scope": "DPO2024B",
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


def read_waveform_mdo34(scope, ch_name: str):
    # MDO34 fixed full-record path. Do not send DATA:COMPOSITION here.
    _w(scope, f"DATA:SOURCE {ch_name}")
    time.sleep(0.08)

    record_length = _get_record_length_mdo34(scope)
    _w(scope, "DATA:START 1")
    _w(scope, f"DATA:STOP {record_length}")

    # Validated MDO34 command chain.
    _w(scope, "WFMOUTPRE:ENCDG BINARY")
    _w(scope, "WFMOUTPRE:BYT_NR 2")
    _w(scope, "WFMOUTPRE:BN_FMT RI")
    _w(scope, "WFMOUTPRE:BYT_OR MSB")

    preamble = "WFMOUTPRE"
    recl = _get_transfer_points_mdo34(scope)

    ymult = float(extract_last_value(_q(scope, f"{preamble}:YMULT?")))
    yzero = float(extract_last_value(_q(scope, f"{preamble}:YZERO?")))
    yoff = float(extract_last_value(_q(scope, f"{preamble}:YOFF?")))
    xincr = float(extract_last_value(_q(scope, f"{preamble}:XINCR?")))
    xzero = float(extract_last_value(_q(scope, f"{preamble}:XZERO?")))

    data_enc = _q(scope, "WFMOUTPRE:ENCDG?").strip()
    byt_nr = _q(scope, f"{preamble}:BYT_NR?").strip()
    bn_fmt = _q(scope, f"{preamble}:BN_FMT?").strip()
    byt_or = _q(scope, f"{preamble}:BYT_OR?").strip().upper()
    width = int(float(extract_last_value(byt_nr)))
    dtype = "h" if width == 2 else "b"
    is_big_endian = width == 2 and "MSB" in byt_or

    raw = None
    transfer_error = None
    for attempt in range(1, 3):
        try:
            with temporary_timeout(scope, MDO34_CURVE_TIMEOUT_MS):
                with binary_transfer_mode(scope):
                    raw = scope.query_binary_values(
                        "CURVE?",
                        datatype=dtype,
                        container=np.array,
                        is_big_endian=is_big_endian,
                        expect_termination=True,
                        chunk_size=1024 * 1024,
                    )
            transfer_error = None
            break
        except Exception as ex:
            transfer_error = ex
            try:
                scope.clear()
                scope.write("*CLS")
                drain_input_buffer(scope, timeout_ms=80)
            except Exception:
                pass
            if attempt < 2:
                time.sleep(0.18)

    if transfer_error is not None or raw is None:
        raise RuntimeError(f"{ch_name} 波形读取失败（MDO34 CURVE传输超时/中断）: {transfer_error}")

    try:
        drain_input_buffer(scope, timeout_ms=80)
    except Exception:
        pass

    if len(raw) != recl:
        recl = len(raw)

    v = (raw - yoff) * ymult + yzero
    t = xzero + np.arange(recl) * xincr

    return t, v, {
        "scope": "MDO34",
        "width": width,
        "dtype": dtype,
        "composition": "N/A",
        "recl": recl,
        "record_length": record_length,
        "data_enc": data_enc,
        "byt_nr": byt_nr,
        "bn_fmt": bn_fmt,
        "byt_or": byt_or,
        "ymult": ymult,
        "yoff": yoff,
        "yzero": yzero,
        "xincr": xincr,
        "xzero": xzero,
        "raw_min": float(np.nanmin(raw)) if len(raw) else None,
        "raw_max": float(np.nanmax(raw)) if len(raw) else None,
        "scale_formula": "(raw - YOFF) * YMULT + YZERO",
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


def normalize_save_format(save_format):
    fmt = str(save_format or "CSV").strip().upper()
    if fmt not in {"CSV", "NPZ"}:
        return "CSV"
    return fmt


def make_filename(idx: int, channels, save_format=None):
    suffix = "".join(sorted(channels))
    ext = normalize_save_format(save_format or CURRENT_SCOPE_SAVE_FORMAT)
    return f"T{idx:04d}{suffix}.{ext}"


def make_screenshot_filename(idx: int, channels):
    name, _ = os.path.splitext(make_filename(idx, channels))
    return f"{name}.PNG"


def existing_capture_path_variants(path: str) -> list[str]:
    variants = [path]
    if path.lower().endswith(".npz"):
        variants.append(path + ".npz")
    return variants


def make_available_capture_paths(save_dir: str, start_idx: int, channels, save_format="CSV"):
    idx = max(start_idx, next_index(save_dir))
    while True:
        waveform_name = make_filename(idx, channels, save_format)
        screenshot_name = make_screenshot_filename(idx, channels)
        waveform_path = os.path.join(save_dir, waveform_name)
        screenshot_path = os.path.join(save_dir, screenshot_name)
        if not any(os.path.exists(path) for path in existing_capture_path_variants(waveform_path)) and not os.path.exists(screenshot_path):
            return idx, waveform_path, screenshot_path
        idx += 1


def _next_non_existing_capture_path(save_path: str) -> str:
    if not os.path.exists(save_path):
        return save_path
    folder = os.path.dirname(save_path) or "."
    filename = os.path.basename(save_path)
    match = FNAME_RE.match(filename)
    if match:
        channels = match.group(2) or ""
        ext = os.path.splitext(filename)[1].lstrip(".").upper()
        idx = next_index(folder)
        while True:
            candidate = os.path.join(folder, f"T{idx:04d}{channels}.{ext}")
            if not os.path.exists(candidate):
                return candidate
            idx += 1

    stem, ext = os.path.splitext(filename)
    idx = 1
    while True:
        candidate = os.path.join(folder, f"{stem}_{idx}{ext}")
        if not os.path.exists(candidate):
            return candidate
        idx += 1


def save_csv(save_path: str, data: dict):
    if os.path.splitext(save_path)[1].lower() == ".npz":
        save_npz(save_path, data)
        return
    if os.path.exists(save_path):
        raise FileExistsError(f"Capture file already exists: {save_path}")
    pd.DataFrame(data).to_csv(save_path, index=False, float_format="%.5f")


def save_npz(save_path: str, data: dict, meta=None):
    if any(os.path.exists(path) for path in existing_capture_path_variants(save_path)):
        raise FileExistsError(f"Capture file already exists: {save_path}")
    arrays = {name: np.asarray(values) for name, values in data.items()}
    if meta is not None:
        arrays["__meta_json__"] = np.asarray(json.dumps(meta, ensure_ascii=False, indent=2))
    with open(save_path, "wb") as file:
        np.savez(file, **arrays)


def save_waveform_file(save_path: str, data: dict, save_format="CSV", meta=None):
    if normalize_save_format(save_format) == "NPZ":
        save_npz(save_path, data, meta=meta)
    else:
        save_csv(save_path, data)


def read_averaged_spectrum(spectrometer, scans_to_average: int):
    scans = max(1, int(scans_to_average))
    wavelengths, intensities = spectrometer.spectrum()
    if scans <= 1:
        return wavelengths, intensities
    summed = np.asarray(intensities, dtype=float)
    for _ in range(scans - 1):
        _, next_intensities = spectrometer.spectrum()
        summed += np.asarray(next_intensities, dtype=float)
    return wavelengths, summed / scans


def _extract_scpi_block_payload(raw: bytes) -> bytes:
    if raw.startswith(b"#") and len(raw) >= 2 and raw[1:2].isdigit():
        digit_count = int(raw[1:2])
        if digit_count > 0:
            len_start = 2
            len_end = len_start + digit_count
            data_len = int(raw[len_start:len_end])
            return raw[len_end:len_end + data_len]
        return raw[2:].rstrip(b"\r\n")

    png_start = raw.find(b"\x89PNG\r\n\x1a\n")
    if png_start > 0:
        return raw[png_start:].rstrip(b"\r\n")
    return raw.rstrip(b"\r\n")


def save_scope_screenshot(scope, save_path: str):
    save_path = _next_non_existing_capture_path(save_path)
    old_timeout = scope.timeout
    try:
        _w(scope, "SAVe:IMAGe:FILEFormat PNG")
        with temporary_timeout(scope, SCREENSHOT_TIMEOUT_MS):
            with binary_transfer_mode(scope):
                scope.write("HARDCopy START")
                raw = scope.read_raw()
        image_data = _extract_scpi_block_payload(raw)
        if not image_data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise RuntimeError("示波器返回的截屏数据不是有效 PNG")
        with open(save_path, "wb") as f:
            f.write(image_data)
    finally:
        scope.timeout = old_timeout


class MainWindow(LaserLifetimeCalibrationUI):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("AutoLifeCal", "ScopeSaver")
        self.selected_channels = {"CH2"}
        self.resource_str = None
        self.idn = None
        self.profile_name = None
        self.allowed_avg_counts = [2, 4, 8, 16, 32, 64, 128, 256, 512]
        self.control_slave = 1
        self.press_control_slave = PRESS_CONTROL_SLAVE
        self.ramp_target_temp = None
        self.ramp_rate_cpm = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        self.current_temp_tol = None
        self.current_temp_hold_s = None
        self.last_hotplate_s_status = None
        self.hotplate_guard_timer = QTimer(self)
        self.hotplate_guard_timer.setInterval(1200)
        self.hotplate_guard_timer.timeout.connect(self.check_hotplate_ramp_guard)

        # 压力 RAMP 独立守护。温度原来有提前 STOP 监控，压力这里补齐同级逻辑。
        # 不设置重发次数上限：如果设备偶发 STOP 且压力未达标，会按冷却间隔持续补发 RAMP。
        self.press_ramp_target = None
        self.press_ramp_rate_kpm = None
        self.press_reach_inband_since = None
        self.press_reach_confirmed = False
        self.current_press_tol = None
        self.current_press_hold_s = None
        self.last_hotplate_press_s_status = None
        self.press_guard_last_restart_ts = 0.0
        self.hotplate_press_guard_timer = QTimer(self)
        self.hotplate_press_guard_timer.setInterval(1200)
        self.hotplate_press_guard_timer.timeout.connect(self.check_hotplate_press_ramp_guard)

        self.hotplate_live_timer = QTimer(self)
        self.hotplate_live_timer.setInterval(1000)
        self.hotplate_live_timer.timeout.connect(self.poll_hotplate_live_data)
        self.hotplate_plot_timer = QTimer(self)
        self.hotplate_plot_timer.setInterval(int(PLOT_TICK_S * 1000))
        self.hotplate_plot_timer.timeout.connect(self._tick_hotplate_plot)
        self.hotplate_live_err_suppressed = False
        self.last_live_tc = None
        self.last_live_pk = None
        self.sim_tf_current = float(self.ref_temp_spin.value())
        self.sim_tf_target = self.sim_tf_current
        self.sim_rate_cpm = 0.0
        self.sim_temp_mode = "hold"  # "hold" | "ramp"
        self.sim_pf_current = float(self.ref_press_spin.value())
        self.sim_pf_target = self.sim_pf_current
        self.sim_prate_kpm = 0.0
        self.sim_press_mode = "hold"  # "hold" | "ramp"
        self.last_sim_ts = time.time()
        self.live_temp_since = None
        self.live_press_since = None
        self.live_temp_stable = False
        self.live_press_stable = False
        self.detect_thread = None
        self._ui_init_done = False
        self.stop_requested = threading.Event()
        self.pause_requested = threading.Event()
        self.task_running = False
        self.sensor_log_file_path = None
        self.status_log_file_path = None
        self.data_log_enabled = False
        self._log_write_err_reported = False
        self.info_fit_window = None
        self.spectrum_preview_window = None
        self.siggen_com_port = ""
        self.siggen_channels = {"CH1"}
        self.scope_manual_resource = ""
        self.scope_connection_mode = "USB"
        self.spectrum_integration_ms = 100
        self.spectrum_average_count = 1
        self.siggen_lifetime_freq_hz = SIGGEN_LIFETIME_FREQ_HZ

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
        self.spectrum_preview_btn = QPushButton("光谱实时")
        self.spectrum_preview_btn.clicked.connect(self.open_spectrum_preview)
        self.capture_back_spectrum_btn = QPushButton("采集BACK")
        self.capture_back_spectrum_btn.clicked.connect(self.capture_background_spectrum)
        action_layout = self.init_btn.parentWidget().layout()
        if action_layout is not None:
            action_layout.insertWidget(1, self.spectrum_preview_btn, 1)
            action_layout.insertWidget(2, self.capture_back_spectrum_btn, 1)
        self.start_btn.setCheckable(False)
        self.start_btn.clicked.connect(self.on_run_pause_clicked)
        self.stop_btn.clicked.connect(self.request_task_stop)
        self.avg_checkbox.toggled.connect(self.on_avg_toggled)
        self.single_mode_btn.toggled.connect(self.on_capture_mode_toggled)
        self.save_format_combo.currentIndexChanged.connect(self.on_save_format_changed)
        self.fit_settings_btn.clicked.connect(self.open_lifetime_fit_settings)
        self.fit_enable_btn.toggled.connect(self.on_fit_enable_toggled)
        self.temp_custom_edit_btn.clicked.connect(self.edit_custom_axis_path)

        self.load_port_settings()
        self.load_saved_path()
        self.load_fit_enable_setting()
        self.load_save_format_setting()
        self.load_run_settings()
        self.load_custom_axis_settings()
        self.avg_count_combo.currentIndexChanged.connect(self.on_avg_count_changed)
        self.cycles_spin.valueChanged.connect(self.on_run_setting_changed)
        self.stable_time_spin.valueChanged.connect(self.on_run_setting_changed)
        self.on_avg_toggled(self.avg_checkbox.isChecked())
        self.update_path_summary()
        self.settings_button.hide()
        self.test_mode_label.setVisible(TEST_MODE)
        self._ui_init_done = True
        self.append_status("[系统] 已加载主窗口")

    def get_calibration_mode(self) -> str:
        mode = self.calibration_mode_combo.currentData()
        if mode not in ("LIFETIME", "SPECTRUM", "LIFETIME_SPECTRUM"):
            return "LIFETIME"
        return str(mode)

    @staticmethod
    def _format_spectrum_condition_value(value, unit: str) -> str:
        if value is None:
            return f"current{unit}"
        try:
            return f"{float(value):g}{unit}"
        except Exception:
            return f"{value}{unit}"

    def _spectrum_filename_prefix(self, stage: dict) -> str:
        temp_text = self._format_spectrum_condition_value(stage.get("temp"), "C")
        press_text = self._format_spectrum_condition_value(stage.get("press"), "kPa")
        prefix = f"{temp_text}_{press_text}"
        return re.sub(r"[^0-9A-Za-z_.+-]+", "_", prefix).strip("_") or "spectrum"

    def _write_oceanview_spectrum_txt(self, path: str, wavelengths, intensities, metadata: dict[str, str]) -> str:
        save_path = _next_non_existing_capture_path(path)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w", encoding="gb18030") as file:
            for key, value in metadata.items():
                file.write(f"# {key}: {value}\n")
            file.write(f"{SPECTRUM_DATA_START_MARKER}\n")
            for wavelength, intensity in zip(wavelengths, intensities):
                official_wavelength = Decimal(str(float(wavelength))).quantize(
                    SPECTRUM_WAVELENGTH_QUANTUM,
                    rounding=ROUND_HALF_UP,
                )
                file.write(f"{official_wavelength}\t{float(intensity):.0f}\n")
        return save_path

    def _capture_spectrum_txt(self, save_path: str, metadata: dict[str, str]) -> str:
        integration_ms = int(self.spectrum_integration_ms)
        integration_us = integration_ms * 1000
        average_count = max(1, int(self.spectrum_average_count))
        metadata = {
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "integration_time_us": str(integration_us),
            "scans_to_average": str(average_count),
            **metadata,
        }

        if TEST_MODE:
            wavelengths = np.linspace(315.719, 1027.352, TEST_CAPTURE_POINTS)
            intensities = 4000.0 + 150.0 * np.sin(np.linspace(0, 8, TEST_CAPTURE_POINTS))
            metadata.update({"model": "TEST_SPECTROMETER", "serial_number": "TEST"})
            return self._write_oceanview_spectrum_txt(save_path, wavelengths, intensities, metadata)

        try:
            import seabreeze.spectrometers as sb
        except ImportError as exc:
            raise RuntimeError("未安装 seabreeze，无法保存光谱。请先安装: pip install seabreeze") from exc

        spectrometer = None
        try:
            spectrometer = sb.Spectrometer.from_first_available()
            spectrometer.integration_time_micros(integration_us)
            wavelengths, intensities = read_averaged_spectrum(spectrometer, average_count)
            metadata.update(
                {
                    "model": str(getattr(spectrometer, "model", "")),
                    "serial_number": str(getattr(spectrometer, "serial_number", "unknown")),
                }
            )
            return self._write_oceanview_spectrum_txt(save_path, wavelengths, intensities, metadata)
        finally:
            try:
                if spectrometer is not None:
                    spectrometer.close()
            except Exception:
                pass

    def _capture_spectrum_for_stage(
        self,
        save_dir: str,
        stage: dict,
        stage_idx: int,
        stage_total: int,
        cycle_no: int | None = None,
        cycle_total: int | None = None,
    ) -> str:
        prefix = self._spectrum_filename_prefix(stage)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        save_path = os.path.join(save_dir, f"{prefix}_{stamp}.txt")

        metadata = {
            "calibration_mode": self.get_calibration_mode(),
            "stage": f"{stage_idx}/{stage_total}",
            "cycle": "" if cycle_no is None or cycle_total is None else f"{cycle_no}/{cycle_total}",
            "temperature_C": "" if stage.get("temp") is None else f"{float(stage.get('temp')):g}",
            "pressure_kPa": "" if stage.get("press") is None else f"{float(stage.get('press')):g}",
        }

        return self._capture_spectrum_txt(save_path, metadata)

    def _capture_background_spectrum(self, save_dir: str) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        save_path = os.path.join(save_dir, f"back_{stamp}.txt")
        return self._capture_spectrum_txt(
            save_path,
            {
                "calibration_mode": "BACKGROUND",
                "stage": "",
                "cycle": "",
                "temperature_C": "",
                "pressure_kPa": "",
            },
        )

    def capture_background_spectrum(self):
        if self.task_running:
            QMessageBox.warning(self, "提示", "自动任务运行中，不能单独采集 BACK 光谱")
            return

        save_dir = self.save_path_edit.text().strip()
        if not save_dir:
            QMessageBox.warning(self, "提示", "请先选择保存路径")
            return

        try:
            os.makedirs(save_dir, exist_ok=True)
            self.persist_save_path()
            saved_path = self._capture_background_spectrum(save_dir)
        except Exception as ex:
            QMessageBox.warning(self, "采集BACK失败", str(ex))
            self.append_status(f"[光谱][BACK] 采集失败: {ex}")
            return

        self.append_status(f"[光谱][BACK] 已保存: {saved_path}")
        QMessageBox.information(self, "BACK光谱已保存", saved_path)

    def get_spectrometer_identity_status(self):
        if TEST_MODE:
            return True, "TEST_SPECTROMETER"
        try:
            import seabreeze.spectrometers as sb
        except ImportError:
            return False, "未安装 seabreeze"

        spectrometer = None
        try:
            devices = sb.list_devices()
            if not devices:
                return False, "未找到"
            spectrometer = sb.Spectrometer(devices[0])
            model = str(getattr(spectrometer, "model", "") or "").strip()
            serial_number = str(getattr(spectrometer, "serial_number", "") or "").strip()
            label = " ".join(part for part in (model, serial_number) if part)
            return True, label or str(devices[0])
        except Exception as ex:
            return False, f"失败: {str(ex)[:32]}"
        finally:
            try:
                if spectrometer is not None:
                    spectrometer.close()
            except Exception:
                pass

    def open_spectrum_preview(self):
        if self.spectrum_preview_window is None:
            self.spectrum_preview_window = SpectrumPreviewDialog(self)
            self.spectrum_preview_window.destroyed.connect(lambda *_: setattr(self, "spectrum_preview_window", None))
        self.spectrum_preview_window.show()
        self.spectrum_preview_window.raise_()
        self.spectrum_preview_window.activateWindow()

    def _current_mode_device_requirements(self) -> tuple[bool, bool]:
        mode = self.get_calibration_mode()
        needs_scope = mode in ("LIFETIME", "LIFETIME_SPECTRUM")
        needs_spectrometer = mode in ("SPECTRUM", "LIFETIME_SPECTRUM")
        return needs_scope, needs_spectrometer

    def _stage_control_order(self) -> str:
        order = self.settings.value("stage_control_order_v2", "TEMP_THEN_PRESS", type=str)
        if order not in ("TEMP_THEN_PRESS", "PRESS_THEN_TEMP"):
            return "TEMP_THEN_PRESS"
        return str(order)

    def _temp_hold_policy(self) -> str:
        policy = self.settings.value("stage_temp_hold_policy_v1", "EVERY_PRESS_POINT", type=str)
        if policy not in ("EVERY_PRESS_POINT", "FIRST_PRESSURE_POINT"):
            return "EVERY_PRESS_POINT"
        return str(policy)

    @staticmethod
    def _format_condition_value(value, unit: str) -> str:
        try:
            return f"{float(value):g}{unit}"
        except Exception:
            return f"{value}{unit}"

    def _format_value_list_preview(self, values, unit: str, limit: int = LOG_STAGE_DETAIL_LIMIT) -> str:
        formatted = [self._format_condition_value(v, unit) for v in values]
        if len(formatted) <= limit * 2:
            return "，".join(formatted)
        head = "，".join(formatted[:limit])
        tail = "，".join(formatted[-limit:])
        return f"{head}，...，{tail}"

    def _log_stage_condition_summary(self, temp_targets, press_targets, order: str):
        temp_values = [item.get("value") for item in temp_targets if item.get("value") is not None]
        press_values = [item.get("value") for item in press_targets if item.get("value") is not None]

        if temp_values and press_values:
            total = len(temp_values) * len(press_values)
            order_text = "压力优先" if order == "PRESS_THEN_TEMP" else "温度优先"
            self.append_status(
                f"[阶段] 工况摘要: 温度 {len(temp_values)} 点 ({self._format_value_list_preview(temp_values, '℃')})；"
                f"压力 {len(press_values)} 点 ({self._format_value_list_preview(press_values, 'kPa')})；"
                f"{order_text}；总工况数={total}"
            )
            return

        if temp_values:
            self.append_status(
                f"[阶段] 工况摘要: 温度 {len(temp_values)} 点 ({self._format_value_list_preview(temp_values, '℃')})"
            )
            return

        if press_values:
            self.append_status(
                f"[阶段] 工况摘要: 压力 {len(press_values)} 点 ({self._format_value_list_preview(press_values, 'kPa')})"
            )
            return

        self.append_status("[阶段] 工况摘要: 保持当前温度和压力；总工况数=1")

    def _manual_axis_defaults(self, axis: str):
        if axis == "temp":
            return "温度", "℃", "℃/min", -200.0, 800.0, "新建升温"
        return "压力", "kPa", "kPa/min", 0.0, 5000.0, "新建升压"

    def _default_axis_segments(self, axis: str) -> list[dict]:
        if axis == "temp":
            return [
                {
                    "start": 50.0,
                    "step": 50.0,
                    "end": 600.0,
                    "mode": "RAMP",
                    "rate": 5.0,
                    "tol": DEFAULT_TEMP_TOL,
                    "hold_s": DEFAULT_TEMP_HOLD_S,
                }
            ]
        return [
            {
                "start": 101.3,
                "step": 5.0,
                "end": 121.3,
                "mode": "RAMP",
                "rate": 5.0,
                "tol": DEFAULT_PRESS_TOL,
                "hold_s": DEFAULT_PRESS_HOLD_S,
            }
        ]

    @staticmethod
    def _axis_path_file_filter() -> str:
        return "标定路径 (*.json);;所有文件 (*)"

    def _axis_path_payload(self, axis: str, segments: list[dict]) -> dict:
        axis_name, value_unit, rate_unit, *_ = self._manual_axis_defaults(axis)
        return {
            "format": "AutoLifeCalAxisPath",
            "version": 1,
            "axis": axis,
            "axis_name": axis_name,
            "value_unit": value_unit,
            "rate_unit": rate_unit,
            "segments": segments,
        }

    def _segments_from_axis_path_payload(self, payload, axis: str) -> list[dict]:
        if not isinstance(payload, dict):
            raise RuntimeError("路径文件格式错误")
        file_axis = payload.get("axis")
        if file_axis not in (None, axis):
            axis_name = "温度" if axis == "temp" else "压力"
            raise RuntimeError(f"该文件不是{axis_name}路径")
        segments = payload.get("segments")
        if not isinstance(segments, list) or not segments:
            raise RuntimeError("路径文件中没有有效路径段")

        normalized = []
        for idx, item in enumerate(segments, 1):
            if not isinstance(item, dict):
                raise RuntimeError(f"第 {idx} 段格式错误")
            mode = str(item.get("mode", "RAMP")).strip().upper()
            if mode not in {"RAMP", "HOLD"}:
                raise RuntimeError(f"第 {idx} 段方式必须是 RAMP 或 HOLD")
            try:
                start = float(item["start"])
                step = float(item["step"])
                end = float(item["end"])
                tol = float(item["tol"])
                hold_s = float(item["hold_s"])
                rate = None if mode == "HOLD" else float(item["rate"])
            except (KeyError, TypeError, ValueError):
                raise RuntimeError(f"第 {idx} 段字段不完整或不是数字")
            if not all(np.isfinite(v) for v in (start, step, end, tol, hold_s)):
                raise RuntimeError(f"第 {idx} 段包含无效数字")
            if step <= 0:
                raise RuntimeError(f"第 {idx} 段间隔必须大于 0")
            if tol < 0:
                raise RuntimeError(f"第 {idx} 段波动阈值不能小于 0")
            if hold_s < 0:
                raise RuntimeError(f"第 {idx} 段稳定时间不能小于 0")
            if rate is not None and (not np.isfinite(rate) or rate <= 0):
                raise RuntimeError(f"第 {idx} 段 Ramp 速率必须大于 0")
            normalized.append(
                {
                    "start": start,
                    "step": step,
                    "end": end,
                    "mode": mode,
                    "rate": rate,
                    "tol": tol,
                    "hold_s": hold_s,
                }
            )
        return normalized

    @staticmethod
    def _set_table_item_editable(item, editable: bool):
        flags = item.flags()
        if editable:
            item.setFlags(flags | Qt.ItemIsEditable)
            item.setForeground(Qt.black)
            item.setBackground(Qt.white)
        else:
            item.setFlags(flags & ~Qt.ItemIsEditable)
            item.setForeground(Qt.gray)
            item.setBackground(Qt.lightGray)

    def _make_table_item(self, value, editable: bool = True):
        item = QTableWidgetItem(str(value))
        self._set_table_item_editable(item, editable)
        return item

    @staticmethod
    def _ensure_table_item(table, row: int, column: int, value: str = ""):
        item = table.item(row, column)
        if item is None:
            item = QTableWidgetItem(value)
            table.setItem(row, column, item)
        return item

    @staticmethod
    def _table_float(table, row: int, column: int) -> float:
        item = table.item(row, column)
        text = "" if item is None else item.text().strip()
        try:
            value = float(text)
        except ValueError:
            raise RuntimeError(f"第 {row + 1} 段第 {column + 1} 列必须填写数字")
        if not np.isfinite(value):
            raise RuntimeError(f"第 {row + 1} 段第 {column + 1} 列数字无效")
        return value

    @staticmethod
    def _normalize_axis_mode(text: str) -> str | None:
        mode = text.strip().upper()
        if mode in {"RAMP", "R"}:
            return "RAMP"
        if mode in {"HOLD", "H"}:
            return "HOLD"
        return None

    def _axis_table_mode(self, table, row: int) -> str | None:
        item = table.item(row, 3)
        mode_text = "" if item is None else item.text()
        return self._normalize_axis_mode(mode_text)

    def _sync_axis_mode_rate_cell(self, table, row: int, mode: str):
        if row < 0:
            return
        rate_item = self._ensure_table_item(table, row, 4)
        if mode == "HOLD":
            rate_item.setText("")
            self._set_table_item_editable(rate_item, False)
        else:
            self._set_table_item_editable(rate_item, True)
            if not rate_item.text().strip():
                rate_item.setText("5")

    def _sync_axis_segment_starts(self, table):
        for row in range(1, table.rowCount()):
            prev_end_item = table.item(row - 1, 2)
            if prev_end_item is None:
                continue
            prev_end = prev_end_item.text()
            item = self._ensure_table_item(table, row, 0, prev_end)
            if item.text() != prev_end:
                item.setText(prev_end)

    def _refresh_axis_start_editability(self, table):
        for row in range(table.rowCount()):
            item = self._ensure_table_item(table, row, 0)
            if row == 0:
                self._set_table_item_editable(item, True)
            else:
                self._set_table_item_editable(item, False)

    def _add_axis_segment_row(self, table, axis: str, segment: dict | None = None):
        row = table.rowCount()
        default_tol = DEFAULT_TEMP_TOL if axis == "temp" else DEFAULT_PRESS_TOL
        default_hold_s = DEFAULT_TEMP_HOLD_S if axis == "temp" else DEFAULT_PRESS_HOLD_S
        if segment is None:
            prev_end = None
            if row > 0:
                prev_end = self._table_float(table, row - 1, 2)
            start = prev_end if prev_end is not None else (float(self.ref_temp_spin.value()) if axis == "temp" else float(self.ref_press_spin.value()))
            step = 10.0 if axis == "temp" else 5.0
            end = start + step
            mode = "RAMP"
            rate = 5.0
            tol = default_tol
            hold_s = default_hold_s
        else:
            start = float(segment["start"])
            step = float(segment["step"])
            end = float(segment["end"])
            mode = segment.get("mode") or "RAMP"
            rate = float(segment.get("rate") or 5.0)
            tol = default_tol if segment.get("tol") is None else float(segment["tol"])
            hold_s = default_hold_s if segment.get("hold_s") is None else float(segment["hold_s"])

        table.insertRow(row)
        table.blockSignals(True)
        table.setItem(row, 0, self._make_table_item(f"{start:g}", editable=(row == 0)))
        table.setItem(row, 1, self._make_table_item(f"{step:g}"))
        table.setItem(row, 2, self._make_table_item(f"{end:g}"))
        table.setItem(row, 3, self._make_table_item("Hold" if mode == "HOLD" else "Ramp"))
        table.setItem(row, 4, self._make_table_item("" if mode == "HOLD" else f"{rate:g}", editable=(mode != "HOLD")))
        table.setItem(row, 5, self._make_table_item(f"{tol:g}"))
        table.setItem(row, 6, self._make_table_item(f"{hold_s:g}"))
        table.blockSignals(False)
        table.selectRow(row)

    def _segments_from_axis_table(self, table) -> list[dict]:
        segments = []
        for row in range(table.rowCount()):
            mode = self._axis_table_mode(table, row)
            if mode is None:
                raise RuntimeError(f"第 {row + 1} 段方式必须是 Ramp 或 Hold")
            rate = None
            if mode == "RAMP":
                rate = self._table_float(table, row, 4)
                if rate <= 0:
                    raise RuntimeError(f"第 {row + 1} 段 Ramp 速率必须大于 0")
            segment = {
                "start": self._table_float(table, row, 0),
                "step": self._table_float(table, row, 1),
                "end": self._table_float(table, row, 2),
                "mode": mode,
                "rate": rate,
                "tol": self._table_float(table, row, 5),
                "hold_s": self._table_float(table, row, 6),
            }
            if segment["tol"] < 0:
                raise RuntimeError(f"第 {row + 1} 段波动阈值不能小于 0")
            if segment["hold_s"] < 0:
                raise RuntimeError(f"第 {row + 1} 段稳定时间不能小于 0")
            segments.append(segment)
        return segments

    @staticmethod
    def _serialize_manual_axis_segments(segments: list[dict]) -> str:
        lines = []
        for segment in segments:
            rate = "h" if segment["mode"] == "HOLD" else f"{float(segment['rate']):g}"
            lines.append(
                f"{float(segment['start']):g}|{float(segment['step']):g}|"
                f"{float(segment['end']):g}|{segment['mode']}|{rate}|"
                f"{float(segment['tol']):g}|{float(segment['hold_s']):g}"
            )
        return "\n".join(lines)

    @staticmethod
    def _parse_serialized_manual_axis(text: str) -> list[dict] | None:
        lines = [line.strip() for line in text.strip().splitlines() if line.strip()]
        if not lines or not all("|" in line for line in lines):
            return None
        segments = []
        for line in lines:
            parts = [part.strip() for part in line.split("|")]
            if len(parts) not in (5, 7):
                return None
            try:
                start, step, end = (float(parts[i]) for i in range(3))
            except ValueError:
                return None
            if not all(np.isfinite(v) for v in (start, step, end)):
                return None
            mode = parts[3].upper()
            if mode not in {"RAMP", "HOLD"}:
                return None
            if mode == "HOLD":
                rate = None
            else:
                try:
                    rate = float(parts[4])
                except ValueError:
                    return None
                if not np.isfinite(rate):
                    return None
            tol = None
            hold_s = None
            if len(parts) == 7:
                try:
                    tol = float(parts[5])
                    hold_s = float(parts[6])
                except ValueError:
                    return None
                if not all(np.isfinite(v) for v in (tol, hold_s)):
                    return None
            segments.append(
                {
                    "start": start,
                    "step": step,
                    "end": end,
                    "mode": mode,
                    "rate": rate,
                    "tol": tol,
                    "hold_s": hold_s,
                }
            )
        return segments

    def _make_axis_path_table(self, axis: str, parent) -> QTableWidget:
        axis_name, value_unit, unit, *_ = self._manual_axis_defaults(axis)
        table = QTableWidget(0, 7, parent)
        table.setHorizontalHeaderLabels(
            [
                f"起点({value_unit})",
                f"间隔({value_unit})",
                f"终点({value_unit})",
                "方式",
                f"速率({unit})",
                f"波动阈值({value_unit})",
                "稳定时间(s)",
            ]
        )
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setItemDelegateForColumn(3, AxisModeDelegate(table))
        table.setStyleSheet(
            "QTableWidget { gridline-color: #c6d2de; }"
            "QTableWidget::item:selected {"
            "background-color: #a9d4ff;"
            "color: #102033;"
            "}"
        )

        edit = self.temp_manual_points_edit if axis == "temp" else self.press_manual_points_edit
        segments = self._parse_serialized_manual_axis(edit.text()) or []
        for segment in segments:
            self._add_axis_segment_row(table, axis, segment)
        if table.rowCount() == 0:
            self._add_axis_segment_row(table, axis)

        table_syncing = {"active": False}

        def sync_after_edit(item):
            if table_syncing["active"]:
                return
            table_syncing["active"] = True
            try:
                row = item.row()
                column = item.column()
                if column == 2:
                    self._sync_axis_segment_starts(table)
                elif column == 3:
                    rate_item = table.item(row, 4)
                    if rate_item is None:
                        return
                    mode = self._normalize_axis_mode(item.text())
                    if mode is None:
                        return
                    if mode == "HOLD":
                        item.setText("Hold")
                        rate_item.setText("")
                        self._set_table_item_editable(rate_item, False)
                    else:
                        item.setText("Ramp")
                        self._set_table_item_editable(rate_item, True)
                        if not rate_item.text().strip():
                            rate_item.setText("5")
            finally:
                table_syncing["active"] = False

        table.itemChanged.connect(sync_after_edit)
        return table

    def _load_segments_into_axis_table(self, table, axis: str, segments: list[dict]):
        table.setRowCount(0)
        for segment in segments:
            self._add_axis_segment_row(table, axis, segment)
        self._refresh_axis_start_editability(table)
        self._sync_axis_segment_starts(table)

    def _axis_tables_to_segments(self, tables: dict[str, QTableWidget]) -> dict[str, list[dict]]:
        result = {}
        for axis, table in tables.items():
            axis_name, *_ = self._manual_axis_defaults(axis)
            segments = self._segments_from_axis_table(table)
            text = self._serialize_manual_axis_segments(segments)
            self._build_manual_axis_targets(text, axis_name)
            result[axis] = segments
        return result

    def _path_plan_payload(self, segments_by_axis: dict[str, list[dict]]) -> dict:
        return {
            "format": "AutoLifeCalPathPlan",
            "version": 1,
            "paths": {
                axis: self._axis_path_payload(axis, segments)
                for axis, segments in segments_by_axis.items()
            },
        }

    def update_path_summary(self):
        label = getattr(self, "path_summary_label", None)
        if label is None:
            return
        try:
            temp_targets = self._build_manual_axis_targets(self.temp_manual_points_edit.text(), "温度")
            press_targets = self._build_manual_axis_targets(self.press_manual_points_edit.text(), "压力")
            temp_enabled = self.temp_cal_btn.isChecked()
            press_enabled = self.press_cal_btn.isChecked()
            temp_values = [item.get("value") for item in temp_targets if item.get("value") is not None]
            press_values = [item.get("value") for item in press_targets if item.get("value") is not None]
            if temp_values:
                temp_text = (
                    f"温度 {'启用' if temp_enabled else '关闭'} / {len(temp_values)} 点: "
                    f"{self._format_value_list_preview(temp_values, '', limit=3)}"
                )
                temp_tip = f"温度 {'启用' if temp_enabled else '关闭'}: {self._format_value_list_preview(temp_values, '', limit=12)}"
            else:
                temp_text = "温度未配置"
                temp_tip = temp_text
            if press_values:
                press_text = (
                    f"压力 {'启用' if press_enabled else '关闭'} / {len(press_values)} 点: "
                    f"{self._format_value_list_preview(press_values, '', limit=3)}"
                )
                press_tip = f"压力 {'启用' if press_enabled else '关闭'}: {self._format_value_list_preview(press_values, '', limit=12)}"
            else:
                press_text = "压力未配置"
                press_tip = press_text
            total = (len(temp_targets) if temp_enabled and temp_targets else 1) * (
                len(press_targets) if press_enabled and press_targets else 1
            )
            label.setText(f"{temp_text} / {press_text} / 总工况 {total}")
            label.setToolTip(f"{temp_tip}\n{press_tip}\n总工况: {total}")
        except Exception:
            label.setText("路径需要检查")
            label.setToolTip("路径需要检查")

    def edit_custom_axis_path(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("编辑温压路径")
        dialog.resize(1080, 460)
        outer_layout = QVBoxLayout(dialog)

        enable_row = QHBoxLayout()
        temp_enable_btn = QPushButton()
        temp_enable_btn.setObjectName("CalToggleButton")
        temp_enable_btn.setCheckable(True)
        temp_enable_btn.setChecked(self.temp_cal_btn.isChecked())
        press_enable_btn = QPushButton()
        press_enable_btn.setObjectName("CalToggleButton")
        press_enable_btn.setCheckable(True)
        press_enable_btn.setChecked(self.press_cal_btn.isChecked())
        enable_hint = QLabel("参与标定:")
        enable_row.addWidget(temp_enable_btn)
        enable_row.addWidget(press_enable_btn)
        enable_row.addStretch(1)
        enable_row.insertWidget(0, enable_hint)
        outer_layout.addLayout(enable_row)

        order_row = QHBoxLayout()
        order_label = QLabel("执行顺序:")
        order_combo = QComboBox(dialog)
        order_combo.addItem("先控温，温度稳定后再控压", "TEMP_THEN_PRESS")
        order_combo.addItem("先控压，压力稳定后再控温", "PRESS_THEN_TEMP")
        order_combo.setToolTip(
            "控制温度和压力都启用时的执行顺序。\n"
            "单独启用温度或压力时，此设置不会影响运行。"
        )
        current_order = self._stage_control_order()
        order_idx = order_combo.findData(current_order)
        order_combo.setCurrentIndex(order_idx if order_idx >= 0 else 0)
        order_row.addWidget(order_label)
        order_row.addWidget(order_combo, 1)
        outer_layout.addLayout(order_row)

        temp_hold_row = QHBoxLayout()
        temp_hold_label = QLabel("温稳策略:")
        temp_hold_combo = QComboBox(dialog)
        temp_hold_combo.addItem("每个压力点都等待温稳（原逻辑）", "EVERY_PRESS_POINT")
        temp_hold_combo.addItem("每个温度点仅首个压力点等待温稳", "FIRST_PRESSURE_POINT")
        temp_hold_combo.setToolTip(
            "仅在温度和压力都启用，且执行顺序为“先控温”时生效。\n"
            "第二种模式下，同一温度的第一个压力点会等待温度稳定，后续压力点只等待压力稳定。"
        )
        current_temp_hold_policy = self._temp_hold_policy()
        temp_hold_idx = temp_hold_combo.findData(current_temp_hold_policy)
        temp_hold_combo.setCurrentIndex(temp_hold_idx if temp_hold_idx >= 0 else 0)
        temp_hold_row.addWidget(temp_hold_label)
        temp_hold_row.addWidget(temp_hold_combo, 1)
        outer_layout.addLayout(temp_hold_row)

        layout = QHBoxLayout()
        outer_layout.addLayout(layout, 1)

        tabs = QTabWidget(dialog)
        tables = {}
        for axis, tab_name in (("temp", "温度路径"), ("press", "压力路径")):
            page = QWidget(dialog)
            page_layout = QVBoxLayout(page)
            table = self._make_axis_path_table(axis, page)
            page_layout.addWidget(table)
            tabs.addTab(page, tab_name)
            tables[axis] = table
        layout.addWidget(tabs, 1)

        def refresh_enable_labels():
            temp_enable_btn.setText("温度：开启" if temp_enable_btn.isChecked() else "温度：关闭")
            press_enable_btn.setText("压力：开启" if press_enable_btn.isChecked() else "压力：关闭")
            temp_state = "启用" if temp_enable_btn.isChecked() else "关闭"
            press_state = "启用" if press_enable_btn.isChecked() else "关闭"
            tabs.setTabText(0, f"温度路径（{temp_state}，{tables['temp'].rowCount()}段）")
            tabs.setTabText(1, f"压力路径（{press_state}，{tables['press'].rowCount()}段）")

        temp_enable_btn.toggled.connect(lambda _checked: refresh_enable_labels())
        press_enable_btn.toggled.connect(lambda _checked: refresh_enable_labels())
        refresh_enable_labels()

        button_layout = QVBoxLayout()
        add_btn = QPushButton("新建段")
        del_btn = QPushButton("删除选中段")
        import_axis_btn = QPushButton("导入当前路径")
        export_axis_btn = QPushButton("导出当前路径")
        import_plan_btn = QPushButton("导入温压方案")
        export_plan_btn = QPushButton("导出温压方案")
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        for btn in (add_btn, del_btn, import_axis_btn, export_axis_btn, import_plan_btn, export_plan_btn):
            button_layout.addWidget(btn)
        button_layout.addStretch(1)
        button_layout.addWidget(buttons)
        layout.addLayout(button_layout)

        def current_axis():
            return "temp" if tabs.currentIndex() == 0 else "press"

        def current_table():
            return tables[current_axis()]

        def add_segment():
            axis = current_axis()
            self._add_axis_segment_row(tables[axis], axis)
            refresh_enable_labels()

        def delete_selected():
            axis = current_axis()
            table = tables[axis]
            row = table.currentRow()
            if row >= 0:
                table.removeRow(row)
                self._refresh_axis_start_editability(table)
                self._sync_axis_segment_starts(table)
                refresh_enable_labels()

        def import_axis_path():
            axis = current_axis()
            axis_name, *_ = self._manual_axis_defaults(axis)
            file_path, _ = QFileDialog.getOpenFileName(dialog, f"导入{axis_name}路径", "", self._axis_path_file_filter())
            if not file_path:
                return
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                segments = self._segments_from_axis_path_payload(payload, axis)
                self._build_manual_axis_targets(self._serialize_manual_axis_segments(segments), axis_name)
            except Exception as ex:
                QMessageBox.warning(dialog, "导入路径失败", str(ex))
                return
            self._load_segments_into_axis_table(tables[axis], axis, segments)
            refresh_enable_labels()
            self.append_status(f"[阶段] 已导入{axis_name}路径: {file_path}")

        def export_axis_path():
            axis = current_axis()
            axis_name, value_unit, *_ = self._manual_axis_defaults(axis)
            try:
                segments = self._segments_from_axis_table(tables[axis])
                self._build_manual_axis_targets(self._serialize_manual_axis_segments(segments), axis_name)
            except RuntimeError as ex:
                QMessageBox.warning(dialog, "导出路径失败", str(ex))
                return
            file_path, _ = QFileDialog.getSaveFileName(
                dialog,
                f"导出{axis_name}路径",
                f"{axis_name}路径.json",
                self._axis_path_file_filter(),
            )
            if not file_path:
                return
            if not file_path.lower().endswith(".json"):
                file_path += ".json"
            try:
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(self._axis_path_payload(axis, segments), f, ensure_ascii=False, indent=2)
            except Exception as ex:
                QMessageBox.warning(dialog, "导出路径失败", str(ex))
                return
            targets = self._build_manual_axis_targets(self._serialize_manual_axis_segments(segments), axis_name)
            self.append_status(f"[阶段] 已导出{axis_name}路径: {self._summarize_axis_targets(targets, value_unit)}")

        def import_plan():
            file_path, _ = QFileDialog.getOpenFileName(dialog, "导入温压方案", "", self._axis_path_file_filter())
            if not file_path:
                return
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                if not isinstance(payload, dict) or payload.get("format") != "AutoLifeCalPathPlan":
                    raise RuntimeError("该文件不是温压方案")
                paths = payload.get("paths")
                if not isinstance(paths, dict):
                    raise RuntimeError("温压方案缺少 paths")
                loaded = {
                    axis: self._segments_from_axis_path_payload(paths.get(axis), axis)
                    for axis in ("temp", "press")
                }
                for axis, segments in loaded.items():
                    axis_name, *_ = self._manual_axis_defaults(axis)
                    self._build_manual_axis_targets(self._serialize_manual_axis_segments(segments), axis_name)
            except Exception as ex:
                QMessageBox.warning(dialog, "导入方案失败", str(ex))
                return
            for axis, segments in loaded.items():
                self._load_segments_into_axis_table(tables[axis], axis, segments)
            refresh_enable_labels()
            self.append_status(f"[阶段] 已导入温压方案: {file_path}")

        def export_plan():
            try:
                segments_by_axis = self._axis_tables_to_segments(tables)
            except RuntimeError as ex:
                QMessageBox.warning(dialog, "导出方案失败", str(ex))
                return
            file_path, _ = QFileDialog.getSaveFileName(
                dialog,
                "导出温压方案",
                "温压路径方案.json",
                self._axis_path_file_filter(),
            )
            if not file_path:
                return
            if not file_path.lower().endswith(".json"):
                file_path += ".json"
            try:
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(self._path_plan_payload(segments_by_axis), f, ensure_ascii=False, indent=2)
            except Exception as ex:
                QMessageBox.warning(dialog, "导出方案失败", str(ex))
                return
            self.append_status(f"[阶段] 已导出温压方案: {file_path}")

        add_btn.clicked.connect(add_segment)
        del_btn.clicked.connect(delete_selected)
        import_axis_btn.clicked.connect(import_axis_path)
        export_axis_btn.clicked.connect(export_axis_path)
        import_plan_btn.clicked.connect(import_plan)
        export_plan_btn.clicked.connect(export_plan)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)

        if dialog.exec_() != QDialog.Accepted:
            return

        try:
            segments_by_axis = self._axis_tables_to_segments(tables)
        except RuntimeError as ex:
            QMessageBox.warning(self, "路径格式错误", str(ex))
            return

        self.temp_cal_btn.setChecked(temp_enable_btn.isChecked())
        self.press_cal_btn.setChecked(press_enable_btn.isChecked())
        selected_order = str(order_combo.currentData() or "TEMP_THEN_PRESS")
        if selected_order != current_order:
            self.settings.setValue("stage_control_order_v2", selected_order)
            self.append_status(f"[阶段] 温压执行顺序: {order_combo.currentText()}")
        selected_temp_hold_policy = str(temp_hold_combo.currentData() or "EVERY_PRESS_POINT")
        if selected_temp_hold_policy != current_temp_hold_policy:
            self.settings.setValue("stage_temp_hold_policy_v1", selected_temp_hold_policy)
            self.append_status(f"[阶段] 温稳策略: {temp_hold_combo.currentText()}")

        summaries = []
        for axis, segments in segments_by_axis.items():
            axis_name, value_unit, *_ = self._manual_axis_defaults(axis)
            text = self._serialize_manual_axis_segments(segments)
            edit = self.temp_manual_points_edit if axis == "temp" else self.press_manual_points_edit
            edit.setText(text.strip())
            targets = self._build_manual_axis_targets(text, axis_name)
            summaries.append(f"{axis_name}: {self._summarize_axis_targets(targets, value_unit)}")
        self.append_status("[阶段] 已更新温压路径: " + "；".join(summaries))
        self.persist_custom_axis_settings()
        self.update_path_summary()

    def append_status(self, text: str):
        line = self._append_log(self.status_log_text, text)
        self._write_log_line(self.status_log_file_path, line)

    def append_data(self, text: str):
        line = self._append_log(self.sensor_log_text, text)
        if self.data_log_enabled:
            self._write_log_line(self.sensor_log_file_path, line)

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
        return line

    def _write_log_line(self, file_path: str | None, line: str):
        if not file_path:
            return
        try:
            with open(file_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as ex:
            if not self._log_write_err_reported:
                self._log_write_err_reported = True
                self.status_log_text.append(
                    f"[{time.strftime('%H:%M:%S')}] [日志] 文件写入失败: {ex}"
                )

    def _init_run_log_files(self, save_dir: str):
        sensor_snapshot = self.sensor_log_text.toPlainText().strip()
        status_snapshot = self.status_log_text.toPlainText().strip()
        self.sensor_log_file_path = os.path.join(save_dir, "温压状态日志.log")
        self.status_log_file_path = os.path.join(save_dir, "运行事件日志.log")
        self._log_write_err_reported = False
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        header = f"\n========== 自动寿命标定启动 {stamp} ==========\n"
        snapshots = (
            (self.sensor_log_file_path, "启动前温压状态日志", sensor_snapshot),
            (self.status_log_file_path, "启动前运行事件日志", status_snapshot),
        )
        for file_path, title, snapshot in snapshots:
            with open(file_path, "a", encoding="utf-8") as f:
                f.write(header)
                if snapshot:
                    f.write(f"------ {title} ------\n")
                    f.write(snapshot + "\n")
                    f.write("------ 启动后追加日志 ------\n")

    def _update_channel_button_text(self):
        self.channel_btn.setText("、".join(sorted(self.selected_channels)))

    @staticmethod
    def _collect_csv_files(folder: str):
        if not folder or not os.path.isdir(folder):
            return []
        return [
            os.path.join(folder, fn)
            for fn in sorted(os.listdir(folder))
            if fn.lower().endswith(".csv")
        ]

    def _get_or_create_info_fit_window(self):
        if self.info_fit_window is None:
            from info_app import mainWindow as InfoFitWindow
            # Keep this as a top-level window to avoid inheriting main-window stylesheet.
            self.info_fit_window = InfoFitWindow()
        return self.info_fit_window

    def _prepare_info_fit_context(self, save_dir: str):
        win = self._get_or_create_info_fit_window()
        csv_files = self._collect_csv_files(save_dir)
        win.apply_external_context(work_dir=save_dir, save_dir=save_dir, target_files=csv_files)
        return win, csv_files

    def open_lifetime_fit_settings(self):
        save_dir = self.save_path_edit.text().strip()
        win, csv_files = self._prepare_info_fit_context(save_dir)
        win.show()
        win.raise_()
        win.activateWindow()
        self.append_status(f"[拟合] 已打开寿命拟合设置，CSV文件数量: {len(csv_files)}")

    def load_fit_enable_setting(self):
        enabled = self.settings.value("fit_after_run_enabled", True, type=bool)
        self.fit_enable_btn.setChecked(bool(enabled))

    def load_save_format_setting(self):
        global CURRENT_SCOPE_SAVE_FORMAT
        fmt = normalize_save_format(self.settings.value("scope_save_format", "CSV", type=str))
        CURRENT_SCOPE_SAVE_FORMAT = fmt
        idx = self.save_format_combo.findData(fmt)
        self.save_format_combo.setCurrentIndex(idx if idx >= 0 else 0)

    def get_save_format(self) -> str:
        return normalize_save_format(self.save_format_combo.currentData() or CURRENT_SCOPE_SAVE_FORMAT)

    def on_save_format_changed(self):
        global CURRENT_SCOPE_SAVE_FORMAT
        fmt = self.get_save_format()
        CURRENT_SCOPE_SAVE_FORMAT = fmt
        self.settings.setValue("scope_save_format", fmt)
        if getattr(self, "_ui_init_done", False):
            self.append_status(f"[示波器] 保存格式: {fmt}")

    def load_run_settings(self):
        stable_time = self.settings.value("run_stable_time_s", self.stable_time_spin.value(), type=int)
        cycles = self.settings.value("run_cycles", self.cycles_spin.value(), type=int)
        avg_count = self.settings.value("scope_avg_count", self.get_avg_count(), type=int)
        self.stable_time_spin.setValue(max(self.stable_time_spin.minimum(), min(self.stable_time_spin.maximum(), int(stable_time))))
        self.cycles_spin.setValue(max(self.cycles_spin.minimum(), min(self.cycles_spin.maximum(), int(cycles))))
        idx = self.avg_count_combo.findData(int(avg_count))
        if idx >= 0:
            self.avg_count_combo.setCurrentIndex(idx)

    def persist_run_settings(self):
        self.settings.setValue("run_stable_time_s", int(self.stable_time_spin.value()))
        self.settings.setValue("run_cycles", int(self.cycles_spin.value()))
        self.settings.setValue("scope_avg_count", int(self.get_avg_count()))

    def on_run_setting_changed(self, *_):
        self.persist_run_settings()

    def load_custom_axis_settings(self):
        self.temp_cal_btn.setChecked(self.settings.value("temp_axis_enabled", True, type=bool))
        self.press_cal_btn.setChecked(self.settings.value("press_axis_enabled", False, type=bool))
        temp_path = self.settings.value("temp_manual_path", "", type=str)
        press_path = self.settings.value("press_manual_path", "", type=str)
        if not temp_path.strip():
            temp_path = self._serialize_manual_axis_segments(self._default_axis_segments("temp"))
        if not press_path.strip():
            press_path = self._serialize_manual_axis_segments(self._default_axis_segments("press"))
        self.temp_manual_points_edit.setText(temp_path)
        self.press_manual_points_edit.setText(press_path)
        self.update_path_summary()

    def persist_custom_axis_settings(self):
        self.settings.setValue("temp_axis_enabled", bool(self.temp_cal_btn.isChecked()))
        self.settings.setValue("press_axis_enabled", bool(self.press_cal_btn.isChecked()))
        self.settings.setValue("temp_manual_path", self.temp_manual_points_edit.text().strip())
        self.settings.setValue("press_manual_path", self.press_manual_points_edit.text().strip())

    def on_fit_enable_toggled(self, checked: bool):
        self.settings.setValue("fit_after_run_enabled", bool(checked))
        if self._ui_init_done:
            state = "开启" if checked else "关闭"
            self.append_status(f"[拟合] 自动后处理: {state}")

    def load_saved_path(self):
        saved = self.settings.value("save_dir", "", type=str)
        if saved and os.path.isdir(saved):
            self.save_path_edit.setText(saved)

    def load_port_settings(self):
        saved_scope = self.settings.value("scope_com_port", "", type=str).strip()
        saved_scope_resource = self.settings.value("scope_manual_resource", "", type=str).strip()
        saved_scope_connection_mode = self.settings.value("scope_connection_mode", "USB", type=str).strip().upper()
        saved_hotplate = self.settings.value("hotplate_com_port", "", type=str).strip()
        saved_siggen = self.settings.value("siggen_com_port", "", type=str).strip()
        saved_siggen_channels = self.settings.value("siggen_channels", "CH1", type=str).strip()
        self.spectrum_integration_ms = int(self.settings.value("spectrum_integration_ms", 100, type=int))
        self.spectrum_average_count = int(self.settings.value("spectrum_average_count", 1, type=int))
        self.siggen_lifetime_freq_hz = float(
            self.settings.value("siggen_lifetime_freq_hz", SIGGEN_LIFETIME_FREQ_HZ, type=float)
        )
        if saved_scope:
            self.scope_com_port = saved_scope
        self.scope_manual_resource = saved_scope_resource
        if saved_scope_connection_mode in ("USB", "TCPIP", "AUTO"):
            self.scope_connection_mode = saved_scope_connection_mode
        if saved_hotplate:
            self.hotplate_com_port = saved_hotplate
        if saved_siggen:
            self.siggen_com_port = saved_siggen
        channels = {ch for ch in saved_siggen_channels.split(",") if ch in {"CH1", "CH2"}}
        self.siggen_channels = channels or {"CH1"}
        self.update_settings_tooltip()

    def persist_port_settings(self):
        scope_port = (self.scope_com_port or "").strip()
        hotplate_port = (self.hotplate_com_port or "").strip()
        siggen_port = (self.siggen_com_port or "").strip()
        if scope_port:
            self.settings.setValue("scope_com_port", scope_port)
        else:
            self.settings.remove("scope_com_port")
        scope_resource = (self.scope_manual_resource or "").strip()
        if scope_resource:
            self.settings.setValue("scope_manual_resource", scope_resource)
        else:
            self.settings.remove("scope_manual_resource")
        self.settings.setValue("scope_connection_mode", self.scope_connection_mode)
        if hotplate_port:
            self.settings.setValue("hotplate_com_port", hotplate_port)
        else:
            self.settings.remove("hotplate_com_port")
        if siggen_port:
            self.settings.setValue("siggen_com_port", siggen_port)
        else:
            self.settings.remove("siggen_com_port")
        channels_text = ",".join(sorted(ch for ch in self.siggen_channels if ch in {"CH1", "CH2"}))
        self.settings.setValue("siggen_channels", channels_text or "CH1")
        self.settings.setValue("spectrum_integration_ms", int(self.spectrum_integration_ms))
        self.settings.setValue("spectrum_average_count", int(self.spectrum_average_count))
        self.settings.setValue("siggen_lifetime_freq_hz", float(self.siggen_lifetime_freq_hz))

    def persist_save_path(self):
        save_dir = self.save_path_edit.text().strip()
        if save_dir and os.path.isdir(save_dir):
            self.settings.setValue("save_dir", save_dir)
        else:
            self.settings.remove("save_dir")

    def initialize_system(self):
        if TEST_MODE:
            self._auto_initialize_for_start()
            self.append_status("[测试模式] 初始化已模拟完成")
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("初始化设置")
        dialog.setMinimumWidth(420)

        layout = QVBoxLayout(dialog)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)

        grid.addWidget(QLabel("示波器连接方式:"), 0, 0)
        scope_mode_combo = QComboBox()
        scope_mode_combo.addItem("USB", "USB")
        scope_mode_combo.addItem("网口/IP", "TCPIP")
        scope_mode_combo.addItem("自动", "AUTO")
        mode_idx = scope_mode_combo.findData(self.scope_connection_mode)
        scope_mode_combo.setCurrentIndex(mode_idx if mode_idx >= 0 else 0)
        grid.addWidget(scope_mode_combo, 0, 1)

        grid.addWidget(QLabel("示波器 IP / VISA:"), 1, 0)
        scope_resource_edit = QLineEdit()
        scope_resource_edit.setPlaceholderText("网口模式输入 10.5.5.88 / TCPIP0::...::INSTR")
        scope_resource_edit.setText(self.scope_manual_resource)
        grid.addWidget(scope_resource_edit, 1, 1)

        def sync_scope_resource_enabled():
            scope_resource_edit.setEnabled(scope_mode_combo.currentData() != "USB")

        scope_mode_combo.currentIndexChanged.connect(sync_scope_resource_enabled)
        sync_scope_resource_enabled()

        grid.addWidget(QLabel("信号发生器频率 (Hz):"), 2, 0)
        siggen_freq_spin = QSpinBox()
        siggen_freq_spin.setRange(1, 1_000_000)
        siggen_freq_spin.setButtonSymbols(QSpinBox.NoButtons)
        siggen_freq_spin.setValue(int(round(float(self.siggen_lifetime_freq_hz))))
        grid.addWidget(siggen_freq_spin, 2, 1)

        grid.addWidget(QLabel("标定台 COM口:"), 3, 0)
        hotplate_combo = QComboBox()
        grid.addWidget(hotplate_combo, 3, 1)

        grid.addWidget(QLabel("信号发生器 COM口:"), 4, 0)
        siggen_combo = QComboBox()
        grid.addWidget(siggen_combo, 4, 1)

        grid.addWidget(QLabel("信号发生器输出通道:"), 5, 0)
        siggen_ch_row = QVBoxLayout()
        siggen_ch1 = QCheckBox("CH1")
        siggen_ch2 = QCheckBox("CH2")
        siggen_ch_row.addWidget(siggen_ch1)
        siggen_ch_row.addWidget(siggen_ch2)
        grid.addLayout(siggen_ch_row, 5, 1)

        grid.addWidget(QLabel("光谱仪:"), 6, 0)
        spectrometer_status_label = QLabel("未检测")
        grid.addWidget(spectrometer_status_label, 6, 1)

        grid.addWidget(QLabel("光谱积分时间 (ms):"), 7, 0)
        spectrum_integration_spin = QSpinBox()
        spectrum_integration_spin.setRange(1, 600000)
        spectrum_integration_spin.setButtonSymbols(QSpinBox.NoButtons)
        spectrum_integration_spin.setValue(int(self.spectrum_integration_ms))
        grid.addWidget(spectrum_integration_spin, 7, 1)

        grid.addWidget(QLabel("光谱平均次数:"), 8, 0)
        spectrum_average_spin = QSpinBox()
        spectrum_average_spin.setRange(1, 100)
        spectrum_average_spin.setButtonSymbols(QSpinBox.NoButtons)
        spectrum_average_spin.setValue(int(self.spectrum_average_count))
        grid.addWidget(spectrum_average_spin, 8, 1)

        status_label = QLabel("USB 模式只尝试 USB VISA；网口模式只尝试 TCPIP/IP")
        grid.addWidget(status_label, 9, 0, 1, 2)
        layout.addLayout(grid)

        tools_layout = QGridLayout()
        refresh_btn = QPushButton("刷新端口")
        auto_btn = QPushButton("自动检测")
        tools_layout.addWidget(refresh_btn, 0, 0)
        tools_layout.addWidget(auto_btn, 0, 1)
        layout.addLayout(tools_layout)

        def fill_port_combos():
            hotplate_combo.clear()
            siggen_combo.clear()
            for label, value in self.get_available_com_ports():
                hotplate_combo.addItem(label, value)
                siggen_combo.addItem(label, value)

            def ensure_current(combo, current_value):
                idx = combo.findData(current_value)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                elif current_value:
                    combo.addItem(f"{current_value} (当前)", current_value)
                    combo.setCurrentIndex(combo.count() - 1)

            ensure_current(hotplate_combo, self.hotplate_com_port)
            ensure_current(siggen_combo, self.siggen_com_port)
            siggen_ch1.setChecked("CH1" in self.siggen_channels)
            siggen_ch2.setChecked("CH2" in self.siggen_channels)
            spectrometer_status_label.setText("点击自动检测以刷新")

        def on_detect_done(result):
            auto_btn.setEnabled(True)
            hotplate_val = result.get("hotplate_com")
            siggen_val = result.get("siggen_com")
            if hotplate_val:
                idx = hotplate_combo.findData(hotplate_val)
                if idx < 0:
                    hotplate_combo.addItem(f"{hotplate_val} (自动检测)", hotplate_val)
                    idx = hotplate_combo.count() - 1
                hotplate_combo.setCurrentIndex(idx)
            if siggen_val:
                idx = siggen_combo.findData(siggen_val)
                if idx < 0:
                    siggen_combo.addItem(f"{siggen_val} (自动检测)", siggen_val)
                    idx = siggen_combo.count() - 1
                siggen_combo.setCurrentIndex(idx)
            spec_ok = bool(result.get("spectrometer_ok"))
            spec_sn = result.get("spectrometer_sn", "未检测")
            spectrometer_status_label.setText(f"{spec_sn}，连接={'是' if spec_ok else '否'}")
            status_label.setText(result.get("message", "自动检测完成"))

        def start_auto_detect():
            if self.detect_thread and self.detect_thread.is_alive():
                status_label.setText("自动检测仍在进行，请稍候...")
                return
            if serial is None:
                QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先安装: pip install pyserial")
                return
            auto_btn.setEnabled(False)
            status_label.setText("正在自动检测串口设备和光谱仪...")

            def worker():
                try:
                    result = self._scan_serial_devices()
                except Exception as ex:
                    result = {
                        "scope_com": None,
                        "hotplate_com": None,
                        "siggen_com": None,
                        "spectrometer_ok": False,
                        "spectrometer_sn": "未检测",
                        "message": f"自动检测失败: {ex}",
                    }
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

        self.scope_connection_mode = scope_mode_combo.currentData() or "USB"
        self.scope_manual_resource = scope_resource_edit.text().strip()
        self.hotplate_com_port = hotplate_combo.currentData() or self.hotplate_com_port
        self.siggen_com_port = siggen_combo.currentData() or self.siggen_com_port
        self.spectrum_integration_ms = int(spectrum_integration_spin.value())
        self.spectrum_average_count = int(spectrum_average_spin.value())
        self.siggen_lifetime_freq_hz = float(siggen_freq_spin.value())
        channels = set()
        if siggen_ch1.isChecked():
            channels.add("CH1")
        if siggen_ch2.isChecked():
            channels.add("CH2")
        self.siggen_channels = channels
        self.persist_port_settings()
        self.update_settings_tooltip()
        hot_ok, hot_sn = self.get_hotplate_identity_status()
        sig_ok, sig_sn = self.get_siggen_identity_status()
        spec_ok, spec_sn = self.get_spectrometer_identity_status()
        needs_scope, needs_spectrometer = self._current_mode_device_requirements()
        scope_ok, scope_sn = False, "未检测"
        if needs_scope:
            resume_live, resume_plot = self._suspend_hotplate_live_poll()
            try:
                scope_ok, scope_sn = self.try_lock_scope(show_error=False, verbose=False)
            finally:
                self._resume_hotplate_live_poll(resume_live, resume_plot)
        if hot_ok:
            self.start_hotplate_live_poll()
        self.append_status(
            f"[初始化] 标定台: 端口={self.hotplate_com_port}, SN={hot_sn}, 连接={'是' if hot_ok else '否'}"
        )
        if needs_scope:
            self.append_status(
                f"[初始化] 示波器: 方式={self.scope_connection_mode}, 资源={self.resource_str or self.scope_manual_resource or '自动'}, SN={scope_sn}, 连接={'是' if scope_ok else '否'}"
            )
        if needs_spectrometer:
            self.append_status(f"[初始化] 光谱仪: SN={spec_sn}, 连接={'是' if spec_ok else '否'}")
        self.append_status(
            f"[初始化] 信号发生器: 端口={self.siggen_com_port or '未配置'}, 状态={sig_sn}, 连接={'是' if sig_ok else '否'}, 通道={','.join(sorted(self.siggen_channels)) or '无'}"
        )

    def _auto_initialize_for_start(self):
        if TEST_MODE:
            self.resource_str = "TEST::SCOPE"
            self.idn = "TEST,TEST_SCOPE,TESTMODE,0"
            self.profile_name = "DPO2024B"
            self.hotplate_com_port = self.hotplate_com_port or "TEST_HOTPLATE"
            self.siggen_com_port = self.siggen_com_port or "TEST_SIGGEN"
            self.start_hotplate_live_poll()
            self.append_status("[测试模式] 自动初始化已跳过真实硬件连接")
            return True, True, True, True

        self.persist_port_settings()
        self.update_settings_tooltip()

        needs_scope, needs_spectrometer = self._current_mode_device_requirements()
        hot_ok, hot_sn = self.get_hotplate_identity_status()
        sig_ok, sig_sn = self.get_siggen_identity_status()
        spec_ok, spec_sn = self.get_spectrometer_identity_status() if needs_spectrometer else (False, "未检测")
        scope_ok, scope_sn = False, "未检测"
        if needs_scope:
            resume_live, resume_plot = self._suspend_hotplate_live_poll()
            try:
                scope_ok, scope_sn = self.try_lock_scope(show_error=False, verbose=False)
            finally:
                self._resume_hotplate_live_poll(resume_live, resume_plot)
        if hot_ok:
            self.start_hotplate_live_poll()

        self.append_status(
            f"[自动初始化] 标定台: 端口={self.hotplate_com_port}, SN={hot_sn}, 连接={'是' if hot_ok else '否'}"
        )
        if needs_scope:
            self.append_status(
                f"[自动初始化] 示波器: 方式={self.scope_connection_mode}, 资源={self.resource_str or self.scope_manual_resource or '自动'}, SN={scope_sn}, 连接={'是' if scope_ok else '否'}"
            )
        if needs_spectrometer:
            self.append_status(f"[自动初始化] 光谱仪: SN={spec_sn}, 连接={'是' if spec_ok else '否'}")
        self.append_status(
            f"[自动初始化] 信号发生器: 端口={self.siggen_com_port or '未配置'}, 状态={sig_sn}, 连接={'是' if sig_ok else '否'}, 通道={','.join(sorted(self.siggen_channels)) or '无'}"
        )
        return scope_ok, hot_ok, sig_ok, spec_ok

    def _suspend_hotplate_live_poll(self):
        live_active = self.hotplate_live_timer.isActive()
        plot_active = self.hotplate_plot_timer.isActive()
        if live_active:
            self.hotplate_live_timer.stop()
        if plot_active:
            self.hotplate_plot_timer.stop()
        return live_active, plot_active

    def _resume_hotplate_live_poll(self, live_active: bool, plot_active: bool):
        if live_active and not self.hotplate_live_timer.isActive():
            self.hotplate_live_timer.start()
        if plot_active and not self.hotplate_plot_timer.isActive():
            self.last_sim_ts = time.time()
            self.hotplate_plot_timer.start()

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

    def _probe_open_serial(self, ser, cmd: str, timeout_s: float = SERIAL_PROBE_TIMEOUT_S) -> str:
        try:
            ser.reset_input_buffer()
            ser.write((cmd + "\r").encode("ascii"))
            ser.flush()
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                n = ser.in_waiting
                if n > 0:
                    return ser.read(n).decode("ascii", errors="ignore").strip("\x00\r\n ")
                time.sleep(0.005)
        except Exception:
            return ""
        return ""

    def _probe_serial_port_for_devices(self, port: str) -> dict:
        result = {"scope": False, "hotplate": False, "siggen": False}
        baud_candidates = list(dict.fromkeys([SIGGEN_BAUD, *HOTPLATE_BAUD_CANDIDATES]))
        for baud in baud_candidates:
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
                idn = self._probe_open_serial(ser, "*IDN?")
                idn_l = idn.lower()
                if "tektronix" in idn_l or SERIAL_LOCK.lower() in idn_l:
                    result["scope"] = True
                if "instec,mk2000b" in idn_l:
                    result["hotplate"] = True

                if not result["hotplate"]:
                    rt = self._probe_open_serial(ser, "TEMPerature:RTINformation?")
                    if "MK:" in rt.upper():
                        result["hotplate"] = True

                if baud == SIGGEN_BAUD and not result["siggen"]:
                    rmn = self._probe_open_serial(ser, "RMN")
                    if re.fullmatch(r"\d+", (rmn or "").strip()):
                        result["siggen"] = True
            except Exception:
                pass
            finally:
                try:
                    if ser and ser.is_open:
                        ser.close()
                except Exception:
                    pass
            if result["scope"] and result["hotplate"] and result["siggen"]:
                break
        return result

    def _scan_serial_devices(self):
        if serial is None:
            return {"scope_com": None, "hotplate_com": None, "siggen_com": None, "spectrometer_ok": False, "spectrometer_sn": "未检测", "message": "未安装pyserial"}
        try:
            ports = [p.device for p in list_ports.comports()] if list_ports else []
        except Exception as ex:
            return {"scope_com": None, "hotplate_com": None, "siggen_com": None, "spectrometer_ok": False, "spectrometer_sn": "未检测", "message": f"串口枚举失败: {ex}"}

        scope_com = None
        hotplate_com = None
        siggen_com = None
        for port in ports:
            probe = self._probe_serial_port_for_devices(port)
            if probe["scope"] and scope_com is None:
                scope_com = port
            if probe["hotplate"] and hotplate_com is None:
                hotplate_com = port
            if probe["siggen"] and siggen_com is None:
                siggen_com = port
            if scope_com and hotplate_com and siggen_com:
                break

        spec_ok, spec_sn = self.get_spectrometer_identity_status()

        msg = (
            f"自动检测完成: 示波器={scope_com or '未找到'}, 标定台={hotplate_com or '未找到'}, "
            f"信号发生器={siggen_com or '未找到'}, 光谱仪={spec_sn if spec_ok else '未找到'}"
        )
        return {
            "scope_com": scope_com,
            "hotplate_com": hotplate_com,
            "siggen_com": siggen_com,
            "spectrometer_ok": spec_ok,
            "spectrometer_sn": spec_sn,
            "message": msg,
        }

    def _query_siggen(self, command: str):
        if TEST_MODE:
            return "1"
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.siggen_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置信号发生器 COM 口，请先在初始化设置中配置")
        ser = None
        try:
            ser = serial.Serial(
                port,
                baudrate=SIGGEN_BAUD,
                timeout=0.3,
                write_timeout=0.4,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            return ser.readline().decode("ascii", errors="ignore").strip("\x00\r\n ")
        finally:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

    def _send_siggen_commands(self, commands):
        if TEST_MODE:
            self.append_status(f"[测试模式][信号发生器] 跳过真实命令: {', '.join(commands)}")
            return
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.siggen_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置信号发生器 COM 口，请先在初始化设置中配置")
        ser = None
        try:
            ser = serial.Serial(
                port,
                baudrate=SIGGEN_BAUD,
                timeout=0.3,
                write_timeout=0.4,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            for cmd in commands:
                ser.write((cmd + "\n").encode("ascii"))
                ser.flush()
                time.sleep(SIGGEN_COMMAND_DELAY_S)
        finally:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

    def _set_siggen_output(self, enable: bool):
        channels = [ch for ch in sorted(self.siggen_channels) if ch in {"CH1", "CH2"}]
        if not channels:
            return
        cmds = []
        for ch in channels:
            if ch == "CH1":
                cmds.append("WMN1" if enable else "WMN0")
            elif ch == "CH2":
                cmds.append("WFN1" if enable else "WFN0")
        if not cmds:
            return
        self._send_siggen_commands(cmds)
        state = "ON" if enable else "OFF"
        self.append_status(f"[信号发生器] 通道 {','.join(channels)} 输出命令已发送 -> {state}")

    @staticmethod
    def _fy6900_frequency_code(hz: float) -> str:
        # This FY6900 firmware displays WMF/WFF arguments as integer Hz.
        return f"{int(round(float(hz))):014d}"

    @staticmethod
    def _fy6900_channel_commands(ch: str, waveform: int, frequency_hz: float, amplitude_vpp: float, offset_v: float, duty_percent: float | None = None):
        if ch == "CH1":
            prefix = {"wave": "WMW", "freq": "WMF", "amp": "WMA", "offset": "WMO", "duty": "WMD", "out": "WMN"}
        elif ch == "CH2":
            prefix = {"wave": "WFW", "freq": "WFF", "amp": "WFA", "offset": "WFO", "duty": "WFD", "out": "WFN"}
        else:
            return []

        cmds = [
            f"{prefix['out']}0",
            f"{prefix['wave']}{int(waveform)}",
            f"{prefix['freq']}{MainWindow._fy6900_frequency_code(frequency_hz)}",
            f"{prefix['amp']}{float(amplitude_vpp):.3f}",
            f"{prefix['offset']}{float(offset_v):.3f}",
        ]
        if duty_percent is not None:
            cmds.append(f"{prefix['duty']}{float(duty_percent):.1f}")
        cmds.append(f"{prefix['out']}1")
        return cmds

    def _configure_siggen_lifetime_light(self):
        channels = [ch for ch in sorted(self.siggen_channels) if ch in {"CH1", "CH2"}]
        if not channels:
            self.append_status("[信号发生器] 未选择输出通道，跳过寿命方波设置")
            return
        cmds = []
        for ch in channels:
            cmds.extend(
                self._fy6900_channel_commands(
                    ch,
                    waveform=1,
                    frequency_hz=float(self.siggen_lifetime_freq_hz),
                    amplitude_vpp=SIGGEN_LIFETIME_AMPLITUDE_VPP,
                    offset_v=SIGGEN_LIFETIME_OFFSET_V,
                    duty_percent=SIGGEN_LIFETIME_DUTY_PERCENT,
                )
            )
        self._send_siggen_commands(cmds)
        self.append_status(
            f"[信号发生器] 已设置寿命光源: 方波 {float(self.siggen_lifetime_freq_hz):g} Hz, "
            "5 Vpp, Offset 2.5 V"
        )

    def _configure_siggen_spectrum_light(self):
        channels = [ch for ch in sorted(self.siggen_channels) if ch in {"CH1", "CH2"}]
        if not channels:
            self.append_status("[信号发生器] 未选择输出通道，跳过光谱直流设置")
            return
        cmds = []
        for ch in channels:
            cmds.extend(
                self._fy6900_channel_commands(
                    ch,
                    waveform=6 if ch == "CH1" else 5,
                    frequency_hz=1.0,
                    amplitude_vpp=0.0,
                    offset_v=SIGGEN_SPECTRUM_DC_V,
                )
            )
        self._send_siggen_commands(cmds)
        self.append_status("[信号发生器] 已设置光谱光源: DC 5 V")

    def get_siggen_identity_status(self):
        if TEST_MODE:
            return True, "TEST_SIGGEN"
        if not (self.siggen_com_port or "").strip():
            return False, "未配置"
        try:
            rmn = self._query_siggen("RMN")
            rfn = self._query_siggen("RFN")
            if re.fullmatch(r"\d+", (rmn or "").strip()) and re.fullmatch(r"\d+", (rfn or "").strip()):
                return True, f"CH1={rmn}, CH2={rfn}"
            return False, "无响应"
        except Exception as ex:
            return False, f"失败: {str(ex)[:32]}"

    def wait_light_source_stable(self, next_step_text: str = "开始示波器保存"):
        wait_s = int(self.stable_time_spin.value())
        if TEST_MODE:
            wait_s = min(wait_s, int(TEST_MAX_LIGHT_STABLE_S))
        if wait_s <= 0:
            return

        siggen_enabled = bool(self.siggen_com_port.strip() and self.siggen_channels)
        if siggen_enabled:
            self._set_siggen_output(True)
        else:
            self.append_status("[光源] 未配置信号发生器或未选择输出通道，仅执行稳定等待")

        self.append_status(f"[光源] 稳定等待启动: {wait_s}s")
        start = time.time()
        last_log_sec = -1
        while True:
            paused_s = self._wait_while_paused()
            if paused_s:
                start += paused_s
            if self.stop_requested.is_set():
                raise TaskStoppedError("用户已停止任务")
            elapsed = int(time.time() - start)
            if elapsed != last_log_sec:
                last_log_sec = elapsed
                self.append_data(f"光源稳定进度={min(elapsed, wait_s)}/{wait_s}s")
            if elapsed >= wait_s:
                break
            QApplication.processEvents()
            time.sleep(0.2)
        self.append_status(f"[光源] 稳定等待完成，{next_step_text}")

    def start_hotplate_live_poll(self):
        if serial is None and not TEST_MODE:
            return
        if not TEST_MODE and not (self.hotplate_com_port or "").strip():
            return
        if not self.hotplate_live_timer.isActive():
            self.hotplate_live_timer.start()
        if not self.hotplate_plot_timer.isActive():
            self.last_sim_ts = time.time()
            self.hotplate_plot_timer.start()

    def _tick_hotplate_plot(self):
        now = time.time()
        dt = max(0.0, now - self.last_sim_ts)
        self.last_sim_ts = now
        if TEST_MODE:
            dt *= TEST_SIM_SPEEDUP

        if self.sim_temp_mode == "ramp":
            step = (self.sim_rate_cpm / 60.0) * dt
            if self.sim_tf_current < self.sim_tf_target:
                self.sim_tf_current = min(self.sim_tf_current + step, self.sim_tf_target)
            else:
                self.sim_tf_current = max(self.sim_tf_current - step, self.sim_tf_target)
            if abs(self.sim_tf_current - self.sim_tf_target) < 1e-6:
                self.sim_temp_mode = "hold"

        if self.sim_press_mode == "ramp":
            pstep = (self.sim_prate_kpm / 60.0) * dt
            if self.sim_pf_current < self.sim_pf_target:
                self.sim_pf_current = min(self.sim_pf_current + pstep, self.sim_pf_target)
            else:
                self.sim_pf_current = max(self.sim_pf_current - pstep, self.sim_pf_target)
            if abs(self.sim_pf_current - self.sim_pf_target) < 1e-6:
                self.sim_press_mode = "hold"

        if self.last_live_tc is None or self.last_live_pk is None:
            return
        self.push_hotplate_plot(
            self.last_live_tc,
            self.last_live_pk,
            self.sim_tf_current,
            self.sim_pf_current,
        )
 
    def get_hotplate_identity_status(self):
        if TEST_MODE:
            return True, "TEST_HOTPLATE"
        if not (self.hotplate_com_port or "").strip():
            return False, "未配置"
        try:
            idn, baud = self._query_hotplate("*IDN?")
            if idn and "instec,mk2000b" in idn.lower():
                parts = [x.strip() for x in idn.split(",")]
                sn = parts[2] if len(parts) > 2 and parts[2] else idn
                return True, sn
            if idn:
                parts = [x.strip() for x in idn.split(",")]
                sn = parts[2] if len(parts) > 2 and parts[2] else idn
                return True, sn
            else:
                rt, _ = self._query_hotplate("TEMPerature:RTINformation?")
                if "MK:" in rt.upper():
                    return True, "RTINformation可读"
                return False, "未识别"
        except Exception as ex:
            return False, f"失败: {str(ex)[:32]}"

    def poll_hotplate_live_data(self):
        if not self.data_log_enabled:
            return
        if not TEST_MODE and not (self.hotplate_com_port or "").strip():
            return
        try:
            rt_resp, _ = self._query_hotplate("TEMPerature:RTINformation?")
            tc, pk, _, _, _, _ = self._extract_rt_channels(rt_resp, self.control_slave)
            if tc is None or pk is None:
                ftc, fpk = self._extract_temp_pressure_fallback(rt_resp)
                if tc is None:
                    tc = ftc
                if pk is None:
                    pk = fpk
            if tc is None and pk is None:
                return

            if tc is not None and self.last_live_tc is None and self.sim_temp_mode == "hold":
                self.sim_tf_current = tc
                self.sim_tf_target = tc
            if pk is not None and self.last_live_pk is None and self.sim_press_mode == "hold":
                self.sim_pf_current = pk
                self.sim_pf_target = pk

            # Avoid writing exactly duplicated lines each second when values stay unchanged.
            if (
                self.last_live_tc is not None and self.last_live_pk is not None and
                tc is not None and pk is not None and
                abs(tc - self.last_live_tc) < 1e-6 and abs(pk - self.last_live_pk) < 1e-6
            ):
                return

            self.last_live_tc = tc
            self.last_live_pk = pk
            tc_txt = f"{tc:.2f}" if tc is not None else "NA"
            pk_txt = f"{pk:.3f}" if pk is not None else "NA"
            self.append_data(f"{tc_txt} °C, {pk_txt} kPa")
            self._update_live_stability_state(tc, pk)
            self.hotplate_live_err_suppressed = False
        except Exception as ex:
            if not self.hotplate_live_err_suppressed:
                self.append_status(f"[标定台] 实时日志采集失败，后续将抑制重复错误: {ex}")
                self.hotplate_live_err_suppressed = True

    def _update_live_stability_state(self, tc, pk):
        need_temp = self.temp_cal_btn.isChecked()
        need_press = self.press_cal_btn.isChecked()
        now = time.time()

        temp_target = float(self.ref_temp_spin.value())
        press_target = float(self.ref_press_spin.value())
        temp_tol = float(DEFAULT_TEMP_TOL if self.current_temp_tol is None else self.current_temp_tol)
        press_tol = float(DEFAULT_PRESS_TOL if self.current_press_tol is None else self.current_press_tol)
        temp_hold_s = float(DEFAULT_TEMP_HOLD_S if self.current_temp_hold_s is None else self.current_temp_hold_s)
        press_hold_s = float(DEFAULT_PRESS_HOLD_S if self.current_press_hold_s is None else self.current_press_hold_s)

        prev_temp_stable = self.live_temp_stable
        prev_press_stable = self.live_press_stable

        if need_temp and tc is not None and abs(tc - temp_target) <= temp_tol:
            if self.live_temp_since is None:
                self.live_temp_since = now
        else:
            self.live_temp_since = None

        if need_press and pk is not None and abs(pk - press_target) <= press_tol:
            if self.live_press_since is None:
                self.live_press_since = now
        else:
            self.live_press_since = None

        self.live_temp_stable = (not need_temp) or (temp_hold_s <= 0) or (
            self.live_temp_since is not None and (now - self.live_temp_since) >= temp_hold_s
        )
        self.live_press_stable = (not need_press) or (press_hold_s <= 0) or (
            self.live_press_since is not None and (now - self.live_press_since) >= press_hold_s
        )

        if need_temp and self.live_temp_stable != prev_temp_stable:
            if self.live_temp_stable:
                self.append_data("温度已稳定")
            else:
                self.append_data("温度离开稳定区")
        if need_press and self.live_press_stable != prev_press_stable:
            if self.live_press_stable:
                self.append_data("压力已稳定")
            else:
                self.append_data("压力离开稳定区")

        if (need_temp or need_press) and (self.live_temp_stable and self.live_press_stable):
            if not (prev_temp_stable and prev_press_stable):
                self.append_status("[监测] 温度、压力已稳定（实时监测）")

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
        if self._ui_init_done:
            self.apply_scope_settings_now(reason="平均开关切换")

    def on_capture_mode_toggled(self, checked: bool):
        if not self._ui_init_done:
            return
        mode = "Single" if checked else "Continuous"
        self.append_status(f"[示波器] 采样模式切换: {mode}")
        self.apply_scope_settings_now(reason="采样模式切换")

    def on_avg_count_changed(self, _index: int):
        if not self._ui_init_done:
            self.persist_run_settings()
            return
        self.persist_run_settings()
        if not self.avg_checkbox.isChecked():
            return
        self.append_status(f"[示波器] 平均次数切换: {self.get_avg_count()}")
        self.apply_scope_settings_now(reason="平均次数切换")

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
        profile = scope.profile
        return {
            "mode": normalize_acq_mode(_q(scope, profile.acquire_mode_query)),
            "state": normalize_acq_state(_q(scope, profile.acquire_state_query)),
            "numavg": int(float(extract_last_value(_q(scope, profile.acquire_numavg_query)))),
            "numacq": int(float(extract_last_value(_q(scope, profile.acquire_numacq_query)))),
            "stopafter": normalize_stopafter(_q(scope, profile.acquire_stopafter_query)),
            "trig_state": extract_last_value(_q(scope, profile.trigger_state_query)).upper(),
        }

    def apply_acquire_settings(self, scope):
        avg_enabled = self.avg_checkbox.isChecked()
        numavg = self.get_avg_count()
        capture_mode = self.get_capture_mode()
        profile = scope.profile

        _w(scope, profile.stopafter_run_cmd)
        if avg_enabled:
            _w(scope, profile.acquire_mode_set_cmd.format(mode="AVERAGE"))
            _w(scope, profile.acquire_numavg_set_cmd.format(numavg=numavg))
            self.append_status(f"[示波器] {profile.name} Acquire模式: AVERAGE, 平均次数: {numavg}")
        else:
            _w(scope, profile.acquire_mode_set_cmd.format(mode="SAMPLE"))
            self.append_status(f"[示波器] {profile.name} Acquire模式: SAMPLE")
        self.append_status(f"[示波器] 采样模式: {capture_mode}")

    def apply_scope_settings_now(self, reason: str = "手动设置") -> bool:
        if not self.resource_str:
            return False
        rm = scope = None
        try:
            rm, scope, _ = open_scope(self.resource_str)
            self.apply_acquire_settings(scope)
            self.append_status(f"[示波器] 参数已立即下发（{reason}）")
            return True
        except Exception as ex:
            self.append_status(f"[示波器] 参数立即下发失败（{reason}）: {ex}")
            return False
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

    def _set_run_button_state(self, text: str):
        self.start_btn.blockSignals(True)
        try:
            self.start_btn.setChecked(False)
            self.start_btn.setText(text)
        finally:
            self.start_btn.blockSignals(False)

    def on_run_pause_clicked(self):
        if not self.task_running:
            self.do_save()
            return
        if self.stop_requested.is_set():
            self.append_status("[任务] 正在停止，请等待当前步骤结束")
            return

        if self.pause_requested.is_set():
            self.pause_requested.clear()
            self._set_run_button_state("暂停")
            self.append_status("[任务] 已继续")
        else:
            self.pause_requested.set()
            self._set_run_button_state("继续")
            self.append_status("[任务] 已请求暂停，正在等待当前安全点...")

    def _wait_while_paused(self):
        paused_at = None
        while self.pause_requested.is_set():
            if self.stop_requested.is_set():
                self.pause_requested.clear()
                raise TaskStoppedError("用户已停止任务")
            if paused_at is None:
                paused_at = time.time()
                self.append_status("[任务] 已暂停")
            QApplication.processEvents()
            time.sleep(0.1)
        if paused_at is None:
            return 0.0
        return time.time() - paused_at

    def _check_task_control(self):
        if self.stop_requested.is_set():
            raise TaskStoppedError("用户已停止任务")
        self._wait_while_paused()
        if self.stop_requested.is_set():
            raise TaskStoppedError("用户已停止任务")

    def _wait_scope_run_started(self, scope, cycle_idx: int, total_cycles: int, timeout_s: float = 1.2):
        deadline = time.time() + timeout_s
        last_state = ""
        last_trig = ""
        while time.time() < deadline:
            self._check_task_control()
            try:
                acq = self.query_acquire_status(scope)
                last_state = acq.get("state", "")
                last_trig = acq.get("trig_state", "")
                if last_state == "1" and last_trig != "SAVE":
                    time.sleep(0.08)
                    QApplication.processEvents()
                    return
            except Exception:
                pass
            time.sleep(0.05)
            QApplication.processEvents()
        if LOG_CAPTURE_DETAIL:
            self.append_status(
                f"[循环 {cycle_idx}/{total_cycles}] RUN确认超时，继续等待采集完成 "
                f"(state={last_state}, trig={last_trig})"
            )

    def _wait_scope_average_ready(self, scope, cycle_idx: int, total_cycles: int, timeout_s: float = 30.0):
        if not self.avg_checkbox.isChecked():
            time.sleep(0.2)
            QApplication.processEvents()
            return

        target = self.get_avg_count()
        deadline = time.time() + timeout_s
        last_numacq = -1
        while time.time() < deadline:
            paused_s = self._wait_while_paused()
            if paused_s:
                deadline += paused_s
            if self.stop_requested.is_set():
                raise TaskStoppedError("用户已停止任务")
            acq = self.query_acquire_status(scope)
            last_numacq = int(acq.get("numacq", 0))
            if last_numacq >= target:
                time.sleep(0.08)
                QApplication.processEvents()
                return
            time.sleep(0.15)
            QApplication.processEvents()
        raise RuntimeError(f"第 {cycle_idx} 次循环等待平均完成超时: {last_numacq}/{target}")

    def _finish_task_ui(self):
        self.task_running = False
        self.pause_requested.clear()
        self._set_run_button_state("RUN")

    def run_capture_once(self, scope, cycle_idx: int, total_cycles: int):
        self._check_task_control()
        if TEST_MODE:
            if LOG_CAPTURE_DETAIL:
                self.append_status(f"[测试模式][循环 {cycle_idx}/{total_cycles}] 模拟采集完成")
            time.sleep(0.05)
            QApplication.processEvents()
            return
        capture_mode = self.get_capture_mode()
        profile = scope.profile
        _w(scope, profile.stopafter_run_cmd)
        _w(scope, profile.acq_run_cmd)
        self._wait_scope_run_started(scope, cycle_idx, total_cycles)

        if capture_mode == "CONTINUOUS":
            self._wait_scope_average_ready(scope, cycle_idx, total_cycles)
            if LOG_CAPTURE_DETAIL:
                self.append_status(f"[循环 {cycle_idx}/{total_cycles}] 连续采样运行")
            return

        _w(scope, profile.stopafter_single_cmd)
        _w(scope, profile.acq_run_cmd)
        self._wait_scope_run_started(scope, cycle_idx, total_cycles)

        deadline = time.time() + 30.0
        while time.time() < deadline:
            paused_s = self._wait_while_paused()
            if paused_s:
                deadline += paused_s
            if self.stop_requested.is_set():
                raise TaskStoppedError("用户已停止任务")
            acq = self.query_acquire_status(scope)
            is_running = acq["state"] == "1" and acq["trig_state"] != "SAVE"
            if not is_running:
                if LOG_CAPTURE_DETAIL:
                    self.append_status(f"[循环 {cycle_idx}/{total_cycles}] SINGLE完成, 触发状态={acq['trig_state']}")
                return
            time.sleep(0.15)
            QApplication.processEvents()
        raise RuntimeError(f"第 {cycle_idx} 次循环等待 SINGLE 完成超时")

    def _write_test_capture_files(self, save_dir: str, idx: int, stage_idx: int, cycle_no: int):
        save_format = self.get_save_format()
        waveform_data = {}
        t = np.linspace(0.0, 1.0, TEST_CAPTURE_POINTS)
        waveform_data["TIME"] = t
        for ch_index, ch in enumerate(sorted(self.selected_channels), start=1):
            freq = 4.0 + ch_index
            phase = stage_idx * 0.2 + cycle_no * 0.1
            baseline = 0.05 * stage_idx
            waveform_data[ch] = baseline + np.sin(2 * np.pi * freq * t + phase) * np.exp(-2.5 * t)

        save_path = os.path.join(save_dir, make_filename(idx, self.selected_channels, save_format))
        screenshot_path = os.path.join(save_dir, make_screenshot_filename(idx, self.selected_channels))
        save_waveform_file(
            save_path,
            waveform_data,
            save_format=save_format,
            meta={"mode": "TEST", "stage_idx": stage_idx, "cycle_no": cycle_no},
        )
        with open(screenshot_path, "wb") as f:
            f.write(TEST_PNG_BYTES)
        return save_path, screenshot_path

    def with_scope(self):
        if not self.resource_str:
            raise RuntimeError("示波器未锁定")
        rm, scope, _ = open_scope(self.resource_str)
        return rm, scope

    def _close_scope_session(self, rm, scope, reason: str = ""):
        if TEST_MODE:
            return
        try:
            if scope:
                try:
                    _w(scope, scope.profile.acq_stop_cmd, retry=1, sleep_s=0.02)
                except Exception:
                    pass
                clear_session(scope)
                scope.close()
        except Exception as ex:
            if reason:
                self.append_status(f"[示波器] 会话关闭异常（{reason}）: {ex}")
        finally:
            try:
                if rm:
                    rm.close()
            except Exception as ex:
                if reason:
                    self.append_status(f"[示波器] 资源管理器关闭异常（{reason}）: {ex}")

    def _recover_scope_connection(self, reason: str, previous_resource: str | None, attempt: int):
        self.append_status(f"[示波器] 正在执行强恢复（{reason}，第{attempt}次）...")
        if previous_resource:
            clear_error = force_clear_scope_resource(previous_resource)
            if clear_error:
                self.append_status(f"[示波器] 旧资源清理未完成: {clear_error}")

        self.resource_str = None
        self.idn = None
        self.profile_name = None
        QApplication.processEvents()
        time.sleep(min(0.8 * attempt, 3.0))
        self.try_lock_scope(show_error=False, verbose=False)

    def _open_scope_session_with_recovery(self, reason: str = "采集", max_attempts: int = 4):
        if TEST_MODE:
            self.resource_str = self.resource_str or "TEST::SCOPE"
            self.profile_name = self.profile_name or "DPO2024B"
            return None, TestScope()

        errors = []
        for attempt in range(1, max_attempts + 1):
            rm = scope = None
            previous_resource = self.resource_str
            try:
                if not self.resource_str:
                    ok, _ = self.try_lock_scope(show_error=False, verbose=False)
                    if not ok:
                        raise RuntimeError("示波器未锁定")
                rm, scope, _ = open_scope(self.resource_str)
                clear_session(scope)
                return rm, scope
            except Exception as ex:
                errors.append(f"第{attempt}次: {ex}")
                self._close_scope_session(rm, scope)
                self.append_status(f"[示波器] 会话打开失败（{reason}，第{attempt}次）: {ex}")
                if attempt < max_attempts:
                    self._recover_scope_connection(reason, previous_resource, attempt)
        raise RuntimeError(f"示波器重连失败（{reason}）\n" + "\n".join(errors))

    def _scope_heartbeat(self, scope, reason: str = ""):
        if TEST_MODE or not SCOPE_HEARTBEAT_BEFORE_CYCLE:
            return
        with temporary_timeout(scope, 1500):
            idn = scope.query("*IDN?").strip()
        if not idn:
            raise RuntimeError(f"示波器心跳无响应（{reason}）")

    @staticmethod
    def _is_scope_recoverable_error(ex: Exception) -> bool:
        if isinstance(ex, pyvisa.errors.VisaIOError):
            return True
        text = str(ex).upper()
        markers = (
            "TIMEOUT",
            "TIMED OUT",
            "VI_ERROR",
            "CONN_LOST",
            "CONNECTION LOST",
            "CURVE传输超时",
            "波形读取失败",
            "截屏",
            "HARDCOPY",
            "USB",
            "USBTMC",
            "DEVICE",
            "NO LISTENERS",
            "RESOURCE",
            "INVALID SESSION",
            "SESSION",
        )
        return any(m in text for m in markers)

    def try_lock_scope(self, show_error=True, verbose=True):
        if TEST_MODE:
            self.resource_str = "TEST::SCOPE"
            self.idn = "TEST,TEST_SCOPE,TESTMODE,0"
            self.profile_name = "DPO2024B"
            if verbose:
                self.append_status("[测试模式] 示波器锁定已模拟")
            return True, "TESTMODE"

        errors = []
        cached_resource = self.settings.value("scope_resource_str", "", type=str)
        manual_resource = getattr(self, "scope_manual_resource", "")

        for attempt in range(1, SCOPE_LOCK_ATTEMPTS + 1):
            try:
                if attempt > 1:
                    QApplication.processEvents()
                    time.sleep(min(0.8 * attempt, 3.0))
                device_info = discover_scope_resource_robust(
                    SERIAL_LOCK,
                    cached_resource=cached_resource,
                    manual_resource=manual_resource,
                    connection_mode=self.scope_connection_mode,
                )
                self.resource_str = device_info["resource"]
                self.idn = device_info["idn"].strip()
                self.profile_name = device_info["profile"].name
                self.settings.setValue("scope_resource_str", self.resource_str)
                parts = [x.strip() for x in self.idn.split(",")]
                sn = parts[2] if len(parts) > 2 and parts[2] else self.idn
                if verbose:
                    self.append_status(f"[示波器] 已锁定 [{sn}]")
                    self.append_status(f"[示波器] 型号: {self.profile_name}")
                    self.append_status(f"[示波器] 资源: {self.resource_str}")
                    self.append_status(f"[示波器] IDN: {self.idn}")
                return True, sn
            except Exception as e:
                errors.append(f"第{attempt}次: {e}")
                if verbose:
                    self.append_status(f"[示波器] 锁定失败（第{attempt}/{SCOPE_LOCK_ATTEMPTS}次）: {e}")
                for resource in (self.resource_str, cached_resource):
                    if resource:
                        force_clear_scope_resource(resource)
                if attempt == 1:
                    cached_resource = ""

        self.settings.remove("scope_resource_str")
        self.resource_str = None
        self.idn = None
        self.profile_name = None
        error_text = "\n".join(errors[-3:])
        if verbose:
            self.append_status(f"[示波器] 未连接或锁定失败，已重试 {SCOPE_LOCK_ATTEMPTS} 次")
        if show_error:
            QMessageBox.critical(self, "连接失败", f"无法锁定示波器\n\n{error_text}")
        return False, "未连接"

    @staticmethod
    def _build_manual_axis_targets(text: str, name: str) -> list[dict]:
        text = text.strip()
        if not text:
            return []

        serialized_segments = MainWindow._parse_serialized_manual_axis(text)
        if serialized_segments is not None:
            targets: list[dict] = []
            eps = 1e-9
            for segment in serialized_segments:
                start = float(segment["start"])
                step = float(segment["step"])
                end = float(segment["end"])
                mode = segment["mode"]
                rate = segment["rate"]
                tol = segment.get("tol")
                hold_s = segment.get("hold_s")
                if not all(np.isfinite(v) for v in (start, step, end)):
                    raise RuntimeError(f"{name}自定义路径包含无效数字")
                if step <= 0:
                    raise RuntimeError(f"{name}自定义路径间隔必须大于 0（当前: {step}）")
                if mode == "RAMP":
                    if rate is None or not np.isfinite(float(rate)) or float(rate) <= 0:
                        raise RuntimeError(f"{name}自定义路径 Ramp 速率必须大于 0")
                if tol is not None and (not np.isfinite(float(tol)) or float(tol) < 0):
                    raise RuntimeError(f"{name}自定义路径波动阈值不能小于 0")
                if hold_s is not None and (not np.isfinite(float(hold_s)) or float(hold_s) < 0):
                    raise RuntimeError(f"{name}自定义路径稳定时间不能小于 0")
                if targets and abs(targets[-1]["value"] - start) > eps:
                    raise RuntimeError(
                        f"{name}自定义路径不连续：上一段终点 {targets[-1]['value']:.6g}，"
                        f"下一段起点 {start:.6g}"
                    )

                direction = 1 if end >= start else -1
                cur = start
                guard = 0
                while (direction > 0 and cur <= end + eps) or (direction < 0 and cur >= end - eps):
                    if not targets or abs(targets[-1]["value"] - cur) > eps:
                        targets.append(
                            {
                                "value": round(cur, 6),
                                "rate": rate,
                                "mode": mode,
                                "tol": tol,
                                "hold_s": hold_s,
                            }
                        )
                    cur += direction * step
                    guard += 1
                    if guard > 20000:
                        raise RuntimeError(f"{name}自定义路径阶段数量异常，请检查间隔")
                if not targets or abs(targets[-1]["value"] - end) > eps:
                    targets.append({"value": end, "rate": rate, "mode": mode, "tol": tol, "hold_s": hold_s})
            return targets

        targets: list[dict] = []
        segments = [part.strip() for part in re.split(r"[;；\n]+", text) if part.strip()]
        for segment in segments:
            match = re.match(
                r"^\s*([-+]?\d+(?:\.\d+)?)\s*(?:->|→|-|,|:|：)\s*(h|\d+(?:\.\d+)?)\s*(?:->|→|-|,|:|：)\s*([-+]?\d+(?:\.\d+)?)\s*$",
                segment,
                re.IGNORECASE,
            )
            if not match:
                raise RuntimeError(f"{name}自定义路径格式错误，请使用: 起点-速率-终点 或 起点-h-终点")

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
                    raise RuntimeError(f"{name}自定义路径速率必须大于 0（当前: {rate}）")

            if targets and abs(targets[-1]["value"] - start) > 1e-9:
                raise RuntimeError(
                    f"{name}自定义路径不连续：上一段终点 {targets[-1]['value']:.6g}，"
                    f"下一段起点 {start:.6g}"
                )

            if not targets:
                targets.append({"value": start, "rate": rate, "mode": mode})
            targets.append({"value": end, "rate": rate, "mode": mode})

        return targets

    @staticmethod
    def _describe_manual_axis_targets(targets: list[dict], value_unit: str, rate_unit: str) -> tuple[int, str]:
        if len(targets) < 2:
            return 0, "(空)"

        parts = []
        for i in range(1, len(targets)):
            start = targets[i - 1]["value"]
            end = targets[i]["value"]
            mode = targets[i].get("mode")
            rate = targets[i].get("rate")
            tol = targets[i].get("tol")
            hold_s = targets[i].get("hold_s")
            stable_text = ""
            if tol is not None and hold_s is not None:
                stable_text = f"，±{float(tol):g}{value_unit}/{float(hold_s):g}s"
            if mode == "HOLD":
                parts.append(f"{i}. {start:g}{value_unit} -> HOLD -> {end:g}{value_unit}{stable_text}")
            else:
                parts.append(f"{i}. {start:g}{value_unit} -> {rate:g}{rate_unit} -> {end:g}{value_unit}{stable_text}")
        return len(parts), "; ".join(parts)

    def _summarize_axis_targets(self, targets: list[dict], value_unit: str) -> str:
        values = [item.get("value") for item in targets if item.get("value") is not None]
        if not values:
            return "空"
        ramp_count = sum(1 for item in targets[1:] if item.get("mode") != "HOLD")
        hold_count = sum(1 for item in targets[1:] if item.get("mode") == "HOLD")
        rates = sorted({float(item["rate"]) for item in targets if item.get("rate") is not None})
        tols = sorted({float(item["tol"]) for item in targets if item.get("tol") is not None})
        holds = sorted({float(item["hold_s"]) for item in targets if item.get("hold_s") is not None})
        parts = [
            f"{len(targets) - 1} 段",
            f"{len(targets)} 点",
            f"{self._format_value_list_preview(values, value_unit)}",
        ]
        if ramp_count and hold_count:
            parts.append(f"Ramp {ramp_count} / Hold {hold_count}")
        elif hold_count:
            parts.append(f"Hold {hold_count}")
        if rates:
            parts.append(f"速率 {self._format_value_list_preview(rates, value_unit + '/min')}")
        if tols:
            parts.append(f"阈值 ±{self._format_value_list_preview(tols, value_unit)}")
        if holds:
            parts.append(f"稳定 {self._format_value_list_preview(holds, 's')}")
        return "；".join(parts)

    def _build_temp_axis_targets(self) -> list[dict]:
        manual = self._build_manual_axis_targets(self.temp_manual_points_edit.text(), "温度")
        if not manual:
            raise RuntimeError("请先编辑温度路径")
        return manual

    def _build_press_axis_targets(self) -> list[dict]:
        manual = self._build_manual_axis_targets(self.press_manual_points_edit.text(), "压力")
        if not manual:
            raise RuntimeError("请先编辑压力路径")
        return manual

    @staticmethod
    def _rate_limit_message(name: str, rate: float, unit: str) -> str:
        return f"{name}不能大于 {MAX_RAMP_RATE:g} {unit}（当前: {rate:.2f} {unit}）"

    def _validate_ramp_rate(self, title: str, name: str, rate: float, unit: str) -> bool:
        if rate > MAX_RAMP_RATE:
            QMessageBox.warning(
                self,
                title,
                self._rate_limit_message(name, rate, unit),
            )
            return False
        return True

    def _validate_auto_ramp_rates(self) -> bool:
        try:
            if self.temp_cal_btn.isChecked():
                temp_targets = self._build_temp_axis_targets()
                rates = [item["rate"] for item in temp_targets if item["rate"] is not None]
                for rate in rates:
                    if not self._validate_ramp_rate("标定台参数错误", "升温速率", float(rate), "℃/min"):
                        return False

            if self.press_cal_btn.isChecked():
                press_targets = self._build_press_axis_targets()
                rates = [item["rate"] for item in press_targets if item["rate"] is not None]
                for rate in rates:
                    if not self._validate_ramp_rate("压力参数错误", "升压速率", float(rate), "kPa/min"):
                        return False
        except RuntimeError as ex:
            QMessageBox.warning(self, "标定路径参数错误", str(ex))
            return False

        return True

    def _build_stage_targets(self) -> list[dict]:
        need_temp = self.temp_cal_btn.isChecked()
        need_press = self.press_cal_btn.isChecked()

        temp_targets = [{"value": None, "rate": None}]
        press_targets = [{"value": None, "rate": None}]
        if need_temp:
            temp_targets = self._build_temp_axis_targets()
            self.append_status(f"[阶段] 温度路径: {self._summarize_axis_targets(temp_targets, '℃')}")
        if need_press:
            press_targets = self._build_press_axis_targets()
            self.append_status(f"[阶段] 压力路径: {self._summarize_axis_targets(press_targets, 'kPa')}")

        if need_temp and need_press:
            stages = []
            order = self._stage_control_order()
            temp_hold_policy = self._temp_hold_policy()
            if order == "PRESS_THEN_TEMP":
                for press_idx, press in enumerate(press_targets):
                    for temp_idx, temp in enumerate(temp_targets):
                        stages.append(
                            {
                                "temp": float(temp["value"]),
                                "temp_rate": temp["rate"],
                                "temp_mode": temp.get("mode"),
                                "temp_tol": temp.get("tol"),
                                "temp_hold_s": temp.get("hold_s"),
                                "press": float(press["value"]),
                                "press_rate": press["rate"],
                                "press_mode": press.get("mode"),
                                "press_tol": press.get("tol"),
                                "press_hold_s": press.get("hold_s"),
                                # 压力优先遍历时，压力组切换后需要先把温度拉回本轮温度起点，
                                # 再切换到新的压力，避免在上一轮高温末点直接改压力。
                                "pre_temp_before_press": press_idx > 0 and temp_idx == 0,
                                # 同一压力组内后续温度点不重复发送压力设定，只继续走温度序列。
                                "press_already_set": temp_idx > 0,
                            }
                        )
                priority_text = "按压力优先遍历；切换压力前先回到温度起点，再控压后循环温度"
            elif order == "TEMP_THEN_PRESS":
                temp_hold_once = temp_hold_policy == "FIRST_PRESSURE_POINT"
                for temp in temp_targets:
                    for press_idx, press in enumerate(press_targets):
                        stage = {
                            "temp": float(temp["value"]),
                            "temp_rate": temp["rate"],
                            "temp_mode": temp.get("mode"),
                            "temp_tol": temp.get("tol"),
                            "temp_hold_s": temp.get("hold_s"),
                            "press": float(press["value"]),
                            "press_rate": press["rate"],
                            "press_mode": press.get("mode"),
                            "press_tol": press.get("tol"),
                            "press_hold_s": press.get("hold_s"),
                        }
                        if temp_hold_once:
                            stage["temp_already_set"] = press_idx > 0
                        stages.append(stage)
                if temp_hold_once:
                    priority_text = "按温度优先遍历；每个温度点首个压力点温稳一次，后续压力点只等压稳"
                else:
                    priority_text = "按温度优先遍历，阶段内先控温后控压（原逻辑）"
            else:
                for temp in temp_targets:
                    for press in press_targets:
                        stages.append(
                            {
                                "temp": float(temp["value"]),
                                "temp_rate": temp["rate"],
                                "temp_mode": temp.get("mode"),
                                "temp_tol": temp.get("tol"),
                                "temp_hold_s": temp.get("hold_s"),
                                "press": float(press["value"]),
                                "press_rate": press["rate"],
                                "press_mode": press.get("mode"),
                                "press_tol": press.get("tol"),
                                "press_hold_s": press.get("hold_s"),
                            }
                        )
                priority_text = "按温度优先遍历，阶段内先控温后控压（原逻辑）"
            self.append_status(
                f"[阶段] 温度段数={len(temp_targets)}，压力段数={len(press_targets)}，{priority_text}"
            )
            self._log_stage_condition_summary(temp_targets, press_targets, order)
            return stages

        if need_temp:
            stages = [
                {
                    "temp": float(item["value"]),
                    "temp_rate": item["rate"],
                    "temp_mode": item.get("mode"),
                    "temp_tol": item.get("tol"),
                    "temp_hold_s": item.get("hold_s"),
                    "press": None,
                    "press_rate": None,
                    "press_mode": None,
                }
                for item in temp_targets
            ]
            self._log_stage_condition_summary(temp_targets, [{"value": None, "rate": None}], self._stage_control_order())
            return stages
        if need_press:
            stages = [
                {
                    "temp": None,
                    "temp_rate": None,
                    "temp_mode": None,
                    "press": float(item["value"]),
                    "press_rate": item["rate"],
                    "press_mode": item.get("mode"),
                    "press_tol": item.get("tol"),
                    "press_hold_s": item.get("hold_s"),
                }
                for item in press_targets
            ]
            self._log_stage_condition_summary([{"value": None, "rate": None}], press_targets, self._stage_control_order())
            return stages
        self._log_stage_condition_summary([{"value": None, "rate": None}], [{"value": None, "rate": None}], self._stage_control_order())
        return [{"temp": None, "temp_rate": None, "temp_mode": None, "press": None, "press_rate": None, "press_mode": None}]

    def _apply_stage_setpoints(
        self,
        temp_target: float | None,
        press_target: float | None,
        temp_rate: float | None = None,
        press_rate: float | None = None,
        temp_mode_override: str | None = None,
        press_mode_override: str | None = None,
        temp_tol: float | None = None,
        temp_hold_s: float | None = None,
        press_tol: float | None = None,
        press_hold_s: float | None = None,
    ):
        if temp_target is not None:
            self.ref_temp_spin.setValue(temp_target)
            self.current_temp_tol = temp_tol
            self.current_temp_hold_s = temp_hold_s
            mode = temp_mode_override or "RAMP"
            rate = float(temp_rate if temp_rate is not None else 5.0)
            if mode == "RAMP":
                self._send_hotplate_commands(
                    [
                        f"TEMPerature:OPSLave {self.control_slave}",
                        f"TEMPerature:RAMP {temp_target},{rate}",
                    ]
                )
                self.ramp_target_temp = temp_target
                self.ramp_rate_cpm = rate
                self.reach_inband_since = None
                self.reach_confirmed = False
                self.last_hotplate_s_status = None
                base_tc = self.last_live_tc if self.last_live_tc is not None else self.sim_tf_current
                self.sim_tf_target = temp_target
                self.sim_tf_current = base_tc
                self.sim_rate_cpm = abs(rate)
                self.sim_temp_mode = "ramp"
                self.hotplate_guard_timer.start()
                self.append_status(f"[阶段] 温度设定: RAMP -> {temp_target:.2f} ℃ @ {rate:.2f} ℃/min")
            else:
                self._send_hotplate_commands(
                    [
                        f"TEMPerature:OPSLave {self.control_slave}",
                        f"TEMPerature:HOLD {temp_target}",
                    ]
                )
                self.sim_tf_target = temp_target
                self.sim_tf_current = temp_target
                self.sim_rate_cpm = 0.0
                self.sim_temp_mode = "hold"
                self._reset_ramp_guard_state()
                self.append_status(f"[阶段] 温度设定: HOLD -> {temp_target:.2f} ℃")

        if press_target is not None:
            self.ref_press_spin.setValue(press_target)
            self.current_press_tol = press_tol
            self.current_press_hold_s = press_hold_s
            mode = press_mode_override or "RAMP"
            rate = float(press_rate if press_rate is not None else 5.0)
            if mode == "RAMP":
                self._send_hotplate_commands(
                    [
                        f"TEMPerature:OPSLave {self.press_control_slave}",
                        f"TEMPerature:RAMP {press_target},{rate}",
                    ]
                )
                self.press_ramp_target = press_target
                self.press_ramp_rate_kpm = rate
                self.press_reach_inband_since = None
                self.press_reach_confirmed = False
                self.last_hotplate_press_s_status = None
                self.press_guard_last_restart_ts = 0.0
                base_pk = self.last_live_pk if self.last_live_pk is not None else self.sim_pf_current
                self.sim_pf_target = press_target
                self.sim_pf_current = base_pk
                self.sim_prate_kpm = abs(rate)
                self.sim_press_mode = "ramp"
                self.hotplate_press_guard_timer.start()
                self.append_status(f"[阶段] 压力设定: RAMP -> {press_target:.2f} kPa @ {rate:.2f} kPa/min")
            else:
                self._send_hotplate_commands(
                    [
                        f"TEMPerature:OPSLave {self.press_control_slave}",
                        f"TEMPerature:HOLD {press_target}",
                    ]
                )
                self.sim_pf_target = press_target
                self.sim_pf_current = press_target
                self.sim_prate_kpm = 0.0
                self.sim_press_mode = "hold"
                self._reset_press_ramp_guard_state()
                self.append_status(f"[阶段] 压力设定: HOLD -> {press_target:.2f} kPa")

    def _wait_for_stage_stability(self, stage: dict, need_temp=None, need_press=None, reason: str = ""):
        self.wait_for_temp_press_stable(
            need_temp=need_temp,
            need_press=need_press,
            reason=reason,
            temp_target=stage.get("temp"),
            press_target=stage.get("press"),
            temp_tol=stage.get("temp_tol"),
            press_tol=stage.get("press_tol"),
            temp_hold_s=stage.get("temp_hold_s"),
            press_hold_s=stage.get("press_hold_s"),
        )

    def _apply_stage_setpoints_and_wait(self, stage: dict):
        """按用户选择的温压顺序执行阶段设定，并等待对应通道稳定。"""
        order = self._stage_control_order()
        temp_hold_policy = self._temp_hold_policy()
        has_temp = self.temp_cal_btn.isChecked() and stage.get("temp") is not None
        has_press = self.press_cal_btn.isChecked() and stage.get("press") is not None

        if has_temp and has_press and order == "PRESS_THEN_TEMP":
            if stage.get("pre_temp_before_press"):
                self.append_status("[阶段] 执行顺序: 切换压力前先回到温度起点，温度稳定后再控压")
                self._apply_stage_setpoints(
                    stage["temp"],
                    None,
                    stage.get("temp_rate"),
                    None,
                    stage.get("temp_mode"),
                    None,
                    temp_tol=stage.get("temp_tol"),
                    temp_hold_s=stage.get("temp_hold_s"),
                )
                self._wait_for_stage_stability(stage, need_temp=True, need_press=False, reason="切换压力前温度先达标")
                self._apply_stage_setpoints(
                    None,
                    stage["press"],
                    None,
                    stage.get("press_rate"),
                    None,
                    stage.get("press_mode"),
                    press_tol=stage.get("press_tol"),
                    press_hold_s=stage.get("press_hold_s"),
                )
                self._wait_for_stage_stability(stage, need_temp=False, need_press=True, reason="压力切换后达标")
                self._wait_for_stage_stability(stage, need_temp=True, need_press=True, reason="温压最终确认")
                return

            if stage.get("press_already_set"):
                self.append_status("[阶段] 执行顺序: 压力保持当前组，继续控温")
                self._apply_stage_setpoints(
                    stage["temp"],
                    None,
                    stage.get("temp_rate"),
                    None,
                    stage.get("temp_mode"),
                    None,
                    temp_tol=stage.get("temp_tol"),
                    temp_hold_s=stage.get("temp_hold_s"),
                )
                self._wait_for_stage_stability(stage, need_temp=True, need_press=False, reason="温度达标")
                self._wait_for_stage_stability(stage, need_temp=True, need_press=True, reason="温压最终确认")
                return

            self.append_status("[阶段] 执行顺序: 先控压，压力稳定后再控温")
            self._apply_stage_setpoints(
                None,
                stage["press"],
                None,
                stage.get("press_rate"),
                None,
                stage.get("press_mode"),
                press_tol=stage.get("press_tol"),
                press_hold_s=stage.get("press_hold_s"),
            )
            self._wait_for_stage_stability(stage, need_temp=False, need_press=True, reason="压力先达标")
            self._apply_stage_setpoints(
                stage["temp"],
                None,
                stage.get("temp_rate"),
                None,
                stage.get("temp_mode"),
                None,
                temp_tol=stage.get("temp_tol"),
                temp_hold_s=stage.get("temp_hold_s"),
            )
            self._wait_for_stage_stability(stage, need_temp=True, need_press=False, reason="温度后达标")
            self._wait_for_stage_stability(stage, need_temp=True, need_press=True, reason="温压最终确认")
            return

        if has_temp and has_press and order == "TEMP_THEN_PRESS":
            if temp_hold_policy == "FIRST_PRESSURE_POINT":
                if stage.get("temp_already_set"):
                    self.append_status("[阶段] 执行顺序: 温度已在本组首个压力点稳定，本压力点只控压")
                    self._apply_stage_setpoints(
                        None,
                        stage["press"],
                        None,
                        stage.get("press_rate"),
                        None,
                        stage.get("press_mode"),
                        press_tol=stage.get("press_tol"),
                        press_hold_s=stage.get("press_hold_s"),
                    )
                    self._wait_for_stage_stability(stage, need_temp=False, need_press=True, reason="压力达标")
                    return

                self.append_status("[阶段] 执行顺序: 先控温，本温度首个压力点温稳一次后再控压")
                self._apply_stage_setpoints(
                    stage["temp"],
                    None,
                    stage.get("temp_rate"),
                    None,
                    stage.get("temp_mode"),
                    None,
                    temp_tol=stage.get("temp_tol"),
                    temp_hold_s=stage.get("temp_hold_s"),
                )
                self._wait_for_stage_stability(stage, need_temp=True, need_press=False, reason="温度先达标")
                self._apply_stage_setpoints(
                    None,
                    stage["press"],
                    None,
                    stage.get("press_rate"),
                    None,
                    stage.get("press_mode"),
                    press_tol=stage.get("press_tol"),
                    press_hold_s=stage.get("press_hold_s"),
                )
                self._wait_for_stage_stability(stage, need_temp=False, need_press=True, reason="压力后达标")
                return

            self.append_status("[阶段] 执行顺序: 先控温，温度稳定后再控压")
            self._apply_stage_setpoints(
                stage["temp"],
                None,
                stage.get("temp_rate"),
                None,
                stage.get("temp_mode"),
                None,
                temp_tol=stage.get("temp_tol"),
                temp_hold_s=stage.get("temp_hold_s"),
            )
            self._wait_for_stage_stability(stage, need_temp=True, need_press=False, reason="温度先达标")
            self._apply_stage_setpoints(
                None,
                stage["press"],
                None,
                stage.get("press_rate"),
                None,
                stage.get("press_mode"),
                press_tol=stage.get("press_tol"),
                press_hold_s=stage.get("press_hold_s"),
            )
            self._wait_for_stage_stability(stage, need_temp=False, need_press=True, reason="压力后达标")
            self._wait_for_stage_stability(stage, need_temp=True, need_press=True, reason="温压最终确认")
            return

        if has_temp and has_press:
            self.append_status("[阶段] 执行顺序: 同步发送后等待")
        self._apply_stage_setpoints(
            stage.get("temp"),
            stage.get("press"),
            stage.get("temp_rate"),
            stage.get("press_rate"),
            stage.get("temp_mode"),
            stage.get("press_mode"),
            temp_tol=stage.get("temp_tol"),
            temp_hold_s=stage.get("temp_hold_s"),
            press_tol=stage.get("press_tol"),
            press_hold_s=stage.get("press_hold_s"),
        )
        self._wait_for_stage_stability(stage)

    def _run_scope_cycles_for_stage(self, scope, save_dir: str, start_idx: int, cycles: int, stage_idx: int, stage_total: int):
        capture_mode = self.get_capture_mode()
        save_format = self.get_save_format()
        saved_files = []
        for n in range(cycles):
            self._check_task_control()
            cycle_no = n + 1
            if LOG_CAPTURE_DETAIL:
                if capture_mode == "SINGLE":
                    self.append_status(f"[阶段 {stage_idx}/{stage_total}][循环 {cycle_no}/{cycles}] RUN -> SINGLE")
                else:
                    self.append_status(f"[阶段 {stage_idx}/{stage_total}][循环 {cycle_no}/{cycles}] RUN")
            idx, save_path, screenshot_path = make_available_capture_paths(save_dir, start_idx + n, self.selected_channels, save_format)
            self._scope_heartbeat(scope, reason=f"阶段 {stage_idx}/{stage_total} 循环 {cycle_no}/{cycles}")
            self.run_capture_once(scope, cycle_no, cycles)
            if TEST_MODE:
                save_path, screenshot_path = self._write_test_capture_files(save_dir, idx, stage_idx, cycle_no)
                saved_files.append(save_path)
                QApplication.processEvents()
                continue

            _w(scope, scope.profile.acq_stop_cmd)
            time.sleep(0.25)

            waveform_data = {}
            waveform_meta = {
                "format": save_format,
                "stage_idx": stage_idx,
                "stage_total": stage_total,
                "cycle_no": cycle_no,
                "channels": sorted(self.selected_channels),
                "scope": getattr(scope.profile, "name", ""),
            }
            base_time = None
            if scope.profile.name != "DPO2024B" or DPO2024B_SAVE_SCREENSHOT:
                save_scope_screenshot(scope, screenshot_path)
                if LOG_CAPTURE_DETAIL:
                    self.append_status(
                        f"[阶段 {stage_idx}/{stage_total}][循环 {cycle_no}/{cycles}] 截屏完成: {screenshot_path}"
                    )
            elif LOG_CAPTURE_DETAIL:
                self.append_status(f"[DPO2024B][阶段 {stage_idx}/{stage_total}][循环 {cycle_no}/{cycles}] 已跳过截屏以降低USB掉线概率")

            for ch in sorted(self.selected_channels):
                self._check_task_control()
                t, v, meta = read_waveform(scope, ch)
                if base_time is None:
                    base_time = t
                    waveform_data["TIME"] = t
                else:
                    if len(t) != len(base_time) or not np.allclose(t, base_time, rtol=0.0, atol=1e-15):
                        raise RuntimeError(f"{ch} 的时间轴与其他通道不一致，已停止合并保存")

                waveform_data[ch] = v
                waveform_meta[ch] = meta
                if LOG_CAPTURE_DETAIL:
                    self.append_status(
                        f"[阶段{stage_idx}][循环{cycle_no}][{ch}] scope={meta['scope']} {meta['composition']} "
                        f"width={meta['width']} dtype={meta['dtype']} BYT_OR={meta['byt_or']} "
                        f"BN_FMT={meta['bn_fmt']} ENC={meta['data_enc']} N={meta['recl']}"
                    )
                QApplication.processEvents()

            save_waveform_file(save_path, waveform_data, save_format=save_format, meta=waveform_meta)
            saved_files.append(save_path)
            self.append_status(f"[寿命][阶段 {stage_idx}/{stage_total}][循环 {cycle_no}/{cycles}] 已保存: {save_path}")
            QApplication.processEvents()
        first_idx = start_idx
        last_idx = idx
        mode_text = "测试模式" if TEST_MODE else "示波器"
        self.append_status(
            f"[{mode_text}][阶段 {stage_idx}/{stage_total}] 采集完成: {cycles} 组，起始请求 T{first_idx:04d}，最后保存 T{last_idx:04d}"
        )
        return saved_files, idx + 1

    def request_task_stop(self):
        if not self.task_running:
            self._set_run_button_state("RUN")
            return
        self.stop_requested.set()
        self.pause_requested.clear()
        self._set_run_button_state("RUN")
        self.append_status("[任务] 已请求停止，正在安全结束当前步骤...")
        self._stop_hotplate_and_live_plot("手动停止")
        try:
            self._set_siggen_output(False)
        except Exception:
            pass

    def _stop_hotplate_and_live_plot(self, reason: str = "流程结束"):
        try:
            self._send_hotplate_commands(
                [
                    f"TEMPerature:OPSLave {self.control_slave}",
                    "TEMPerature:STOP",
                    f"TEMPerature:OPSLave {self.press_control_slave}",
                    "TEMPerature:STOP",
                ]
            )
            self.append_status(f"[标定台] 已自动发送 STOP（{reason}）")
        except Exception as ex:
            self.append_status(f"[标定台] STOP 发送失败（{reason}）: {ex}")
        if self.hotplate_live_timer.isActive():
            self.hotplate_live_timer.stop()
            self.append_status("[标定台] 实时曲线已停止刷新")
        if self.hotplate_plot_timer.isActive():
            self.hotplate_plot_timer.stop()
        self.sim_temp_mode = "hold"
        self.sim_rate_cpm = 0.0
        self.sim_tf_target = self.sim_tf_current
        self.sim_press_mode = "hold"
        self.sim_prate_kpm = 0.0
        self.sim_pf_target = self.sim_pf_current
        self._reset_ramp_guard_state()
        self._reset_press_ramp_guard_state()

    def do_save(self):
        if self.task_running:
            return
        self.task_running = True
        self.stop_requested.clear()
        self.pause_requested.clear()
        self._set_run_button_state("暂停")
        save_dir = self.save_path_edit.text().strip()
        if save_dir and not os.path.isdir(save_dir):
            try:
                os.makedirs(save_dir, exist_ok=True)
            except Exception:
                QMessageBox.warning(self, "提示", "保存路径无效，请重新选择")
                self._finish_task_ui()
                return

        if not save_dir:
            QMessageBox.warning(self, "提示", "请先选择保存路径")
            self._finish_task_ui()
            return
        if not self._validate_auto_ramp_rates():
            self._finish_task_ui()
            return

        calibration_mode = self.get_calibration_mode()
        needs_scope, needs_spectrometer = self._current_mode_device_requirements()

        scope_ok, hot_ok, sig_ok, spec_ok = self._auto_initialize_for_start()
        if needs_scope and not scope_ok:
            QMessageBox.warning(self, "提示", "示波器未连接，无法开始")
            self._finish_task_ui()
            return
        if needs_spectrometer and not spec_ok:
            QMessageBox.warning(self, "提示", "光谱仪未连接，无法开始")
            self._finish_task_ui()
            return
        if not sig_ok:
            QMessageBox.warning(self, "提示", "信号发生器未连接，无法开始")
            self._finish_task_ui()
            return
        if (self.temp_cal_btn.isChecked() or self.press_cal_btn.isChecked()) and not hot_ok:
            QMessageBox.warning(self, "提示", "标定台未连接，无法开始")
            self._finish_task_ui()
            return

        try:
            self._init_run_log_files(save_dir)
            self.data_log_enabled = True
            mode_text = self.calibration_mode_combo.currentText()
            self.append_data(f"自动{mode_text}开始")
            self.append_status(f"自动{mode_text}开始")
            stages = self._build_stage_targets()
            cycles = int(self.cycles_spin.value())
            if cycles < 1:
                raise RuntimeError("循环次数必须大于等于 1")

            self.persist_save_path()
            start_idx = next_index(save_dir)

            capture_mode = self.get_capture_mode()
            save_format = self.get_save_format()
            self.append_status(
                f"[任务] 自动流程启动: 标定模式={mode_text}, 阶段数={len(stages)}, "
                f"每阶段循环={cycles}, 模式={capture_mode}, 保存格式={save_format}"
            )

            saved_files = []
            stage_total = len(stages)
            for i, stage in enumerate(stages, start=1):
                self._check_task_control()
                t_txt = "保持当前" if stage["temp"] is None else f"{stage['temp']:.2f}℃"
                if stage.get("temp_rate") is not None:
                    t_txt += f" @ {float(stage['temp_rate']):.2f}℃/min"
                elif stage.get("temp_mode") == "HOLD":
                    t_txt += " @ HOLD"
                p_txt = "保持当前" if stage["press"] is None else f"{stage['press']:.2f}kPa"
                if stage.get("press_rate") is not None:
                    p_txt += f" @ {float(stage['press_rate']):.2f}kPa/min"
                elif stage.get("press_mode") == "HOLD":
                    p_txt += " @ HOLD"
                self.append_status(f"[阶段 {i}/{stage_total}] 目标: 温度={t_txt}, 压力={p_txt}")

                self._apply_stage_setpoints_and_wait(stage)

                if calibration_mode == "SPECTRUM":
                    self._configure_siggen_spectrum_light()
                    self.wait_light_source_stable("开始光谱保存")
                    for spectrum_cycle in range(1, cycles + 1):
                        self._check_task_control()
                        spectrum_path = self._capture_spectrum_for_stage(
                            save_dir,
                            stage,
                            i,
                            stage_total,
                            cycle_no=spectrum_cycle,
                            cycle_total=cycles,
                        )
                        saved_files.append(spectrum_path)
                        self.append_status(
                            f"[光谱][阶段 {i}/{stage_total}][循环 {spectrum_cycle}/{cycles}] 已保存: {spectrum_path}"
                        )
                        QApplication.processEvents()
                    try:
                        self._set_siggen_output(False)
                    except Exception as ex:
                        self.append_status(f"[信号发生器] 关闭输出失败: {ex}")
                    continue

                self._configure_siggen_lifetime_light()
                self.wait_light_source_stable("开始示波器保存")

                stage_start_idx = start_idx
                self.append_status(
                    f"[测量开始][阶段 {i}/{stage_total}] 循环={cycles}，模式={capture_mode}，"
                    f"温度={t_txt}，压力={p_txt}，起始编号 T{stage_start_idx:04d}"
                )
                remaining = cycles
                resume_attempts = 0
                max_resume_attempts = 6
                stage_files = []
                while remaining > 0:
                    rm_stage = scope_stage = None
                    try:
                        rm_stage, scope_stage = self._open_scope_session_with_recovery(
                            reason=f"阶段 {i}/{stage_total} 采集前",
                            max_attempts=4,
                        )
                        if not TEST_MODE:
                            self.apply_acquire_settings(scope_stage)
                        new_files, start_idx = self._run_scope_cycles_for_stage(
                            scope_stage, save_dir, start_idx, remaining, i, stage_total
                        )
                        stage_files.extend(new_files)
                        remaining = 0
                    except TaskStoppedError:
                        raise
                    except Exception as ex:
                        resume_attempts += 1
                        if (not self._is_scope_recoverable_error(ex)) or resume_attempts > max_resume_attempts:
                            raise
                        self.append_status(
                            f"[示波器] 阶段 {i}/{stage_total} 采集中断（可恢复）: {ex}"
                        )
                        self.append_status(
                            f"[示波器] 正在自动重连并断点续采（第 {resume_attempts}/{max_resume_attempts} 次）..."
                        )
                        recovered_idx = max(start_idx, next_index(save_dir))
                        progressed = max(0, recovered_idx - start_idx)
                        if progressed > 0:
                            self.append_status(
                                f"[示波器] 已检测到本阶段新增 {progressed} 个文件，按断点继续"
                            )
                        start_idx = recovered_idx
                        remaining = max(0, cycles - len(stage_files) - progressed)
                        if remaining > 0:
                            self._recover_scope_connection(
                                f"阶段 {i}/{stage_total} 断点续采",
                                self.resource_str,
                                resume_attempts,
                            )
                    finally:
                        self._close_scope_session(
                            rm_stage, scope_stage, reason=f"阶段 {i}/{stage_total} 子流程结束"
                        )

                saved_files.extend(stage_files)
                if calibration_mode == "LIFETIME_SPECTRUM":
                    self._configure_siggen_spectrum_light()
                    self.wait_light_source_stable("开始光谱保存")
                    for spectrum_cycle in range(1, cycles + 1):
                        self._check_task_control()
                        spectrum_path = self._capture_spectrum_for_stage(
                            save_dir,
                            stage,
                            i,
                            stage_total,
                            cycle_no=spectrum_cycle,
                            cycle_total=cycles,
                        )
                        saved_files.append(spectrum_path)
                        self.append_status(
                            f"[联合标定][阶段 {i}/{stage_total}][光谱循环 {spectrum_cycle}/{cycles}] 已保存: {spectrum_path}"
                        )
                        QApplication.processEvents()
                try:
                    self._set_siggen_output(False)
                except Exception as ex:
                    self.append_status(f"[信号发生器] 关闭输出失败: {ex}")

            self._stop_hotplate_and_live_plot("流程完成")

            summary = "\n".join(saved_files[:6])
            if len(saved_files) > 6:
                summary += f"\n... 共 {len(saved_files)} 个文件"
            if calibration_mode == "SPECTRUM":
                completion_msg = f"已完成 {stage_total} 个阶段，每阶段 {cycles} 张光谱\n{summary}"
            elif calibration_mode == "LIFETIME_SPECTRUM":
                completion_msg = f"已完成 {stage_total} 个阶段，每阶段 {cycles} 次寿命采集 + {cycles} 张光谱\n{summary}"
            else:
                completion_msg = f"已完成 {stage_total} 个阶段，每阶段 {cycles} 次采集\n{summary}"
            if calibration_mode == "SPECTRUM":
                QMessageBox.information(self, "保存完成", completion_msg)
                self.append_status("[后处理] 光谱标定完成，跳过寿命拟合")
            elif self.fit_enable_btn.isChecked():
                self.append_status(f"[任务] {completion_msg}")
                self.append_status("[后处理] 自动启动寿命拟合全部文件结果保存")
                fit_win, csv_files = self._prepare_info_fit_context(save_dir)
                fit_win.show()
                fit_win.raise_()
                fit_win.activateWindow()
                if csv_files:
                    # 延迟到窗口事件完成后再触发，避免被当前流程结束时序阻塞。
                    QTimer.singleShot(100, fit_win.auto_run_save_all)
                else:
                    self.append_status("[后处理] 未找到CSV文件，已跳过寿命拟合保存")
            else:
                QMessageBox.information(self, "保存完成", completion_msg)
                self.append_status("[后处理] 已关闭自动寿命拟合，跳过全部文件结果保存")

        except TaskStoppedError:
            QMessageBox.information(self, "任务已停止", "自动流程已按请求停止。")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"{e}\n\n{traceback.format_exc()}")
        finally:
            try:
                self._set_siggen_output(False)
            except Exception:
                pass
            self._finish_task_ui()
            self.data_log_enabled = False
            self.sensor_log_file_path = None
            self.status_log_file_path = None

    def _log_hotplate_send_commands(self, commands, baud=None, prefix="SEND"):
        """把标定台写命令写入活动日志。实时温压查询不走这个函数，所以不会刷屏。"""
        if not LOG_HOTPLATE_COMMANDS:
            return
        try:
            baud_text = "" if baud is None else f" @{baud}bps"
            for cmd in commands:
                self.append_status(f"[标定台命令] {prefix}{baud_text}: {cmd}")
        except Exception:
            pass

    def _log_hotplate_diag_query(self, cmd: str, baud=None):
        """异常诊断/守护查询要显式写入活动日志，便于复盘。"""
        try:
            baud_text = "" if baud is None else f" @{baud}bps"
            self.append_status(f"[标定台命令] DIAG-QUERY{baud_text}: {cmd}")
        except Exception:
            pass

    def _send_hotplate_commands(self, commands):
        if TEST_MODE:
            self._log_hotplate_send_commands(commands, baud="TEST")
            return "TEST"
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.hotplate_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置标定台 COM 口，请先在 menu-settings 中设置")

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
                self._log_hotplate_send_commands(commands, baud=baud)
                return baud
            except Exception as ex:
                errors.append(f"{baud}bps -> {ex}")
            finally:
                try:
                    if ser and ser.is_open:
                        ser.close()
                except Exception:
                    pass
        raise RuntimeError(f"标定台命令发送失败: {port}\n" + "\n".join(errors))

    def _query_hotplate(self, command: str):
        if TEST_MODE:
            cmd = command.strip().upper()
            if cmd == "*IDN?":
                return "INSTEC,MK2000B,TEST_HOTPLATE,TEST", "TEST"
            if "RTINFORMATION" in cmd:
                temp_status = 2 if self.sim_temp_mode == "ramp" else 1
                press_status = 2 if self.sim_press_mode == "ramp" else 1
                resp = (
                    f"MK:1:{self.sim_tf_current:.3f}:0:{self.sim_tf_target:.3f}:0:0:0:{temp_status}:0:0\r\n"
                    f"MK:2:{self.sim_pf_current:.3f}:0:{self.sim_pf_target:.3f}:0:0:0:{press_status}:0:0"
                )
                return resp, "TEST"
            if "STATUS" in cmd:
                return "1", "TEST"
            if "ERROR" in cmd:
                return "0", "TEST"
            return "OK", "TEST"
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        port = (self.hotplate_com_port or "").strip()
        if not port:
            raise RuntimeError("未配置标定台 COM 口，请先在 menu-settings 中设置")

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
        raise RuntimeError(f"标定台查询失败: {port}\n" + "\n".join(errors))

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
    def _extract_slave_rt_fields(resp: str, slave: int):
        """解析 RTINformation 中指定 slave 的当前值、目标值、剩余/时间、状态和错误码。"""
        if not resp:
            return None, None, None, None, None
        frames = re.findall(r"MK:\d+:[^\r\n]*", resp, flags=re.IGNORECASE)
        for fr in frames:
            parts = fr.split(":")
            if len(parts) < 10:
                continue
            try:
                sx = int(float(parts[1]))
            except Exception:
                continue
            if sx != int(slave):
                continue
            current = target = tt = s_status = err_code = None
            try:
                current = float(parts[2])
            except Exception:
                pass
            try:
                target = float(parts[4])
            except Exception:
                pass
            try:
                tt = float(parts[5])
            except Exception:
                pass
            try:
                s_status = int(float(parts[8]))
            except Exception:
                pass
            try:
                err_code = int(float(parts[10].split(",")[-1])) if len(parts) > 10 else None
            except Exception:
                pass
            return current, target, tt, s_status, err_code
        return None, None, None, None, None

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

    def wait_for_temp_press_stable(
        self,
        need_temp=None,
        need_press=None,
        reason: str = "",
        temp_target: float | None = None,
        press_target: float | None = None,
        temp_tol: float | None = None,
        press_tol: float | None = None,
        temp_hold_s: float | None = None,
        press_hold_s: float | None = None,
    ):
        if need_temp is None:
            need_temp = self.temp_cal_btn.isChecked()
        if need_press is None:
            need_press = self.press_cal_btn.isChecked()
        if not need_temp and not need_press:
            return

        temp_target = float(self.ref_temp_spin.value() if temp_target is None else temp_target)
        press_target = float(self.ref_press_spin.value() if press_target is None else press_target)
        temp_tol = float(DEFAULT_TEMP_TOL if temp_tol is None else temp_tol)
        press_tol = float(DEFAULT_PRESS_TOL if press_tol is None else press_tol)
        temp_hold_s = float(DEFAULT_TEMP_HOLD_S if temp_hold_s is None else temp_hold_s)
        press_hold_s = float(DEFAULT_PRESS_HOLD_S if press_hold_s is None else press_hold_s)
        if TEST_MODE:
            temp_hold_s = min(temp_hold_s, TEST_MAX_STABLE_HOLD_S)
            press_hold_s = min(press_hold_s, TEST_MAX_STABLE_HOLD_S)

        temp_since = None
        press_since = None
        last_log_ts = 0.0

        parts = []
        if need_temp:
            parts.append(f"温度目标={temp_target:.2f}±{temp_tol:.2f}")
        if need_press:
            parts.append(f"压力目标={press_target:.2f}±{press_tol:.2f}")
        reason_text = f"（{reason}）" if reason else ""
        self.append_status(f"[稳定等待] 启动{reason_text}: {', '.join(parts)}")

        while True:
            paused_s = self._wait_while_paused()
            if paused_s:
                if temp_since is not None:
                    temp_since += paused_s
                if press_since is not None:
                    press_since += paused_s
                last_log_ts += paused_s
            if self.stop_requested.is_set():
                raise TaskStoppedError("用户已停止任务")
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

            if now - last_log_ts >= 1.0:
                fields = []
                if need_temp:
                    temp_prog = 0.0 if temp_since is None else max(0.0, now - temp_since)
                    fields.append(f"温稳进度={temp_prog:.1f}/{temp_hold_s:g}s")
                if need_press:
                    press_prog = 0.0 if press_since is None else max(0.0, now - press_since)
                    fields.append(f"压稳进度={press_prog:.1f}/{press_hold_s:g}s")
                self.append_data(", ".join(fields))
                last_log_ts = now

            if temp_ok and press_ok:
                if need_temp and need_press:
                    self.append_status("[稳定等待] 温度、压力已稳定")
                elif need_temp:
                    self.append_status("[稳定等待] 温度已稳定")
                else:
                    self.append_status("[稳定等待] 压力已稳定")
                return

            QApplication.processEvents()
            time.sleep(0.2)

    def _reset_ramp_guard_state(self):
        self.ramp_target_temp = None
        self.ramp_rate_cpm = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        self.last_hotplate_s_status = None
        self.hotplate_guard_timer.stop()

    def _update_reach_judge(self, tc_now, s_status, tf_rt=None, tt_rt=None, err_code=None):
        if self.ramp_target_temp is None or tc_now is None:
            return

        target = self.ramp_target_temp
        diff = abs(tc_now - target)
        tol = float(DEFAULT_TEMP_TOL if self.current_temp_tol is None else self.current_temp_tol)
        hold_s = float(DEFAULT_TEMP_HOLD_S if self.current_temp_hold_s is None else self.current_temp_hold_s)
        now = time.time()

        if s_status == 2:
            if diff <= tol:
                if self.reach_inband_since is None:
                    self.reach_inband_since = now
                inband_sec = now - self.reach_inband_since
            else:
                self.reach_inband_since = None
                inband_sec = 0.0

            if (not self.reach_confirmed) and inband_sec >= hold_s:
                self.reach_confirmed = True
                self.append_status(
                    f"[标定台] RAMP达标: |TC-TF| <= {tol:.3g}℃ 持续 {hold_s:g}s"
                )
        elif s_status == 0 and not self.reach_confirmed:
            inband_sec = 0.0 if self.reach_inband_since is None else (now - self.reach_inband_since)
            self.append_status(
                f"[标定台] WARN ramp stopped before reach-confirmed: tc={tc_now:.3f}, target={target:.3f}, "
                f"diff={diff:.3f}, inband={inband_sec:.1f}/{hold_s:g}s, tf={tf_rt}, tt={tt_rt}, err={err_code}"
            )

    def _diagnose_hotplate_unexpected_stop(self, axis: str = "温度"):
        commands = [
            "TEMPerature:STATus?",
            "TEMPerature:ERRor?",
            "TEMPerature:CHSWitch?",
            "TEMPerature:RANGe?",
            "TEMPerature:OPSLave?",
            "TEMPerature:RTINformation?",
        ]
        fields = []
        for cmd in commands:
            try:
                self._log_hotplate_diag_query(cmd)
                resp, baud = self._query_hotplate(cmd)
                self._log_hotplate_diag_query(cmd, baud=baud)
                fields.append(f"{cmd}@{baud}bps={resp!r}")
            except Exception as ex:
                fields.append(f"{cmd}=<ERR:{ex}>")
        self.append_status(f"[标定台] DIAG stop snapshot ({axis}): " + "; ".join(fields))

    def check_hotplate_ramp_guard(self):
        if self.ramp_target_temp is None:
            self.hotplate_guard_timer.stop()
            return
        try:
            rt_resp, baud = self._query_hotplate("TEMPerature:RTINformation?")
            tc, _, tf_rt, tt_rt, s_status, err_code = self._extract_rt_channels(rt_resp, self.control_slave)
            self._update_reach_judge(tc, s_status, tf_rt=tf_rt, tt_rt=tt_rt, err_code=err_code)

            if (
                self.last_hotplate_s_status == 2
                and s_status == 0
                and not self.reach_confirmed
            ):
                self.append_status(
                    f"[标定台] 警告: RAMP在未达标时提前停止 [{baud}bps], rt={rt_resp!r}"
                )
                self._diagnose_hotplate_unexpected_stop(axis="温度")
            self.last_hotplate_s_status = s_status

            if self.reach_confirmed and s_status in (0, 1):
                self.append_status("[标定台] 达标后已进入稳定/停止状态，停止RAMP监控")
                self.hotplate_guard_timer.stop()

        except Exception as ex:
            self.append_status(f"[标定台] 监控查询失败: {ex}")

    def _reset_press_ramp_guard_state(self):
        self.press_ramp_target = None
        self.press_ramp_rate_kpm = None
        self.press_reach_inband_since = None
        self.press_reach_confirmed = False
        self.last_hotplate_press_s_status = None
        self.press_guard_last_restart_ts = 0.0
        self.hotplate_press_guard_timer.stop()

    def _update_press_reach_judge(self, pk_now, s_status, pf_rt=None, tt_rt=None, err_code=None):
        if self.press_ramp_target is None or pk_now is None:
            return

        target = self.press_ramp_target
        try:
            tol = float(DEFAULT_PRESS_TOL if self.current_press_tol is None else self.current_press_tol)
        except Exception:
            tol = max(0.5, abs(target) * 0.001)
        try:
            hold_s = float(
                DEFAULT_PRESS_HOLD_S if self.current_press_hold_s is None else self.current_press_hold_s
            )
        except Exception:
            hold_s = TARGET_HOLD_S

        diff = abs(pk_now - target)
        now = time.time()

        if s_status == 2:
            if diff <= tol:
                if self.press_reach_inband_since is None:
                    self.press_reach_inband_since = now
                inband_sec = now - self.press_reach_inband_since
            else:
                self.press_reach_inband_since = None
                inband_sec = 0.0

            if (not self.press_reach_confirmed) and inband_sec >= hold_s:
                self.press_reach_confirmed = True
                self.append_status(
                    f"[标定台] P-RAMP达标: |PK-PF| <= {tol:.3f} kPa 持续 {hold_s:.0f}s"
                )
        elif s_status == 0 and not self.press_reach_confirmed:
            inband_sec = 0.0 if self.press_reach_inband_since is None else (now - self.press_reach_inband_since)
            self.append_status(
                f"[标定台] WARN pressure ramp stopped before reach-confirmed: pk={pk_now:.3f}, "
                f"target={target:.3f}, diff={diff:.3f}, inband={inband_sec:.1f}/{hold_s:.0f}s, "
                f"pf={pf_rt}, tt={tt_rt}, err={err_code}"
            )

    def _restart_hotplate_press_ramp_without_limit(self, reason: str):
        if self.press_ramp_target is None or self.press_ramp_rate_kpm is None:
            return
        now = time.time()
        # 无次数上限，但加一个短冷却，避免控制台持续 stop 时一秒刷爆串口。
        if now - self.press_guard_last_restart_ts < 5.0:
            return
        self.press_guard_last_restart_ts = now
        commands = [
            f"TEMPerature:OPSLave {self.press_control_slave}",
            f"TEMPerature:RAMP {self.press_ramp_target},{self.press_ramp_rate_kpm}",
        ]
        self.append_status(
            f"[标定台] 压力RAMP异常恢复: {reason}，重新发送 P-RAMP -> "
            f"{self.press_ramp_target:.3f} kPa @ {self.press_ramp_rate_kpm:.3f} kPa/min（无次数上限）"
        )
        self._send_hotplate_commands(commands)
        self.last_hotplate_press_s_status = None
        self.sim_pf_target = self.press_ramp_target
        self.sim_prate_kpm = abs(self.press_ramp_rate_kpm)
        self.sim_press_mode = "ramp"

    def check_hotplate_press_ramp_guard(self):
        if self.press_ramp_target is None:
            self.hotplate_press_guard_timer.stop()
            return
        try:
            rt_resp, baud = self._query_hotplate("TEMPerature:RTINformation?")
            pk, pf_rt, tt_rt, s_status, err_code = self._extract_slave_rt_fields(rt_resp, self.press_control_slave)
            if pk is None:
                _, pk = self._extract_temp_pressure_fallback(rt_resp)
            self._update_press_reach_judge(pk, s_status, pf_rt=pf_rt, tt_rt=tt_rt, err_code=err_code)

            if s_status == 0 and not self.press_reach_confirmed:
                self.append_status(
                    f"[标定台] 警告: P-RAMP在未达标时停止/卡住 [{baud}bps], rt={rt_resp!r}"
                )
                self._diagnose_hotplate_unexpected_stop(axis="压力")
                self._restart_hotplate_press_ramp_without_limit("检测到压力通道 STOP 且未达标")

            self.last_hotplate_press_s_status = s_status

            if self.press_reach_confirmed and s_status in (0, 1):
                self.append_status("[标定台] 压力达标后已进入稳定/停止状态，停止P-RAMP监控")
                self.hotplate_press_guard_timer.stop()

        except Exception as ex:
            self.append_status(f"[标定台] 压力监控查询失败: {ex}")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

