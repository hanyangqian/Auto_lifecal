# -*- coding: utf-8 -*-
"""
Tektronix DPO2024B / MDO34 waveform saver with simple UI

Key changes:
- IDN based profile detection: DPO2024B and MDO34 use separate logic.
- Save path uses fixed commands per model. No command guessing or fallback inside waveform save.
- MDO34 save does not use DATA:COMPOSITION.
- Acquire status query is profile specific.
- USB-first discovery with cached resource support.

Requirements:
  pip install pyvisa pyqt5 numpy pandas
NI-VISA must be installed, instrument connected.
"""

import os
import json
import re
import sys
import traceback
from contextlib import contextmanager
import time

import numpy as np
import pandas as pd
import pyvisa

from PyQt5.QtCore import Qt, QTimer, QSettings
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox,
    QFileDialog, QDialog, QDialogButtonBox, QCheckBox, QLabel, QHBoxLayout,
    QLineEdit, QComboBox
)


# =========================
# Global constants
# =========================
SERIAL_LOCK = "C030795"
DEFAULT_TIMEOUT_MS = 20000
CURVE_TIMEOUT_MS = 3000
MDO34_CURVE_TIMEOUT_MS = 120000
DISCOVERY_TIMEOUT_MS = 1500
DEFAULT_SCOPE_IP = ""

# 保存前准备阶段加速参数：
# 1) 每次保存时不再重复 *IDN?，直接使用锁定示波器时已经识别出的型号；
# 2) STOP 后等待时间从 0.35 s 缩短到 0.05 s。
# 如果你的示波器在极少数情况下保存到上一帧数据，可以把 SAVE_STOP_SETTLE_S 改回 0.15~0.35。
SAVE_STOP_SETTLE_S = 0

# 状态刷新参数
STATUS_POLL_INTERVAL_MS = 500
STATUS_POLL_MAX_ERRORS = 5


# =========================
# Scope profile
# =========================
class ScopeProfile:
    def __init__(
        self,
        name: str,
        idn_keywords,
        acquire_mode_query: str,
        acquire_state_query: str,
        acquire_numavg_query: str,
        acquire_numacq_query: str,
        acquire_stopafter_query: str,
        trigger_state_query: str,
        stopafter_run_cmd: str,
        stopafter_single_cmd: str,
        acq_run_cmd: str,
        acq_stop_cmd: str,
        acquire_mode_set_cmd: str,
        acquire_numavg_set_cmd: str,
        acquire_dialog_uses_acq_query: bool = False,
    ):
        self.name = name
        self.idn_keywords = [x.upper() for x in idn_keywords]
        self.acquire_mode_query = acquire_mode_query
        self.acquire_state_query = acquire_state_query
        self.acquire_numavg_query = acquire_numavg_query
        self.acquire_numacq_query = acquire_numacq_query
        self.acquire_stopafter_query = acquire_stopafter_query
        self.trigger_state_query = trigger_state_query
        self.stopafter_run_cmd = stopafter_run_cmd
        self.stopafter_single_cmd = stopafter_single_cmd
        self.acq_run_cmd = acq_run_cmd
        self.acq_stop_cmd = acq_stop_cmd
        self.acquire_mode_set_cmd = acquire_mode_set_cmd
        self.acquire_numavg_set_cmd = acquire_numavg_set_cmd
        self.acquire_dialog_uses_acq_query = acquire_dialog_uses_acq_query


PROFILE_DPO2024B = ScopeProfile(
    name="DPO2024B",
    idn_keywords=["DPO2024B"],
    acquire_mode_query="ACQ:MODE?",
    acquire_state_query="ACQ:STATE?",
    acquire_numavg_query="ACQ:NUMAVG?",
    acquire_numacq_query="ACQ:NUMACQ?",
    acquire_stopafter_query="ACQ:STOPAFTER?",
    trigger_state_query="TRIG:STATE?",
    stopafter_run_cmd="ACQ:STOPAFTER RUNST",
    stopafter_single_cmd="ACQ:STOPAFTER SEQ",
    acq_run_cmd="ACQ:STATE RUN",
    acq_stop_cmd="ACQ:STATE STOP",
    acquire_mode_set_cmd="ACQ:MODE {mode}",
    acquire_numavg_set_cmd="ACQ:NUMAVG {numavg}",
    acquire_dialog_uses_acq_query=True,
)

PROFILE_MDO34 = ScopeProfile(
    name="MDO34",
    idn_keywords=["MDO34"],
    acquire_mode_query="ACQ:MODE?",
    acquire_state_query="ACQ:STATE?",
    acquire_numavg_query="ACQ:NUMAVG?",
    acquire_numacq_query="ACQ:NUMACQ?",
    acquire_stopafter_query="ACQ:STOPAFTER?",
    trigger_state_query="TRIG:STATE?",
    stopafter_run_cmd="ACQ:STOPAFTER RUNST",
    stopafter_single_cmd="ACQ:STOPAFTER SEQ",
    acq_run_cmd="ACQ:STATE RUN",
    acq_stop_cmd="ACQ:STATE STOP",
    acquire_mode_set_cmd="ACQ:MODE {mode}",
    acquire_numavg_set_cmd="ACQ:NUMAVG {numavg}",
    acquire_dialog_uses_acq_query=False,
)

SUPPORTED_PROFILES = [PROFILE_DPO2024B, PROFILE_MDO34]


def profile_from_idn(idn: str) -> ScopeProfile:
    idn_upper = idn.upper()
    for profile in SUPPORTED_PROFILES:
        if any(k in idn_upper for k in profile.idn_keywords):
            return profile

    raise RuntimeError(
        "未识别到支持的示波器型号，已停止操作以避免错误 SCPI 阻塞通信。\n\n"
        f"IDN: {idn}\n\n"
        "当前仅支持 DPO2024B 和 MDO34。"
    )


def profile_from_name(profile_name: str) -> ScopeProfile:
    text = str(profile_name or "").upper()
    for profile in SUPPORTED_PROFILES:
        if profile.name.upper() == text:
            return profile
    raise RuntimeError(f"未识别到已锁定示波器型号：{profile_name}")


def open_scope_fast(resource_str: str, known_profile_name: str = ""):
    """
    保存时快速打开示波器会话。

    open_scope() 每次都会执行 *IDN? 并重新识别型号；
    这里复用“锁定示波器”阶段已经识别出的 profile_name，避免每次保存重复 *IDN?。
    """
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(resource_str)
    scope.timeout = DEFAULT_TIMEOUT_MS
    scope.read_termination = '\n'
    scope.write_termination = '\n'

    if known_profile_name:
        scope.profile = profile_from_name(known_profile_name)
    else:
        idn = scope.query("*IDN?").strip()
        scope.profile = profile_from_idn(idn)

    return rm, scope


def normalize_acq_mode(mode_text: str) -> str:
    text = mode_text.strip().upper()
    if " " in text:
        text = text.split()[-1].strip().upper()
    if text in ("AVE", "AVER", "AVERAGE"):
        return "AVERAGE"
    if text in ("SAM", "SAMPLE"):
        return "SAMPLE"
    return text


def normalize_acq_state(state_text: str) -> str:
    text = state_text.strip().upper()
    if " " in text:
        text = text.split()[-1].strip().upper()
    if text in ("1", "ON", "RUN", "RUNNING"):
        return "1"
    if text in ("0", "OFF", "STOP", "STOPPED"):
        return "0"
    return text


def normalize_stopafter(stop_text: str) -> str:
    text = stop_text.strip().upper()
    if " " in text:
        text = text.split()[-1].strip().upper()
    if text in ("RUNSTOP", "RUNST", "RUN/STOP"):
        return "RUNST"
    if text in ("SEQUENCE", "SEQ", "SINGLE"):
        return "SEQ"
    return text


def normalize_trigger_state(trigger_text: str) -> str:
    text = trigger_text.strip().upper()
    if " " in text:
        text = text.split()[-1].strip().upper()
    return text


def extract_last_value(resp: str) -> str:
    text = resp.strip()
    if " " in text:
        return text.split()[-1]
    return text


# =========================
# UI dialogs
# =========================
class AcquireDialog(QDialog):
    def __init__(self, scope, parent=None):
        super().__init__(parent)
        self.scope = scope
        self.profile = scope.profile
        self.setWindowTitle(f"Acquire 设置 - {self.profile.name}")

        layout = QVBoxLayout(self)

        mode, numavg, samplerate = self._read_initial_acquire_values()

        self.cb_mode = QComboBox()
        self.cb_mode.addItem("普通采样", "SAMPLE")
        self.cb_mode.addItem("平均", "AVERAGE")

        self.cb_avg = QComboBox()
        for v in ["2", "4", "8", "16", "32", "64", "128", "256", "512"]:
            self.cb_avg.addItem(v, v)

        def _set_by_data(combo: QComboBox, data_value: str):
            for i in range(combo.count()):
                if str(combo.itemData(i)).upper() == str(data_value).upper():
                    combo.setCurrentIndex(i)
                    return

        _set_by_data(self.cb_mode, mode)
        _set_by_data(self.cb_avg, str(numavg))
        self.cb_mode.currentIndexChanged.connect(self._sync_average_controls)

        self.lb_sr = QLabel(samplerate)
        self.lb_sr.setEnabled(False)

        def row(name, widget):
            h = QHBoxLayout()
            h.addWidget(QLabel(name))
            h.addWidget(widget)
            return h

        layout.addLayout(row("采样模式", self.cb_mode))
        layout.addLayout(row("平均次数", self.cb_avg))
        layout.addLayout(row("采样速率", self.lb_sr))
        self._sync_average_controls()

        btns = QDialogButtonBox(QDialogButtonBox.Apply | QDialogButtonBox.Close)
        btns.button(QDialogButtonBox.Apply).clicked.connect(self.apply)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def _read_initial_acquire_values(self):
        if self.profile.acquire_dialog_uses_acq_query:
            resp = _q(self.scope, "ACQ?").strip()
            parts = resp.split(";")
            if len(parts) < 5:
                raise RuntimeError(f"ACQ? 返回值格式异常：{resp}")
            _stopafter, _state, mode, numavg, samplerate = parts[:5]
            mode = normalize_acq_mode(mode)
            return mode, int(float(numavg)), samplerate

        mode = normalize_acq_mode(_q(self.scope, self.profile.acquire_mode_query))
        numavg = int(float(extract_last_value(_q(self.scope, self.profile.acquire_numavg_query))))
        samplerate = _q(self.scope, "HORIZONTAL:SAMPLERATE?").strip()
        return mode, numavg, samplerate

    def _sync_average_controls(self):
        is_average = self.cb_mode.currentData() == "AVERAGE"
        self.cb_avg.setEnabled(is_average)
        self.cb_avg.setToolTip("" if is_average else "普通采样模式下不使用平均次数")

    def apply(self):
        try:
            mode = self.cb_mode.currentData()
            numavg = int(self.cb_avg.currentData())

            _w(self.scope, self.profile.stopafter_run_cmd)
            _w(self.scope, self.profile.acquire_mode_set_cmd.format(mode=mode))

            if mode == "AVERAGE":
                _w(self.scope, self.profile.acquire_numavg_set_cmd.format(numavg=numavg))

            parent = self.parent()
            if parent and hasattr(parent, "on_acquire_applied"):
                parent.on_acquire_applied(mode, numavg)
            if parent and hasattr(parent, "show_notice"):
                parent.show_notice("Acquire", "设置已应用")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"写入失败：\n{e}")


class ChannelDialog(QDialog):
    def __init__(self, selected: set, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择保存通道（可多选）")
        self.selected = set(selected)

        layout = QVBoxLayout(self)
        self.checks = {}
        for i in range(1, 5):
            cb = QCheckBox(f"CH{i}")
            cb.setChecked(f"CH{i}" in self.selected)
            self.checks[f"CH{i}"] = cb
            layout.addWidget(cb)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def get_selected(self) -> set:
        s = set()
        for name, cb in self.checks.items():
            if cb.isChecked():
                s.add(name)
        return s


# =========================
# VISA / Scope helper
# =========================
def _parse_idn(idn: str):
    parts = [p.strip() for p in idn.strip().split(",")]
    maker = parts[0] if len(parts) > 0 else ""
    model = parts[1] if len(parts) > 1 else ""
    serial = parts[2] if len(parts) > 2 else ""
    firmware = parts[3] if len(parts) > 3 else ""
    return maker, model, serial, firmware


def build_tcpip_resource(address_or_resource: str) -> str:
    text = (address_or_resource or "").strip()
    if not text:
        return ""
    if "::" in text:
        return text
    return f"TCPIP0::{text}::inst0::INSTR"


def discover_scope_resource_fast(
    serial_lock: str,
    cached_resource: str = "",
    manual_resource: str = "",
    include_usb: bool = True,
    include_tcpip: bool = True,
):
    rm = pyvisa.ResourceManager()
    try:
        resources = list(rm.list_resources())

        usb_resources = [
            r for r in resources
            if r.upper().startswith("USB") and "::INSTR" in r.upper()
        ] if include_usb else []
        tcpip_resources = [
            r for r in resources
            if r.upper().startswith("TCPIP")
        ] if include_tcpip else []

        search_order = []

        manual_resource = build_tcpip_resource(manual_resource)
        if manual_resource:
            search_order.append(manual_resource)

        cached_upper = (cached_resource or "").upper()
        cached_allowed = (
            (include_usb and cached_upper.startswith("USB")) or
            (include_tcpip and cached_upper.startswith("TCPIP"))
        )
        if cached_resource and cached_allowed and cached_resource not in search_order:
            search_order.append(cached_resource)

        if serial_lock:
            for r in usb_resources:
                if serial_lock.upper() in r.upper() and r not in search_order:
                    search_order.append(r)

        tek_usb = [r for r in usb_resources if "0X0699" in r.upper() or "0699" in r.upper()]
        for r in tek_usb + usb_resources + tcpip_resources:
            if r not in search_order:
                search_order.append(r)

        if not search_order:
            raise RuntimeError(
                "未发现任何可尝试的 VISA 资源。请确认示波器已开机、网线已连接、NI-VISA 已安装，"
                "或在 IP 输入框中填写示波器 IP。"
            )

        matches = []
        errors = []

        for resource in search_order:
            scope = None
            try:
                scope = rm.open_resource(resource)
                scope.timeout = DISCOVERY_TIMEOUT_MS
                scope.read_termination = '\n'
                scope.write_termination = '\n'
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
                errors.append(f"{resource} -> {ex}")
            finally:
                if scope is not None:
                    try:
                        scope.close()
                    except Exception:
                        pass

        if len(matches) == 1:
            return matches[0]

        if len(matches) > 1:
            text = "\n".join(f"{m['resource']} -> {m['idn']}" for m in matches)
            raise RuntimeError("发现多台支持的 Tek 示波器，无法自动决定目标：\n" + text)

        err_text = "\n".join(errors[:12]) if errors else "无探测错误信息"
        raise RuntimeError(
            "未找到 DPO2024B 或 MDO34。\n\n"
            "当前 VISA 资源：\n" + ("\n".join(resources) if resources else "(无)") +
            "\n\n探测结果：\n" + err_text
        )

    finally:
        rm.close()


def open_scope(resource_str: str):
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(resource_str)

    scope.timeout = DEFAULT_TIMEOUT_MS
    scope.read_termination = '\n'
    scope.write_termination = '\n'

    idn = scope.query("*IDN?").strip()
    scope.profile = profile_from_idn(idn)
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


def _get_record_length_dpo2024b(scope) -> int:
    try:
        return int(float(_q(scope, "WFMOUTPRE:RECORDLENGTH?")))
    except Exception:
        return int(float(_q(scope, "HORIZONTAL:RECORDLENGTH?")))


def _get_record_length_mdo34(scope) -> int:
    return int(float(extract_last_value(_q(scope, "HORIZONTAL:RECORDLENGTH?"))))


def _get_transfer_points_mdo34(scope) -> int:
    # MDO34 manual uses WFMOutpre:NR_Pt? for the number of points that CURVe? will send.
    return int(float(extract_last_value(_q(scope, "WFMOUTPRE:NR_PT?"))))


def _query_optional(scope, cmd: str, default="N/A"):
    try:
        return _q(scope, cmd).strip()
    except Exception:
        return default


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


# =========================
# Waveform read
# =========================
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
    profile_name = scope.profile.name
    if profile_name == "DPO2024B":
        return read_waveform_dpo2024b(scope, ch_name)
    if profile_name == "MDO34":
        return read_waveform_mdo34(scope, ch_name)
    raise RuntimeError(f"不支持的示波器型号：{profile_name}")


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
    # MDO34 fixed save path. Do not send DATA:COMPOSITION here.
    # Full-record transfer: select the whole current acquisition record.
    # This does not change the oscilloscope record length; it only sets the readout range.
    _w(scope, f"DATA:SOURCE {ch_name}")
    time.sleep(0.08)

    record_length = _get_record_length_mdo34(scope)
    _w(scope, "DATA:START 1")
    _w(scope, f"DATA:STOP {record_length}")
    data_start = _query_optional(scope, "DATA:START?")
    data_stop = _query_optional(scope, "DATA:STOP?")

    # Use explicit WFMOutpre commands from the manual rather than DATA:ENC abbreviation.
    # Width 1 is the manual's analog waveform example and avoids huge 2-byte transfers.
    _w(scope, "WFMOUTPRE:ENCDG BINARY")
    _w(scope, "WFMOUTPRE:BYT_NR 1")
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

    with temporary_timeout(scope, MDO34_CURVE_TIMEOUT_MS):
        with binary_transfer_mode(scope):
            raw = scope.query_binary_values(
                "CURVE?",
                datatype="b",
                container=np.array,
                is_big_endian=True,
                expect_termination=True,
                chunk_size=1024 * 1024,
            )

    drain_input_buffer(scope)

    if len(raw) != recl:
        recl = len(raw)

    # 3 Series MDO manual analog conversion formula:
    # Yi = YZERO + YMULT * DataPointi
    # Do not subtract YOFF here.
    v = yzero + ymult * raw
    t = xzero + np.arange(recl) * xincr

    return t, v, {
        "scope": "MDO34",
        "width": 1,
        "dtype": "b",
        "composition": "N/A",
        "recl": recl,
        "record_length": record_length,
        "data_start": data_start,
        "data_stop": data_stop,
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


# =========================
# File naming / indexing
# =========================
FNAME_RE = re.compile(r"^T(\d{4})(CH[1-4](?:CH[1-4])*)?\.(?:CSV|NPZ)$", re.IGNORECASE)


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
    fmt = str(save_format or "NPZ").strip().upper()
    if fmt not in {"NPZ", "CSV"}:
        return "NPZ"
    return fmt


def make_filename(idx: int, channels, save_format="NPZ") -> str:
    suffix = "".join(sorted(channels))
    ext = normalize_save_format(save_format)
    return f"T{idx:04d}{suffix}.{ext}"


def save_waveform_npz(save_path: str, data: dict, meta=None):
    """
    无精度损失保存为 .npz。

    np.savez 会按 numpy 数组原始 dtype 保存二进制数据：
    - TIME 和 CHx 当前是 float64 数组时，就按 float64 保存；
    - 不做文本格式化；
    - 不做 %.10g / %.12g 之类的精度截断；
    - 不压缩，优先保证速度和原始数组精度。
    """
    arrays = {}
    for name, values in data.items():
        arrays[name] = np.asarray(values)
    if meta is not None:
        arrays["__meta_json__"] = np.asarray(json.dumps(meta, ensure_ascii=False, indent=2))
    np.savez(save_path, **arrays)


def save_waveform_csv(save_path: str, data: dict):
    """
    保存为 CSV，保持 pandas 默认写法：
    不指定 float_format，不做 %.10g / %.12g 等人为精度格式限制。
    """
    df = pd.DataFrame(data)
    df.to_csv(save_path, index=False)


def save_waveform_file(save_path: str, data: dict, save_format="NPZ", meta=None):
    fmt = normalize_save_format(save_format)
    if fmt == "CSV":
        save_waveform_csv(save_path, data)
    else:
        save_waveform_npz(save_path, data, meta=meta)


# =========================
# Main UI
# =========================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tek 波形保存（CSV / NPZ，DPO2024B / MDO34）")

        self.settings = QSettings("Codex", "TekWaveSaver")
        self.selected_channels = {"CH1"}
        self.resource_str = None
        self.idn = None
        self.profile_name = None
        self.device_serial = None
        self.awaiting_run_after_apply = False
        self.current_acq_mode = None
        self.current_numavg = None
        self.current_stopafter = None
        self.run_numacq_base = None
        self.status_extra_lines = []
        self._save_rm = None
        self._save_scope = None
        self._save_resource_str = None

        self.status_refresh_errors = 0

        self.progress_timer = QTimer(self)
        self.progress_timer.setInterval(STATUS_POLL_INTERVAL_MS)
        self.progress_timer.timeout.connect(self.refresh_acquire_progress)

        layout = QVBoxLayout(self)

        conn_layout = QHBoxLayout()
        self.ip_label = QLabel("示波器 IP / VISA:")
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("10.5.5.88 或 TCPIP0::10.5.5.88::inst0::INSTR")
        saved_ip = self.settings.value("scope_ip", DEFAULT_SCOPE_IP, type=str)
        self.ip_input.setText(saved_ip)
        self.btn_usb_connect = QPushButton("USB 自动识别")
        self.btn_lan_connect = QPushButton("网口连接")
        self.btn_usb_connect.clicked.connect(self.try_lock_scope_usb)
        self.btn_lan_connect.clicked.connect(self.try_lock_scope_lan)
        conn_layout.addWidget(self.ip_label)
        conn_layout.addWidget(self.ip_input)
        conn_layout.addWidget(self.btn_usb_connect)
        conn_layout.addWidget(self.btn_lan_connect)
        layout.addLayout(conn_layout)

        self.status_label = QLabel("设备：未连接")
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.status_label)

        self.save_dir = None
        self.btn_path = QPushButton("请设置保存路径")
        self.btn_path.clicked.connect(self.choose_path)
        layout.addWidget(self.btn_path)

        ch_fmt_layout = QHBoxLayout()
        self.btn_ch = QPushButton("通道：" + ",".join(sorted(self.selected_channels)))
        self.btn_ch.clicked.connect(self.choose_channels)
        ch_fmt_layout.addWidget(self.btn_ch, 1)

        ch_fmt_layout.addWidget(QLabel("保存格式:"))
        self.save_format_combo = QComboBox()
        self.save_format_combo.addItem("NPZ无损", "NPZ")
        self.save_format_combo.addItem("CSV", "CSV")
        saved_format = normalize_save_format(self.settings.value("save_format", "NPZ", type=str))
        idx_fmt = self.save_format_combo.findData(saved_format)
        self.save_format_combo.setCurrentIndex(idx_fmt if idx_fmt >= 0 else 0)
        self.save_format_combo.currentIndexChanged.connect(self.on_save_format_changed)
        ch_fmt_layout.addWidget(self.save_format_combo)
        layout.addLayout(ch_fmt_layout)

        self.btn_save = QPushButton("保存NPZ")
        self.btn_save.clicked.connect(self.do_save)
        layout.addWidget(self.btn_save)
        self.on_save_format_changed()

        ctrl_layout = QHBoxLayout()
        self.btn_run = QPushButton("RUN")
        self.btn_stop = QPushButton("STOP")
        self.btn_single = QPushButton("SINGLE")
        self.btn_acquire = QPushButton("Acquire")

        self.btn_run.clicked.connect(self.do_run)
        self.btn_stop.clicked.connect(self.do_stop)
        self.btn_single.clicked.connect(self.do_single)
        self.btn_acquire.clicked.connect(self.open_acquire)

        ctrl_layout.addWidget(self.btn_run)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_single)
        ctrl_layout.addWidget(self.btn_acquire)
        layout.addLayout(ctrl_layout)

        cmd_layout = QHBoxLayout()
        self.cmd_edit = QLabel("SCPI命令:")
        self.cmd_input = QLineEdit()
        self.cmd_send = QPushButton("发送")
        self.cmd_send.clicked.connect(self.send_scpi)

        cmd_layout.addWidget(self.cmd_edit)
        cmd_layout.addWidget(self.cmd_input)
        cmd_layout.addWidget(self.cmd_send)
        layout.addLayout(cmd_layout)

        self.load_saved_path()

    def show_notice(self, title: str, message: str):
        pass

    def load_saved_path(self):
        saved = self.settings.value("save_dir", "", type=str)
        if saved and os.path.isdir(saved):
            self.save_dir = saved
        else:
            self.save_dir = None
            if saved:
                self.settings.remove("save_dir")
        self.refresh_save_path_button()

    def persist_save_path(self):
        if self.save_dir and os.path.isdir(self.save_dir):
            self.settings.setValue("save_dir", self.save_dir)
        else:
            self.settings.remove("save_dir")

    def refresh_save_path_button(self):
        if self.save_dir and os.path.isdir(self.save_dir):
            self.btn_path.setText(self.save_dir)
        else:
            self.btn_path.setText("请设置保存路径")

    def query_acquire_status(self, scope) -> dict:
        profile = scope.profile

        mode = normalize_acq_mode(_q(scope, profile.acquire_mode_query))
        state = normalize_acq_state(_q(scope, profile.acquire_state_query))
        numavg = int(float(extract_last_value(_q(scope, profile.acquire_numavg_query))))
        numacq = int(float(extract_last_value(_q(scope, profile.acquire_numacq_query))))
        stopafter = normalize_stopafter(_q(scope, profile.acquire_stopafter_query))
        trig_state = normalize_trigger_state(_q(scope, profile.trigger_state_query))

        return {
            "mode": mode,
            "state": state,
            "numavg": numavg,
            "numacq": numacq,
            "stopafter": stopafter,
            "trig_state": trig_state,
        }

    def update_status_label(self, extra_lines=None):
        if not self.resource_str or not self.idn:
            self.status_label.setText("设备：未连接 / 未锁定")
            return

        if extra_lines is not None:
            self.status_extra_lines = list(extra_lines)

        lines = [
            f"设备：已连接 [{self.device_serial or SERIAL_LOCK}]",
            f"型号：{self.profile_name or 'UNKNOWN'}",
            f"资源：{self.resource_str}",
            f"IDN：{self.idn}",
        ]
        if self.status_extra_lines:
            lines.extend(self.status_extra_lines)
        self.status_label.setText("\n".join(lines))

    def on_acquire_applied(self, mode: str, numavg: int):
        self.current_acq_mode = mode.upper()
        self.current_numavg = numavg
        self.current_stopafter = "RUNST"
        self.run_numacq_base = None
        self.progress_timer.stop()
        self.awaiting_run_after_apply = (self.current_acq_mode == "AVERAGE")

        extra = [f"采集模式：{self.current_acq_mode}"]
        if self.current_acq_mode == "AVERAGE":
            extra.append(f"平均次数：{self.current_numavg}")
            extra.append("状态：参数已应用，尚未点击 RUN")
        self.update_status_label(extra)

    @staticmethod
    def _is_scope_running(acq: dict) -> bool:
        """
        运行状态以 ACQ:STATE? 为主。
        TRIG:STATE? 只作为显示信息，不再用来否定 RUN。
        """
        return str(acq.get("state", "")).strip() == "1"

    @staticmethod
    def _average_progress_from_numacq(acq: dict) -> int:
        """
        平均进度直接使用示波器返回的 ACQ:NUMACQ?。

        MDO34 屏幕上的平均进度与 NUMACQ 一致，所以这里不再做
        当前 NUMACQ - run_numacq_base 的差值计算。
        """
        numacq = int(acq.get("numacq", 0))
        numavg = int(acq.get("numavg", numacq))
        return max(0, min(numacq, numavg))

    def _restart_progress_timer_if_running(self, acq: dict):
        if self._is_scope_running(acq):
            if not self.progress_timer.isActive():
                self.progress_timer.start()
        else:
            self.progress_timer.stop()

    def refresh_acquire_progress(self):
        if not self.resource_str:
            self.progress_timer.stop()
            return

        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq = self.query_acquire_status(scope)
            self.status_refresh_errors = 0

            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = acq["stopafter"]
            trig_state = acq["trig_state"]
            is_running = self._is_scope_running(acq)

            extra = [f"采集模式：{self.current_acq_mode}"]
            if acq["mode"] == "AVERAGE":
                progress = self._average_progress_from_numacq(acq)
                extra.append(f"平均进度：{progress}/{acq['numavg']}")

                if is_running:
                    label = "SINGLE 运行中" if acq["stopafter"] == "SEQ" else "RUN 中"
                    extra.append(f"状态：{label} ({trig_state})")
                    if not self.progress_timer.isActive():
                        self.progress_timer.start()
                else:
                    extra.append(f"状态：已停止 ({trig_state})")
                    self.progress_timer.stop()
            else:
                if is_running:
                    label = "SINGLE 运行中" if acq["stopafter"] == "SEQ" else "RUN 中"
                    extra.append(f"状态：{label} ({trig_state})")
                    if not self.progress_timer.isActive():
                        self.progress_timer.start()
                else:
                    extra.append(f"状态：已停止 ({trig_state})")
                    self.progress_timer.stop()

            extra.append(f"原始状态：STATE={acq['state']} TRIG={trig_state} NUMACQ={acq['numacq']}")
            self.update_status_label(extra)

        except Exception as ex:
            self.status_refresh_errors += 1
            if self.status_refresh_errors >= STATUS_POLL_MAX_ERRORS:
                self.progress_timer.stop()
                extra = list(self.status_extra_lines or [])
                extra.append(f"状态刷新失败：{ex}")
                self.update_status_label(extra)
        finally:
            try:
                if scope:
                    scope.close()
                if rm:
                    rm.close()
            except Exception:
                pass


    def close_persistent_save_session(self):
        """关闭保存专用的常驻 VISA 会话。"""
        scope = getattr(self, "_save_scope", None)
        rm = getattr(self, "_save_rm", None)
        try:
            if scope:
                try:
                    scope.close()
                except Exception:
                    pass
            if rm:
                try:
                    rm.close()
                except Exception:
                    pass
        finally:
            self._save_scope = None
            self._save_rm = None
            self._save_resource_str = None

    def get_persistent_save_scope(self):
        """
        返回保存专用常驻会话。

        参考版本的加速点是保存时使用 open_scope_fast() 避免重复 *IDN?。
        这里再进一步：第一次保存打开会话，后续保存复用同一个 VISA 会话，
        因此“打开/关闭会话”耗时会接近 0。
        """
        if (
            self._save_scope is not None
            and self._save_rm is not None
            and self._save_resource_str == self.resource_str
        ):
            try:
                # 只做本地属性访问，不向示波器发查询命令。
                # 若 VISA 后端已经把句柄置为无效，这里可能直接抛异常。
                _ = self._save_scope.session
                return self._save_rm, self._save_scope, True
            except Exception:
                self.close_persistent_save_session()

        self.close_persistent_save_session()
        rm, scope = open_scope_fast(self.resource_str, self.profile_name)
        self._save_rm = rm
        self._save_scope = scope
        self._save_resource_str = self.resource_str
        return rm, scope, False

    def with_scope(self):
        if not self.resource_str:
            raise RuntimeError("示波器未锁定")
        rm, scope, _idn = open_scope(self.resource_str)
        return rm, scope

    def open_acquire(self):
        # RUN/AVERAGE 状态下 progress_timer 会周期性查询示波器。
        # 如果 Acquire 对话框持有一个 scope 会话，同时定时器又打开/关闭另一个会话，
        # 某些 VISA 后端会把前一个会话句柄变成 invalid session handle。
        # 因此进入 Acquire 设置前先暂停状态轮询，并关闭保存专用常驻会话。
        self.progress_timer.stop()
        self.close_persistent_save_session()

        rm = scope = None
        try:
            rm, scope = self.with_scope()
            dlg = AcquireDialog(scope, self)
            dlg.exec_()

            try:
                acq = self.query_acquire_status(scope)
                self.current_acq_mode = acq["mode"]
                self.current_numavg = acq["numavg"]
                self.current_stopafter = acq["stopafter"]
                state_label = "RUN 中" if self._is_scope_running(acq) else "已停止"
                extra = [f"采集模式：{acq['mode']}"]
                if acq["mode"] == "AVERAGE":
                    progress = self._average_progress_from_numacq(acq)
                    extra.append(f"平均进度：{progress}/{acq['numavg']}")
                extra.extend([
                    f"状态：{state_label} ({acq['trig_state']})",
                    f"原始状态：STATE={acq['state']} TRIG={acq['trig_state']} NUMACQ={acq['numacq']}",
                ])
                self.update_status_label(extra)
                self._restart_progress_timer_if_running(acq)
            except Exception:
                pass
        except Exception as e:
            QMessageBox.critical(self, "Acquire", str(e))
        finally:
            try:
                if scope:
                    scope.close()
                if rm:
                    rm.close()
            except Exception:
                pass

    def do_run(self):
        self.close_persistent_save_session()
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq_before = self.query_acquire_status(scope)
            self.current_acq_mode = acq_before["mode"]
            self.current_numavg = acq_before["numavg"]
            self.current_stopafter = "RUNST"

            _w(scope, scope.profile.stopafter_run_cmd)
            _w(scope, scope.profile.acq_run_cmd)
            time.sleep(0.08)

            try:
                acq_after = self.query_acquire_status(scope)
            except Exception:
                acq_after = dict(acq_before)
                acq_after["state"] = "1"
                acq_after["stopafter"] = "RUNST"
                acq_after["trig_state"] = "UNKNOWN"

            self.current_acq_mode = acq_after["mode"]
            self.current_numavg = acq_after["numavg"]
            self.current_stopafter = "RUNST"
            self.run_numacq_base = None

            self.awaiting_run_after_apply = False
            self.status_refresh_errors = 0
            if self.current_acq_mode == "AVERAGE":
                progress = self._average_progress_from_numacq(acq_after)
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"平均进度：{progress}/{self.current_numavg}",
                    f"状态：RUN 中 ({acq_after['trig_state']})",
                    f"原始状态：STATE={acq_after['state']} TRIG={acq_after['trig_state']} NUMACQ={acq_after['numacq']}",
                ])
                self.progress_timer.start()
            else:
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"状态：RUN 中 ({acq_after['trig_state']})",
                    f"原始状态：STATE={acq_after['state']} TRIG={acq_after['trig_state']} NUMACQ={acq_after['numacq']}",
                ])
            self.show_notice("RUN", "示波器已开始运行")
        except Exception as e:
            QMessageBox.critical(self, "RUN 失败", str(e))
        finally:
            if scope:
                scope.close()
            if rm:
                rm.close()

    def do_single(self):
        self.close_persistent_save_session()
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq_before = self.query_acquire_status(scope)
            self.current_acq_mode = acq_before["mode"]
            self.current_numavg = acq_before["numavg"]
            self.current_stopafter = "SEQ"

            _w(scope, scope.profile.stopafter_single_cmd)
            _w(scope, scope.profile.acq_run_cmd)
            time.sleep(0.08)

            try:
                acq_after = self.query_acquire_status(scope)
            except Exception:
                acq_after = dict(acq_before)
                acq_after["state"] = "1"
                acq_after["stopafter"] = "SEQ"
                acq_after["trig_state"] = "UNKNOWN"

            self.current_acq_mode = acq_after["mode"]
            self.current_numavg = acq_after["numavg"]
            self.current_stopafter = "SEQ"
            self.run_numacq_base = None

            self.awaiting_run_after_apply = False
            self.status_refresh_errors = 0
            if self.current_acq_mode == "AVERAGE":
                progress = self._average_progress_from_numacq(acq_after)
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"平均进度：{progress}/{self.current_numavg}",
                    f"状态：SINGLE 运行中 ({acq_after['trig_state']})",
                    f"原始状态：STATE={acq_after['state']} TRIG={acq_after['trig_state']} NUMACQ={acq_after['numacq']}",
                ])
            else:
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"状态：SINGLE 运行中 ({acq_after['trig_state']})",
                    f"原始状态：STATE={acq_after['state']} TRIG={acq_after['trig_state']} NUMACQ={acq_after['numacq']}",
                ])
            self.progress_timer.start()
            self.show_notice("SINGLE", "示波器已开始单次采集")
        except Exception as e:
            QMessageBox.critical(self, "SINGLE 失败", str(e))
        finally:
            if scope:
                scope.close()
            if rm:
                rm.close()

    def do_stop(self):
        self.close_persistent_save_session()
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            _w(scope, scope.profile.acq_stop_cmd)
            self.progress_timer.stop()
            self.awaiting_run_after_apply = False

            acq = self.query_acquire_status(scope)
            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = acq["stopafter"]
            extra = [f"采集模式：{self.current_acq_mode}"]
            if self.current_acq_mode == "AVERAGE":
                progress = self._average_progress_from_numacq(acq)
                extra.append(f"平均进度：{progress}/{acq['numavg']}")
            extra.append("状态：已停止")
            self.update_status_label(extra)
            self.show_notice("STOP", "示波器已停止")
        except Exception as e:
            QMessageBox.critical(self, "STOP 失败", str(e))
        finally:
            if scope:
                scope.close()
            if rm:
                rm.close()

    def send_scpi(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        self.close_persistent_save_session()
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            if cmd.endswith("?"):
                resp = scope.query(cmd)
                QMessageBox.information(self, "SCPI 返回", resp.strip())
            else:
                scope.write(cmd)
                QMessageBox.information(self, "SCPI 发送", "命令已发送")
        except Exception as e:
            QMessageBox.critical(self, "SCPI 错误", str(e))
        finally:
            if scope:
                scope.close()
            if rm:
                rm.close()

    def _connect_resource(self, resource_str: str, mode_name: str):
        self.close_persistent_save_session()
        rm = scope = None
        try:
            rm, scope, idn = open_scope(resource_str)
            maker, model, serial, firmware = _parse_idn(idn)

            self.resource_str = resource_str
            self.idn = idn.strip()
            self.profile_name = scope.profile.name
            self.device_serial = serial
            self.settings.setValue("resource_str", self.resource_str)
            self.settings.setValue("connection_mode", mode_name)

            acq = self.query_acquire_status(scope)
            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = acq["stopafter"]

            extra = [
                f"连接方式：{mode_name}",
                f"设备类型：{scope.profile.name}",
                f"采集模式：{self.current_acq_mode}",
            ]
            if self.current_acq_mode == "AVERAGE":
                extra.append(f"平均次数：{self.current_numavg}")
            if self._is_scope_running(acq):
                if acq["stopafter"] == "SEQ":
                    extra.append(f"状态：SINGLE 运行中 ({acq['trig_state']})")
                else:
                    extra.append(f"状态：RUN 中 ({acq['trig_state']})")
            else:
                extra.append("状态：已停止")
            self.update_status_label(extra)
        finally:
            if scope:
                scope.close()
            if rm:
                rm.close()

    def _clear_connection_state(self):
        self.close_persistent_save_session()
        self.resource_str = None
        self.idn = None
        self.profile_name = None
        self.device_serial = None
        self.status_label.setText("设备：未连接 / 未锁定")

    def try_lock_scope_usb(self):
        try:
            cached_resource = self.settings.value("resource_str", "", type=str)
            device_info = discover_scope_resource_fast(
                SERIAL_LOCK,
                cached_resource=cached_resource,
                include_usb=True,
                include_tcpip=False,
            )
            self._connect_resource(device_info["resource"], "USB")
        except Exception as e:
            self._clear_connection_state()
            QMessageBox.critical(
                self,
                "USB 连接失败",
                f"无法通过 USB 自动识别示波器（序列号 {SERIAL_LOCK}）。\n\n{e}"
            )

    def try_lock_scope_lan(self):
        try:
            manual_text = self.ip_input.text().strip()
            resource = build_tcpip_resource(manual_text)
            if not resource:
                QMessageBox.warning(self, "网口连接", "请先输入示波器 IP 或完整 VISA 资源。")
                return

            self.settings.setValue("scope_ip", manual_text)
            self._connect_resource(resource, "LAN")
        except Exception as e:
            self._clear_connection_state()
            QMessageBox.critical(
                self,
                "网口连接失败",
                f"无法通过网口连接示波器。\n\n当前资源：{build_tcpip_resource(self.ip_input.text())}\n\n{e}"
            )

    def try_lock_scope(self):
        try:
            cached_resource = self.settings.value("resource_str", "", type=str)
            manual_resource = build_tcpip_resource(self.ip_input.text())
            if manual_resource:
                self.settings.setValue("scope_ip", self.ip_input.text().strip())
            else:
                self.settings.remove("scope_ip")

            device_info = discover_scope_resource_fast(SERIAL_LOCK, cached_resource, manual_resource)
            self.resource_str = device_info["resource"]
            self.idn = device_info["idn"].strip()
            self.profile_name = device_info["profile"].name
            self.device_serial = device_info["serial"]
            self.settings.setValue("resource_str", self.resource_str)

            rm = scope = None
            try:
                rm, scope = self.with_scope()
                acq = self.query_acquire_status(scope)
                self.current_acq_mode = acq["mode"]
                self.current_numavg = acq["numavg"]
                self.current_stopafter = acq["stopafter"]

                extra = [
                    f"设备类型：{scope.profile.name}",
                    f"采集模式：{self.current_acq_mode}",
                ]
                if self.current_acq_mode == "AVERAGE":
                    extra.append(f"平均次数：{self.current_numavg}")
                if self._is_scope_running(acq):
                    if acq["stopafter"] == "SEQ":
                        extra.append(f"状态：SINGLE 运行中 ({acq['trig_state']})")
                    else:
                        extra.append(f"状态：RUN 中 ({acq['trig_state']})")
                else:
                    extra.append("状态：已停止")
                self.update_status_label(extra)
            finally:
                if scope:
                    scope.close()
                if rm:
                    rm.close()

        except Exception as e:
            self.resource_str = None
            self.idn = None
            self.profile_name = None
            self.device_serial = None
            self.status_label.setText("设备：未连接 / 未锁定")
            QMessageBox.critical(
                self,
                "连接失败",
                f"无法锁定示波器（序列号 {SERIAL_LOCK}）\n\n{e}"
            )

    def choose_path(self):
        d = QFileDialog.getExistingDirectory(self, "选择保存文件夹", self.save_dir or "")
        if d:
            self.save_dir = d
            self.persist_save_path()
            self.refresh_save_path_button()

    def on_save_format_changed(self):
        if not hasattr(self, "save_format_combo"):
            return
        fmt = normalize_save_format(self.save_format_combo.currentData())
        self.settings.setValue("save_format", fmt)
        if hasattr(self, "btn_save"):
            self.btn_save.setText("保存NPZ" if fmt == "NPZ" else "保存CSV")

    def choose_channels(self):
        dlg = ChannelDialog(self.selected_channels, self)
        if dlg.exec_() == QDialog.Accepted:
            sel = dlg.get_selected()
            if not sel:
                QMessageBox.warning(self, "提示", "至少选择一个通道。")
                return
            self.selected_channels = sel
            self.btn_ch.setText("通道：" + ",".join(sorted(self.selected_channels)))

    def do_save(self):
        if self.save_dir and not os.path.isdir(self.save_dir):
            self.save_dir = None
            self.persist_save_path()
            self.refresh_save_path_button()
        if not self.save_dir:
            QMessageBox.warning(self, "提示", "请先选择保存文件夹")
            return
        if not self.resource_str:
            QMessageBox.warning(self, "提示", "设备未锁定，无法保存")
            return
        if self.awaiting_run_after_apply and self.current_acq_mode == "AVERAGE":
            ans = QMessageBox.question(
                self,
                "尚未运行",
                f"平均参数已应用，但还没有点击 RUN。\n当前目标平均次数：{self.current_numavg}\n\n是否仍然继续保存？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                return

        os.makedirs(self.save_dir, exist_ok=True)
        idx = next_index(self.save_dir)
        save_format = normalize_save_format(self.save_format_combo.currentData())
        self.settings.setValue("save_format", save_format)
        save_path = os.path.join(self.save_dir, make_filename(idx, self.selected_channels, save_format))

        rm = scope = None
        save_t0 = time.perf_counter()
        open_elapsed = 0.0
        stop_elapsed = 0.0
        write_elapsed = 0.0
        channel_timings = []
        save_failed = False
        reused_session = False

        try:
            print(
                f"[保存计时] 开始保存 idx={idx:04d}, 格式={save_format}, "
                f"通道={','.join(sorted(self.selected_channels))}, 目标={save_path}",
                flush=True,
            )

            t_open0 = time.perf_counter()
            rm, scope, reused_session = self.get_persistent_save_scope()
            open_elapsed = time.perf_counter() - t_open0

            # Save path uses the detected profile's fixed stop command only.
            t_stop0 = time.perf_counter()
            _w(scope, scope.profile.acq_stop_cmd)
            time.sleep(SAVE_STOP_SETTLE_S)
            stop_elapsed = time.perf_counter() - t_stop0

            waveform_data = {}
            base_time = None
            debug_lines = []
            meta_by_channel = {}

            for ch in sorted(self.selected_channels):
                ch_t0 = time.perf_counter()
                t, v, meta = read_waveform(scope, ch)
                ch_elapsed = time.perf_counter() - ch_t0
                channel_timings.append((ch, ch_elapsed, int(meta.get("recl", len(v)))))
                meta_by_channel[ch] = dict(meta)

                print(
                    f"[保存计时] {ch} 读取完成: {ch_elapsed:.3f} s, "
                    f"N={meta.get('recl', len(v))}, scope={meta.get('scope', 'UNKNOWN')}",
                    flush=True,
                )

                if base_time is None:
                    base_time = t
                    waveform_data["TIME"] = t
                else:
                    if len(t) != len(base_time) or not np.allclose(t, base_time, rtol=0.0, atol=1e-15):
                        raise RuntimeError(f"{ch} 的时间轴与其他通道不一致，已停止合并保存")

                waveform_data[ch] = v
                record_detail = ""
                if "record_length" in meta:
                    record_detail = (
                        f" record_length={meta['record_length']}"
                        f" DATA:START={meta.get('data_start', 'N/A')}"
                        f" DATA:STOP={meta.get('data_stop', 'N/A')}"
                    )
                debug_lines.append(
                    f"{ch}: scope={meta['scope']} composition={meta['composition']} "
                    f"width={meta['width']} dtype={meta['dtype']} "
                    f"BYT_OR={meta['byt_or']} BN_FMT={meta['bn_fmt']} ENC={meta['data_enc']} "
                    f"N={meta['recl']}{record_detail} XINCR={meta['xincr']}"
                )

            t_write0 = time.perf_counter()
            save_waveform_file(
                save_path,
                waveform_data,
                save_format=save_format,
                meta={
                    "save_format": save_format,
                    "channels": sorted(self.selected_channels),
                    "scope_profile": self.profile_name,
                    "scope_resource": self.resource_str,
                    "scope_idn": self.idn,
                    "format": "npz_lossless_numpy_arrays" if save_format == "NPZ" else "csv_pandas_default",
                    "compression": "none" if save_format == "NPZ" else "N/A",
                    "channel_meta": meta_by_channel,
                },
            )
            write_elapsed = time.perf_counter() - t_write0

            total_elapsed = time.perf_counter() - save_t0
            read_elapsed = sum(item[1] for item in channel_timings)
            timing_lines = [
                f"保存会话: {open_elapsed:.3f} s" + (" (复用)" if reused_session else " (新开)"),
                f"停止采集等待: {stop_elapsed:.3f} s",
                f"波形读取合计: {read_elapsed:.3f} s",
                f"{save_format}写入: {write_elapsed:.3f} s",
                f"总耗时: {total_elapsed:.3f} s",
                (
                    "保存格式: NPZ无压缩二进制，保持numpy数组原始dtype，无文本精度截断"
                    if save_format == "NPZ"
                    else "保存格式: CSV，pandas默认to_csv，未指定float_format"
                ),
            ]
            for ch, elapsed, npts in channel_timings:
                timing_lines.append(f"{ch}读取: {elapsed:.3f} s, N={npts}")

            print("[保存计时] " + " | ".join(timing_lines), flush=True)
            print(f"[保存计时] 完成: {save_path}", flush=True)

            self.show_notice("保存完成", save_path)
            done_box = QMessageBox(self)
            done_box.setWindowTitle("保存完成")
            done_box.setIcon(QMessageBox.NoIcon)
            done_box.setText(
                "已保存：\n"
                + save_path
                + "\n\n"
                + "\n".join(debug_lines)
                + "\n\n保存计时：\n"
                + "\n".join(timing_lines)
            )
            done_box.exec_()

        except Exception as e:
            save_failed = True
            total_elapsed = time.perf_counter() - save_t0
            print(f"[保存计时] 保存失败，已耗时: {total_elapsed:.3f} s，错误: {e}", flush=True)
            QMessageBox.critical(self, "保存失败", f"{e}\n\n{traceback.format_exc()}")

        finally:
            # 成功时保留保存专用会话，下次保存复用。
            # 失败时关闭，下一次重新打开干净会话。
            if save_failed:
                self.close_persistent_save_session()



    def closeEvent(self, event):
        self.close_persistent_save_session()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(620, 260)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
