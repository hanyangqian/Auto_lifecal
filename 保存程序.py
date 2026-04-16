# -*- coding: utf-8 -*-
"""
Tektronix DPO/TDS (pyvisa) waveform saver with simple UI (3 buttons)
- Auto-discover & lock serial: C030795
- Save path button (shows chosen folder)
- Channel select button (multi-select CH1~CH4)
- Save button: T0000CH1.CSV naming, increasing index from 0

Requirements:
  pip install pyvisa pyqt5 numpy pandas
NI-VISA must be installed, instrument connected.
"""

import os
import re
import sys
import traceback
from contextlib import contextmanager
import numpy as np
import pandas as pd
import pyvisa
import time

from PyQt5.QtCore import Qt, QTimer, QSettings
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox,
    QFileDialog, QDialog, QDialogButtonBox, QCheckBox, QLabel, QHBoxLayout,
    QLineEdit, QComboBox
)

SERIAL_LOCK = "C030795"
DEFAULT_TIMEOUT_MS = 20000
CURVE_TIMEOUT_MS = 3000
DISCOVERY_TIMEOUT_MS = 1500


class AcquireDialog(QDialog):
    def __init__(self, scope, parent=None):
        super().__init__(parent)
        self.scope = scope
        self.setWindowTitle("Acquire 设置")

        layout = QVBoxLayout(self)

        # ====== 读取 ACQ? ======
        resp = scope.query("ACQ?").strip()
        stopafter, state, mode, numavg, samplerate = resp.split(";")

        self.cb_mode = QComboBox()
        self.cb_mode.addItem("普通采样", "SAMPLE")
        self.cb_mode.addItem("平均", "AVERAGE")

        self.cb_avg = QComboBox()
        for v in ["2", "4", "8", "16", "32", "64", "128", "256", "512"]:
            self.cb_avg.addItem(v, v)

        def _set_by_data(combo: QComboBox, data_value: str):
            for i in range(combo.count()):
                if str(combo.itemData(i)) == str(data_value):
                    combo.setCurrentIndex(i)
                    return

        _set_by_data(self.cb_mode, "AVERAGE" if mode == "AVE" else mode)
        _set_by_data(self.cb_avg, numavg)
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

    def _sync_average_controls(self):
        is_average = self.cb_mode.currentData() == "AVERAGE"
        self.cb_avg.setEnabled(is_average)
        self.cb_avg.setToolTip("" if is_average else "普通采样模式下不使用平均次数")

    def apply(self):
        try:
            self.scope.write("ACQ:STOPAFTER RUNST")
            mode = self.cb_mode.currentData()
            self.scope.write(f"ACQ:MODE {mode}")
            numavg = int(self.cb_avg.currentData())
            if mode == "AVERAGE":
                self.scope.write(f"ACQ:NUMAVG {numavg}")
            parent = self.parent()
            if parent and hasattr(parent, "on_acquire_applied"):
                parent.on_acquire_applied(mode, numavg)
            if parent and hasattr(parent, "show_notice"):
                parent.show_notice("Acquire", "设置已应用")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"写入失败：\n{e}")


# =========================
# VISA / Scope helper
# =========================
def discover_scope_resource(serial_lock: str) -> str:
    rm = pyvisa.ResourceManager()
    try:
        resources = rm.list_resources()
    finally:
        rm.close()

    candidates = [r for r in resources if serial_lock in r]
    if candidates:
        usb = [r for r in candidates if r.upper().startswith("USB")]
        return usb[0] if usb else candidates[0]

    tek = [r for r in resources if "0x0699" in r.lower() or "0699" in r.lower()]
    if tek:
        return tek[0]

    raise RuntimeError(
        f"未发现包含序列号 {serial_lock} 的设备资源。当前可见资源：\n" + "\n".join(resources)
    )


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
        scope.read_termination = '\n'
        scope.write_termination = '\n'
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
            raise RuntimeError("未发现任何 VISA 资源。请确认示波器已开机，且 NI-VISA 已安装。")

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
            options = "\n".join(
                f"{item['resource']} -> {item['idn']}" for item in matches
            )
            raise RuntimeError(
                f"发现了多台 Tek 设备，但没有找到序列号为 {serial_lock} 的目标设备：\n{options}"
            )

        resource_list = "\n".join(resources)
        error_text = "\n".join(probe_errors[:8]) if probe_errors else "(无可用探测结果)"
        raise RuntimeError(
            f"未找到可用的 Tek 示波器。当前 VISA 资源：\n{resource_list}\n\n探测结果：\n{error_text}"
        )
    finally:
        rm.close()


def open_scope(resource_str: str):
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(resource_str)

    scope.timeout = DEFAULT_TIMEOUT_MS
    scope.read_termination = '\n'
    scope.write_termination = '\n'

    idn = scope.query("*IDN?")
    return rm, scope, idn


def clear_session(scope, drain=True):
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


# =========================
# File naming / indexing
# =========================
FNAME_RE = re.compile(r"^T(\d{4})(CH[1-4](?:CH[1-4])*)?\.CSV$", re.IGNORECASE)

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

def make_filename(idx: int, channels) -> str:
    suffix = "".join(sorted(channels))
    return f"T{idx:04d}{suffix}.CSV"

def save_csv(save_path: str, data: dict):
    df = pd.DataFrame(data)
    df.to_csv(save_path, index=False)


# =========================
# UI dialogs
# =========================
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
# Main UI
# =========================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tek 波形保存（T0000CHx.CSV）")

        self.settings = QSettings("Codex", "TekWaveSaver")
        self.selected_channels = {"CH1"}
        self.resource_str = None
        self.idn = None
        self.awaiting_run_after_apply = False
        self.current_acq_mode = None
        self.current_numavg = None
        self.current_stopafter = None
        self.run_numacq_base = None
        self.status_extra_lines = []
        self.progress_timer = QTimer(self)
        self.progress_timer.setInterval(300)
        self.progress_timer.timeout.connect(self.refresh_acquire_progress)

        layout = QVBoxLayout(self)

        self.status_label = QLabel("设备：未连接")
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.status_label)

        self.save_dir = None
        self.btn_path = QPushButton("请设置保存路径")
        self.btn_path.clicked.connect(self.choose_path)
        layout.addWidget(self.btn_path)

        self.btn_ch = QPushButton("通道：" + ",".join(sorted(self.selected_channels)))
        self.btn_ch.clicked.connect(self.choose_channels)
        layout.addWidget(self.btn_ch)

        self.btn_save = QPushButton("保存")
        self.btn_save.clicked.connect(self.do_save)
        layout.addWidget(self.btn_save)

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

    def update_status_label(self, extra_lines=None):
        if not self.resource_str or not self.idn:
            self.status_label.setText("设备：未连接 / 未锁定")
            return

        if extra_lines is not None:
            self.status_extra_lines = list(extra_lines)

        lines = [
            f"设备：已锁定 [{SERIAL_LOCK}]",
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

    def refresh_acquire_progress(self):
        if not self.resource_str:
            self.progress_timer.stop()
            return

        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq = self.query_acquire_status(scope)
            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = acq["stopafter"]
            trig_state = acq["trig_state"]
            is_running = acq["state"] == "1" and trig_state != "SAVE"

            extra = [f"采集模式：{self.current_acq_mode}"]
            if acq["mode"] == "AVERAGE":
                if self.run_numacq_base is None:
                    self.run_numacq_base = acq["numacq"]
                progress = max(0, acq["numacq"] - self.run_numacq_base)
                progress = min(progress, acq["numavg"])
                extra.append(f"平均进度：{progress}/{acq['numavg']}")
                if is_running:
                    if acq["stopafter"] == "SEQ":
                        extra.append(f"状态：SINGLE 运行中 ({trig_state})")
                    else:
                        extra.append(f"状态：RUN 中 ({trig_state})")
                else:
                    extra.append("状态：已停止")
                    self.progress_timer.stop()
            else:
                if is_running:
                    if acq["stopafter"] == "SEQ":
                        extra.append(f"状态：SINGLE 运行中 ({trig_state})")
                    else:
                        extra.append(f"状态：RUN 中 ({trig_state})")
                else:
                    extra.append("状态：已停止")
                    self.progress_timer.stop()

            self.update_status_label(extra)
        except Exception:
            self.progress_timer.stop()
        finally:
            try:
                if scope:
                    scope.close()
                if rm:
                    rm.close()
            except Exception:
                pass

    def with_scope(self):
        if not self.resource_str:
            raise RuntimeError("示波器未锁定")
        rm, scope, _ = open_scope(self.resource_str)
        return rm, scope

    def open_acquire(self):
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            dlg = AcquireDialog(scope, self)
            dlg.exec_()
        except Exception as e:
            QMessageBox.critical(self, "Acquire", str(e))
        finally:
            try:
                if scope: scope.close()
                if rm: rm.close()
            except Exception:
                pass

    def do_run(self):
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq = self.query_acquire_status(scope)
            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = "RUNST"
            self.run_numacq_base = acq["numacq"]
            scope.write("ACQ:STOPAFTER RUNST")
            scope.write("ACQ:STATE RUN")
            self.awaiting_run_after_apply = False
            if self.current_acq_mode == "AVERAGE":
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"平均进度：0/{self.current_numavg}",
                    "状态：RUN 中",
                ])
                self.progress_timer.start()
            else:
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    "状态：RUN 中",
                ])
            self.show_notice("RUN", "示波器已开始运行")
        finally:
            if scope: scope.close()
            if rm: rm.close()

    def do_single(self):
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            acq = self.query_acquire_status(scope)
            self.current_acq_mode = acq["mode"]
            self.current_numavg = acq["numavg"]
            self.current_stopafter = "SEQ"
            self.run_numacq_base = acq["numacq"]
            scope.write("ACQ:STOPAFTER SEQ")
            scope.write("ACQ:STATE RUN")
            self.awaiting_run_after_apply = False
            if self.current_acq_mode == "AVERAGE":
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    f"平均进度：0/{self.current_numavg}",
                    "状态：SINGLE 等待触发",
                ])
                self.progress_timer.start()
            else:
                self.update_status_label([
                    f"采集模式：{self.current_acq_mode}",
                    "状态：SINGLE 等待触发",
                ])
                self.progress_timer.start()
            self.show_notice("SINGLE", "示波器已开始单次采集")
        finally:
            if scope: scope.close()
            if rm: rm.close()

    def do_stop(self):
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            scope.write("ACQ:STATE STOP")
            self.progress_timer.stop()
            self.awaiting_run_after_apply = False
            try:
                acq = self.query_acquire_status(scope)
                self.current_acq_mode = acq["mode"]
                self.current_numavg = acq["numavg"]
                self.current_stopafter = acq["stopafter"]
                extra = [f"采集模式：{self.current_acq_mode}"]
                if self.current_acq_mode == "AVERAGE" and self.run_numacq_base is not None:
                    progress = max(0, acq["numacq"] - self.run_numacq_base)
                    progress = min(progress, acq["numavg"])
                    extra.append(f"平均进度：{progress}/{acq['numavg']}")
                extra.append("状态：已停止")
                self.update_status_label(extra)
            except Exception:
                pass
            self.show_notice("STOP", "示波器已停止")
        finally:
            if scope: scope.close()
            if rm: rm.close()

    def send_scpi(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        rm = scope = None
        try:
            rm, scope = self.with_scope()
            if cmd.endswith("?"):
                resp = scope.query(cmd)
                self.show_notice("SCPI 返回", resp.strip())
            else:
                scope.write(cmd)
                self.show_notice("SCPI 发送", "命令已发送")
        except Exception as e:
            QMessageBox.critical(self, "SCPI 错误", str(e))
        finally:
            if scope: scope.close()
            if rm: rm.close()

    def try_lock_scope(self):
        try:
            device_info = discover_scope_resource_robust(SERIAL_LOCK)
            self.resource_str = device_info["resource"]
            self.idn = device_info["idn"].strip()
            rm = scope = None
            try:
                rm, scope = self.with_scope()
                acq = self.query_acquire_status(scope)
                self.current_acq_mode = acq["mode"]
                self.current_numavg = acq["numavg"]
                self.current_stopafter = acq["stopafter"]

                extra = [f"采集模式：{self.current_acq_mode}"]
                if self.current_acq_mode == "AVERAGE":
                    extra.append(f"平均次数：{self.current_numavg}")
                if acq["state"] == "1" and acq["trig_state"] != "SAVE":
                    if acq["stopafter"] == "SEQ":
                        extra.append(f"状态：SINGLE 运行中 ({acq['trig_state']})")
                    else:
                        extra.append(f"状态：RUN 中 ({acq['trig_state']})")
                else:
                    extra.append("状态：已停止")
                self.update_status_label(extra)
            finally:
                try:
                    if scope:
                        scope.close()
                    if rm:
                        rm.close()
                except Exception:
                    pass
        except Exception as e:
            self.resource_str = None
            self.idn = None
            self.status_label.setText("设备：未连接 / 未锁定")
            QMessageBox.critical(self, "连接失败",
                                 f"无法锁定示波器（序列号 {SERIAL_LOCK}）\n\n{e}")

    def choose_path(self):
        d = QFileDialog.getExistingDirectory(self, "选择保存文件夹", self.save_dir)
        if d:
            self.save_dir = d
            self.persist_save_path()
            self.refresh_save_path_button()

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
        csv_path = os.path.join(self.save_dir, make_filename(idx, self.selected_channels))

        rm = scope = None
        try:
            rm, scope, _ = open_scope(self.resource_str)
            clear_session(scope, drain=True)

            _w(scope, "ACQ:STATE STOP")
            time.sleep(0.35)

            csv_data = {}
            base_time = None
            debug_lines = []

            for ch in sorted(self.selected_channels):
                t, v, meta = read_waveform(scope, ch)
                if base_time is None:
                    base_time = t
                    csv_data["TIME"] = t
                else:
                    if len(t) != len(base_time) or not np.allclose(t, base_time, rtol=0.0, atol=1e-15):
                        raise RuntimeError(f"{ch} 的时间轴与其他通道不一致，已停止合并保存")

                csv_data[ch] = v

                debug_lines.append(
                    f"{ch}: {meta['composition']} width={meta['width']} dtype={meta['dtype']} "
                    f"BYT_OR={meta['byt_or']} BN_FMT={meta['bn_fmt']} ENC={meta['data_enc']} "
                    f"N={meta['recl']} XINCR={meta['xincr']}"
                )

            save_csv(csv_path, csv_data)

            self.show_notice("保存完成", csv_path)
            done_box = QMessageBox(self)
            done_box.setWindowTitle("保存完成")
            done_box.setIcon(QMessageBox.NoIcon)
            done_box.setText(f"已保存：\n{csv_path}")
            done_box.exec_()

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


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(520, 220)
    w.show()

    from PyQt5.QtCore import QTimer
    QTimer.singleShot(0, w.try_lock_scope)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
