# -*- coding: utf-8 -*-
"""
Small PyQt5 UI for direct OceanFX/Ocean spectrometer capture.

Install the spectrometer library first:
    pip install seabreeze
    seabreeze_os_setup

Run:
    python oceanfx_capture_ui.py
"""

from __future__ import annotations

import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


DATA_START_MARKER = ">>>>>Begin Spectral Data<<<<<"


def load_seabreeze(backend: str | None = None):
    if backend:
        import seabreeze

        seabreeze.use(backend)

    import seabreeze.spectrometers as sb

    return sb


def default_output_dir() -> Path:
    return Path.cwd() / "oceanview_spectra" / "python_ui"


def save_spectrum(path: Path, wavelengths, intensities, metadata: dict[str, str], file_format: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    if file_format == "csv":
        import csv

        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            for key, value in metadata.items():
                writer.writerow([f"# {key}", value])
            writer.writerow(["wavelength_nm", "intensity_counts"])
            writer.writerows(zip(wavelengths, intensities))
        return

    with path.open("w", encoding="gb18030") as file:
        for key, value in metadata.items():
            file.write(f"# {key}: {value}\n")
        file.write(f"{DATA_START_MARKER}\n")
        for wavelength, intensity in zip(wavelengths, intensities):
            file.write(f"{float(wavelength):.6f}\t{float(intensity):.6f}\n")


def timestamp_name(prefix: str, index: int, total: int, suffix: str) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    if total == 1:
        return f"{prefix}_{stamp}.{suffix}"
    return f"{prefix}_{stamp}_{index:04d}.{suffix}"


def averaged_spectrum(spectrometer, scans_to_average: int):
    wavelengths, intensities = spectrometer.spectrum()
    if scans_to_average <= 1:
        return wavelengths, intensities

    summed = [float(value) for value in intensities]
    for _ in range(scans_to_average - 1):
        _, next_intensities = spectrometer.spectrum()
        for index, value in enumerate(next_intensities):
            summed[index] += float(value)
    return wavelengths, [value / scans_to_average for value in summed]


class CaptureWorker(QThread):
    message = pyqtSignal(str)
    finished_ok = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(
        self,
        backend: str | None,
        serial: str | None,
        integration_ms: int,
        average: int,
        output_dir: Path,
        prefix: str,
        file_format: str,
        count: int,
        interval_s: float,
        continuous: bool,
    ):
        super().__init__()
        self.backend = backend
        self.serial = serial
        self.integration_ms = integration_ms
        self.average = average
        self.output_dir = output_dir
        self.prefix = prefix
        self.file_format = file_format
        self.count = count
        self.interval_s = interval_s
        self.continuous = continuous
        self._stop_requested = False

    def stop(self) -> None:
        self._stop_requested = True

    def run(self) -> None:
        spectrometer = None
        try:
            sb = load_seabreeze(self.backend)
            if self.serial:
                spectrometer = sb.Spectrometer.from_serial_number(self.serial)
            else:
                spectrometer = sb.Spectrometer.from_first_available()

            model = str(getattr(spectrometer, "model", "unknown"))
            serial = str(getattr(spectrometer, "serial_number", "unknown"))
            integration_us = int(self.integration_ms * 1000)
            spectrometer.integration_time_micros(integration_us)
            self.message.emit(f"Connected: model={model}, serial={serial}")

            index = 1
            while not self._stop_requested:
                wavelengths, intensities = averaged_spectrum(spectrometer, self.average)
                total = 999999 if self.continuous else self.count
                file_path = self.output_dir / timestamp_name(self.prefix, index, total, self.file_format)
                metadata = {
                    "created_at": datetime.now().isoformat(timespec="milliseconds"),
                    "model": model,
                    "serial_number": serial,
                    "integration_time_us": str(integration_us),
                    "scans_to_average": str(self.average),
                }
                save_spectrum(file_path, wavelengths, intensities, metadata, self.file_format)
                self.message.emit(f"Saved: {file_path}")

                index += 1
                if not self.continuous and index > self.count:
                    break

                deadline = time.time() + max(self.interval_s, 0.0)
                while time.time() < deadline:
                    if self._stop_requested:
                        break
                    time.sleep(0.05)

            self.finished_ok.emit()
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if spectrometer is not None:
                try:
                    spectrometer.close()
                except Exception:
                    pass


class OceanFxCaptureUi(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OceanFX 光谱采集")
        self.resize(760, 520)

        self.worker: CaptureWorker | None = None
        self.devices: list[object] = []

        self.backend_combo = QComboBox()
        self.backend_combo.addItem("auto", None)
        self.backend_combo.addItem("cseabreeze", "cseabreeze")
        self.backend_combo.addItem("pyseabreeze", "pyseabreeze")

        self.device_combo = QComboBox()
        self.refresh_button = QPushButton("扫描设备")
        self.refresh_button.clicked.connect(self.refresh_devices)

        device_row = QHBoxLayout()
        device_row.addWidget(self.device_combo, 1)
        device_row.addWidget(self.refresh_button)

        self.integration_spin = QSpinBox()
        self.integration_spin.setRange(1, 600000)
        self.integration_spin.setValue(20)
        self.integration_spin.setSuffix(" ms")

        self.average_spin = QSpinBox()
        self.average_spin.setRange(1, 100)
        self.average_spin.setValue(1)

        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 100000)
        self.count_spin.setValue(1)

        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(0, 86400)
        self.interval_spin.setValue(1)
        self.interval_spin.setSuffix(" s")

        self.prefix_edit = QLineEdit("spectrum")

        self.format_combo = QComboBox()
        self.format_combo.addItem("txt - OceanView compatible", "txt")
        self.format_combo.addItem("csv", "csv")

        self.output_edit = QLineEdit(str(default_output_dir()))
        self.browse_button = QPushButton("选择保存路径")
        self.browse_button.clicked.connect(self.choose_output_dir)

        output_row = QHBoxLayout()
        output_row.addWidget(self.output_edit, 1)
        output_row.addWidget(self.browse_button)

        form = QFormLayout()
        form.addRow("Backend", self.backend_combo)
        form.addRow("设备", device_row)
        form.addRow("积分时间", self.integration_spin)
        form.addRow("平均次数", self.average_spin)
        form.addRow("保存数量", self.count_spin)
        form.addRow("保存间隔", self.interval_spin)
        form.addRow("文件前缀", self.prefix_edit)
        form.addRow("保存格式", self.format_combo)
        form.addRow("保存路径", output_row)

        self.single_button = QPushButton("保存一次")
        self.single_button.clicked.connect(self.save_once)
        self.batch_button = QPushButton("按数量保存")
        self.batch_button.clicked.connect(self.save_batch)
        self.continuous_button = QPushButton("连续保存")
        self.continuous_button.clicked.connect(self.save_continuous)
        self.stop_button = QPushButton("停止")
        self.stop_button.clicked.connect(self.stop_capture)
        self.stop_button.setEnabled(False)

        button_row = QHBoxLayout()
        button_row.addWidget(self.single_button)
        button_row.addWidget(self.batch_button)
        button_row.addWidget(self.continuous_button)
        button_row.addWidget(self.stop_button)

        self.status_label = QLabel("Ready")
        self.log = QTextEdit()
        self.log.setReadOnly(True)

        root = QVBoxLayout(self)
        root.addLayout(form)
        root.addLayout(button_row)
        root.addWidget(self.status_label)
        root.addWidget(self.log, 1)

        QTimer.singleShot(0, self.refresh_devices)

        self.append_log(f"Python: {sys.executable}")

    def append_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log.append(f"[{stamp}] {text}")
        self.status_label.setText(text)

    def current_backend(self) -> str | None:
        return self.backend_combo.currentData()

    def refresh_devices(self) -> None:
        self.device_combo.clear()
        self.devices = []
        try:
            sb = load_seabreeze(self.current_backend())
            self.devices = list(sb.list_devices())
        except ImportError as exc:
            self.append_log("未安装 seabreeze，无法扫描设备。")
            QMessageBox.warning(
                self,
                "缺少依赖",
                "需要先安装光谱仪 Python 库：\n\n"
                f'"{sys.executable}" -m pip install seabreeze\n'
                "seabreeze_os_setup\n\n"
                f"当前 UI 使用的 Python：\n{sys.executable}\n\n"
                f"错误：{exc}",
            )
            return
        except Exception as exc:
            self.append_log(f"扫描失败：{exc}")
            self.append_log(traceback.format_exc())
            return

        if not self.devices:
            self.device_combo.addItem("未发现设备", None)
            self.append_log("未发现光谱仪。")
            return

        for device in self.devices:
            self.device_combo.addItem(str(device), device)
        self.append_log(f"发现 {len(self.devices)} 个设备。")

    def choose_output_dir(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "选择光谱保存路径", self.output_edit.text())
        if folder:
            self.output_edit.setText(folder)

    def selected_serial(self) -> str | None:
        device = self.device_combo.currentData()
        if device is None:
            return None
        serial = getattr(device, "serial_number", None)
        return str(serial) if serial else None

    def start_capture(self, count: int, continuous: bool) -> None:
        if self.worker and self.worker.isRunning():
            QMessageBox.information(self, "正在采集", "当前已有采集任务在运行。")
            return

        output_dir = Path(self.output_edit.text()).expanduser().resolve()
        prefix = self.prefix_edit.text().strip() or "spectrum"

        self.worker = CaptureWorker(
            backend=self.current_backend(),
            serial=self.selected_serial(),
            integration_ms=self.integration_spin.value(),
            average=self.average_spin.value(),
            output_dir=output_dir,
            prefix=prefix,
            file_format=self.format_combo.currentData(),
            count=count,
            interval_s=float(self.interval_spin.value()),
            continuous=continuous,
        )
        self.worker.message.connect(self.append_log)
        self.worker.failed.connect(self.on_capture_failed)
        self.worker.finished_ok.connect(self.on_capture_finished)

        self.set_controls_enabled(False)
        self.append_log("开始采集。")
        self.worker.start()

    def set_controls_enabled(self, enabled: bool) -> None:
        for widget in (
            self.backend_combo,
            self.device_combo,
            self.refresh_button,
            self.integration_spin,
            self.average_spin,
            self.count_spin,
            self.interval_spin,
            self.prefix_edit,
            self.format_combo,
            self.output_edit,
            self.browse_button,
            self.single_button,
            self.batch_button,
            self.continuous_button,
        ):
            widget.setEnabled(enabled)
        self.stop_button.setEnabled(not enabled)

    def save_once(self) -> None:
        self.start_capture(count=1, continuous=False)

    def save_batch(self) -> None:
        self.start_capture(count=self.count_spin.value(), continuous=False)

    def save_continuous(self) -> None:
        self.start_capture(count=1, continuous=True)

    def stop_capture(self) -> None:
        if self.worker and self.worker.isRunning():
            self.append_log("正在停止采集...")
            self.worker.stop()

    def on_capture_failed(self, error: str) -> None:
        self.append_log(f"采集失败：{error}")
        self.set_controls_enabled(True)

    def on_capture_finished(self) -> None:
        self.append_log("采集结束。")
        self.set_controls_enabled(True)

    def closeEvent(self, event):
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(3000)
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = OceanFxCaptureUi()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
