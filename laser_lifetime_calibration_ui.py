import os
import sys
import time

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QGroupBox,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QTextEdit,
    QSpinBox,
    QDoubleSpinBox,
    QComboBox,
    QFrame,
    QToolButton,
    QMenu,
    QDialog,
    QDialogButtonBox,
    QMessageBox,
    QAbstractSpinBox,
    QSizePolicy,
    QTabWidget,
)
try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except Exception:
    FigureCanvas = None
    Figure = None


class PanelBox(QGroupBox):
    def __init__(self, title=""):
        super().__init__(title)
        self.setObjectName("PanelBox")


class LaserLifetimeCalibrationUI(QWidget):
    def __init__(self):
        super().__init__()
        self.scope_com_port = "COM1"
        self.hotplate_com_port = "COM2"
        self.hotplate_dialog = None
        self.hotplate_panel = None
        self.hotplate_plot_panel = None
        self._plot_t0 = time.time()
        self._plot_ts = []
        self._plot_tc = []
        self._plot_tf = []
        self._plot_pk = []
        self._plot_pf = []
        self._plot_canvas_temp = None
        self._plot_canvas_press = None
        self._plot_line_tc = None
        self._plot_line_tf = None
        self._plot_line_pk = None
        self._plot_line_pf = None
        self._plot_ax_temp = None
        self._plot_ax_press = None

        self.setWindowTitle("激光-寿命标定系统界面")
        self.resize(1180, 860)
        self.setup_ui()
        self.apply_styles()

    def setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 8, 10, 10)
        main_layout.setSpacing(8)

        settings_button = QToolButton()
        settings_button.setObjectName("SettingsMenuButton")
        settings_button.setText("menu-settings")
        settings_button.setPopupMode(QToolButton.InstantPopup)
        settings_menu = QMenu(settings_button)
        self.port_settings_action = settings_menu.addAction("COM端口设置...")
        self.port_settings_action.triggered.connect(self.open_port_settings_dialog)
        settings_button.setMenu(settings_menu)
        self.settings_button = settings_button
        self.update_settings_tooltip()

        top_tools = QHBoxLayout()
        top_tools.setContentsMargins(0, 0, 0, 0)
        top_tools.addStretch(1)
        top_tools.addWidget(settings_button, 0, Qt.AlignRight | Qt.AlignVCenter)
        main_layout.addLayout(top_tools)

        main_layout.addWidget(self.build_top_section(), 0)
        main_layout.addWidget(self.build_middle_section(), 0)
        main_layout.addWidget(self.build_bottom_section(), 1)

        self.hotplate_ctrl_btn.clicked.connect(self.open_hotplate_control_dialog)
        self.hide_spin_arrows()

    def hide_spin_arrows(self):
        for spin in self.findChildren(QAbstractSpinBox):
            spin.setButtonSymbols(QAbstractSpinBox.NoButtons)

    def open_hotplate_control_dialog(self):
        if self.hotplate_dialog is None:
            self.hotplate_dialog = QDialog(self)
            self.hotplate_dialog.setWindowTitle("标定台手动控制")
            self.hotplate_dialog.setMinimumSize(980, 420)
            layout = QHBoxLayout(self.hotplate_dialog)
            layout.setContentsMargins(10, 10, 10, 10)
            layout.setSpacing(10)
            if self.hotplate_plot_panel is not None:
                layout.addWidget(self.hotplate_plot_panel, 3)
            layout.addWidget(self.hotplate_panel, 2)
            self.hotplate_dialog.setLayout(layout)
        # Ensure spin boxes in this popup do not show up/down arrows.
        for spin in self.hotplate_dialog.findChildren(QAbstractSpinBox):
            spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.hotplate_dialog.show()
        self.hotplate_dialog.raise_()
        self.hotplate_dialog.activateWindow()

    def _build_hotplate_plot_panel(self):
        box = PanelBox("实时曲线")
        v = QVBoxLayout()
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)
        if Figure is None or FigureCanvas is None:
            v.addWidget(QLabel("未检测到 matplotlib，无法显示曲线图。"))
            box.setLayout(v)
            return box

        tabs = QTabWidget()

        temp_page = QFrame()
        temp_layout = QVBoxLayout(temp_page)
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_fig = Figure(figsize=(5.8, 3.0), dpi=100)
        temp_fig.patch.set_facecolor("white")
        ax_temp = temp_fig.add_subplot(111)
        temp_fig.subplots_adjust(left=0.12, right=0.98, bottom=0.18, top=0.95)
        ax_temp.set_facecolor("white")
        ax_temp.grid(color="#d9e2ec", linestyle="-", linewidth=0.8)
        ax_temp.set_xlabel("Time (s)")
        ax_temp.set_ylabel("Temperature (°C)")
        line_tc, = ax_temp.plot([], [], color="#d64545", linewidth=1.8, marker="o", markersize=2.5, label="TC")
        line_tf, = ax_temp.plot([], [], color="#ff9f43", linewidth=1.8, marker="o", markersize=2.5, label="TF")
        ax_temp.legend(loc="upper left")
        ax_temp.set_xlim(0, 1)
        t0 = float(self.hotplate_tf_spin.value()) if hasattr(self, "hotplate_tf_spin") else 25.0
        t_lo, t_hi = self._compute_y_limits([t0, t0])
        ax_temp.set_ylim(t_lo, t_hi)
        canvas_temp = FigureCanvas(temp_fig)
        temp_layout.addWidget(canvas_temp)

        press_page = QFrame()
        press_layout = QVBoxLayout(press_page)
        press_layout.setContentsMargins(0, 0, 0, 0)
        press_fig = Figure(figsize=(5.8, 3.0), dpi=100)
        press_fig.patch.set_facecolor("white")
        ax_press = press_fig.add_subplot(111)
        press_fig.subplots_adjust(left=0.12, right=0.98, bottom=0.18, top=0.95)
        ax_press.set_facecolor("white")
        ax_press.grid(color="#d9e2ec", linestyle="-", linewidth=0.8)
        ax_press.set_xlabel("Time (s)")
        ax_press.set_ylabel("Pressure (kPa)")
        line_pk, = ax_press.plot([], [], color="#2d79c7", linewidth=1.8, marker="o", markersize=2.5, label="PK")
        line_pf, = ax_press.plot([], [], color="#6c5ce7", linewidth=1.8, marker="o", markersize=2.5, label="PF")
        ax_press.legend(loc="upper left")
        ax_press.set_xlim(0, 1)
        p0 = float(self.hotplate_pf_spin.value()) if hasattr(self, "hotplate_pf_spin") else 101.3
        p_lo, p_hi = self._compute_y_limits([p0, p0])
        ax_press.set_ylim(p_lo, p_hi)
        canvas_press = FigureCanvas(press_fig)
        press_layout.addWidget(canvas_press)

        tabs.addTab(temp_page, "温度曲线图")
        tabs.addTab(press_page, "压力曲线图")
        v.addWidget(tabs)
        box.setLayout(v)

        self._plot_canvas_temp = canvas_temp
        self._plot_canvas_press = canvas_press
        self._plot_line_tc = line_tc
        self._plot_line_tf = line_tf
        self._plot_line_pk = line_pk
        self._plot_line_pf = line_pf
        self._plot_ax_temp = ax_temp
        self._plot_ax_press = ax_press
        return box

    @staticmethod
    def _compute_y_limits(values):
        if not values:
            return 0.0, 1.0
        raw_min = min(values)
        raw_max = max(values)
        span = max(raw_max - raw_min, 1.0)
        top_pad = max(span * 0.22, abs(raw_max) * 0.02, 1.5)
        bottom_pad = max(span * 0.12, abs(raw_min) * 0.01, 1.0)
        lo = raw_min - bottom_pad
        hi = raw_max + top_pad
        if hi <= lo:
            hi = lo + 1.0
        return lo, hi

    def push_hotplate_plot(self, tc, pk, tf=None, pf=None):
        if self._plot_canvas_temp is None or self._plot_canvas_press is None or tc is None or pk is None:
            return
        if tf is None and hasattr(self, "hotplate_tf_spin"):
            tf = float(self.hotplate_tf_spin.value())
        if pf is None and hasattr(self, "hotplate_pf_spin"):
            pf = float(self.hotplate_pf_spin.value())
        if tf is None:
            tf = tc
        if pf is None:
            pf = pk
        t = time.time() - self._plot_t0
        self._plot_ts.append(t)
        self._plot_tc.append(float(tc))
        self._plot_tf.append(float(tf))
        self._plot_pk.append(float(pk))
        self._plot_pf.append(float(pf))
        xs = list(self._plot_ts)
        ys_tc = list(self._plot_tc)
        ys_tf = list(self._plot_tf)
        ys_pk = list(self._plot_pk)
        ys_pf = list(self._plot_pf)

        if len(xs) == 1:
            xs = [xs[0] - 0.1, xs[0]]
            ys_tc = [ys_tc[0], ys_tc[0]]
            ys_tf = [ys_tf[0], ys_tf[0]]
            ys_pk = [ys_pk[0], ys_pk[0]]
            ys_pf = [ys_pf[0], ys_pf[0]]

        self._plot_line_tc.set_data(xs, ys_tc)
        self._plot_line_tf.set_data(xs, ys_tf)
        self._plot_line_pk.set_data(xs, ys_pk)
        self._plot_line_pf.set_data(xs, ys_pf)

        self._plot_ax_temp.set_xlim(0, xs[-1] + 1)
        self._plot_ax_press.set_xlim(0, xs[-1] + 1)

        t_lo, t_hi = self._compute_y_limits(ys_tc + ys_tf)
        p_lo, p_hi = self._compute_y_limits(ys_pk + ys_pf)
        self._plot_ax_temp.set_ylim(t_lo, t_hi)
        self._plot_ax_press.set_ylim(p_lo, p_hi)
        self._plot_canvas_temp.draw_idle()
        self._plot_canvas_press.draw_idle()

    def _sync_cal_toggle_text(self, btn, checked):
        btn.setText("开启" if checked else "关闭")

    def _sync_single_mode_text(self, checked):
        self.single_mode_btn.setText("Single" if checked else "连续")

    def _sync_avg_mode_text(self, checked):
        self.avg_checkbox.setText("平均" if checked else "采样")

    def _sync_start_btn_text(self, checked):
        self.start_btn.setText("RUNNING" if checked else "RUN")

    def _sync_fit_mode_text(self, checked):
        self.fit_enable_btn.setText("拟合寿命" if checked else "不拟合")

    def update_settings_tooltip(self):
        self.settings_button.setToolTip(f"示波器: {self.scope_com_port} | 标定台: {self.hotplate_com_port}")

    def get_available_com_ports(self):
        if list_ports is None:
            return []
        ports = []
        for port_info in list_ports.comports():
            port_name = port_info.device
            description = (port_info.description or "").strip()
            if description and description != port_name:
                label = f"{port_name} ({description})"
            else:
                label = port_name
            ports.append((label, port_name))
        ports.sort(key=lambda item: item[1])
        return ports

    def open_port_settings_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("COM端口设置")
        dialog.setMinimumWidth(380)

        layout = QVBoxLayout(dialog)
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(10)

        form.addWidget(QLabel("示波器 COM口:"), 0, 0)
        scope_combo = QComboBox()
        form.addWidget(scope_combo, 0, 1)

        form.addWidget(QLabel("标定台 COM口:"), 1, 0)
        hotplate_combo = QComboBox()
        form.addWidget(hotplate_combo, 1, 1)

        port_items = self.get_available_com_ports()
        if not port_items:
            for fallback in [self.scope_com_port, self.hotplate_com_port]:
                if fallback and fallback not in [item[1] for item in port_items]:
                    port_items.append((f"{fallback} (当前配置)", fallback))

        for label, value in port_items:
            scope_combo.addItem(label, value)
            hotplate_combo.addItem(label, value)

        scope_idx = scope_combo.findData(self.scope_com_port)
        if scope_idx >= 0:
            scope_combo.setCurrentIndex(scope_idx)

        hotplate_idx = hotplate_combo.findData(self.hotplate_com_port)
        if hotplate_idx >= 0:
            hotplate_combo.setCurrentIndex(hotplate_idx)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if list_ports is None:
            QMessageBox.information(self, "提示", "未安装 pyserial，无法读取本机真实串口。\n请先安装: pip install pyserial")

        if dialog.exec_() == QDialog.Accepted:
            self.scope_com_port = scope_combo.currentData() or self.scope_com_port
            self.hotplate_com_port = hotplate_combo.currentData() or self.hotplate_com_port
            self.update_settings_tooltip()

    def build_top_section(self):
        box = PanelBox("基础参数区")
        layout = QHBoxLayout()
        layout.setSpacing(10)

        temp_box = PanelBox("温度标定")
        temp_grid = QGridLayout()
        temp_grid.setHorizontalSpacing(14)
        temp_grid.setVerticalSpacing(10)

        temp_grid.addWidget(QLabel("温度标定:"), 0, 0)
        self.temp_cal_btn = QPushButton()
        self.temp_cal_btn.setObjectName("CalToggleButton")
        self.temp_cal_btn.setCheckable(True)
        self.temp_cal_btn.setChecked(True)
        self.temp_cal_btn.toggled.connect(lambda checked: self._sync_cal_toggle_text(self.temp_cal_btn, checked))
        self._sync_cal_toggle_text(self.temp_cal_btn, True)
        temp_grid.addWidget(self.temp_cal_btn, 0, 1)

        temp_grid.addWidget(QLabel("参考温度 (°C):"), 0, 2)
        self.ref_temp_spin = QDoubleSpinBox()
        self.ref_temp_spin.setRange(-50, 500)
        self.ref_temp_spin.setValue(25.0)
        self.ref_temp_spin.setDecimals(1)
        temp_grid.addWidget(self.ref_temp_spin, 0, 3)

        temp_grid.addWidget(QLabel("温度范围:"), 1, 0)
        temp_range_row = QFrame()
        temp_range_layout = QHBoxLayout(temp_range_row)
        temp_range_layout.setContentsMargins(0, 0, 0, 0)
        temp_range_layout.setSpacing(8)
        self.temp_range_start_spin = QDoubleSpinBox(); self.temp_range_start_spin.setRange(-50, 500); self.temp_range_start_spin.setDecimals(1); self.temp_range_start_spin.setValue(50.0)
        self.temp_range_step_spin = QDoubleSpinBox(); self.temp_range_step_spin.setRange(0.1, 200); self.temp_range_step_spin.setDecimals(1); self.temp_range_step_spin.setValue(50.0)
        self.temp_range_end_spin = QDoubleSpinBox(); self.temp_range_end_spin.setRange(-50, 800); self.temp_range_end_spin.setDecimals(1); self.temp_range_end_spin.setValue(600.0)
        temp_range_layout.addWidget(self.temp_range_start_spin, 1)
        temp_range_layout.addWidget(QLabel(":"))
        temp_range_layout.addWidget(self.temp_range_step_spin, 1)
        temp_range_layout.addWidget(QLabel(":"))
        temp_range_layout.addWidget(self.temp_range_end_spin, 1)
        temp_range_layout.addWidget(QLabel("(℃)"))
        temp_grid.addWidget(temp_range_row, 1, 1, 1, 3)

        temp_grid.addWidget(QLabel("温度波动阈值 (°C):"), 2, 0)
        self.temp_fluc_spin = QDoubleSpinBox(); self.temp_fluc_spin.setRange(0, 50); self.temp_fluc_spin.setDecimals(2); self.temp_fluc_spin.setValue(0.20)
        temp_grid.addWidget(self.temp_fluc_spin, 2, 1)
        temp_grid.addWidget(QLabel("温稳时间 (s):"), 2, 2)
        self.temp_stable_time_spin = QSpinBox(); self.temp_stable_time_spin.setRange(0, 3600); self.temp_stable_time_spin.setValue(300)
        temp_grid.addWidget(self.temp_stable_time_spin, 2, 3)

        temp_grid.addWidget(QLabel("控制模式:"), 3, 0)
        self.temp_mode_combo = QComboBox(); self.temp_mode_combo.addItem("Ramp", "RAMP"); self.temp_mode_combo.addItem("Hold", "HOLD")
        temp_grid.addWidget(self.temp_mode_combo, 3, 1)
        temp_grid.addWidget(QLabel("升温速率 (℃/min):"), 3, 2)
        self.temp_auto_rate_spin = QDoubleSpinBox(); self.temp_auto_rate_spin.setRange(0.1, 100); self.temp_auto_rate_spin.setDecimals(2); self.temp_auto_rate_spin.setValue(5.0)
        temp_grid.addWidget(self.temp_auto_rate_spin, 3, 3)

        temp_box.setLayout(temp_grid)

        press_box = PanelBox("压力标定")
        press_grid = QGridLayout()
        press_grid.setHorizontalSpacing(14)
        press_grid.setVerticalSpacing(10)

        press_grid.addWidget(QLabel("压力标定:"), 0, 0)
        self.press_cal_btn = QPushButton()
        self.press_cal_btn.setObjectName("CalToggleButton")
        self.press_cal_btn.setCheckable(True)
        self.press_cal_btn.setChecked(False)
        self.press_cal_btn.toggled.connect(lambda checked: self._sync_cal_toggle_text(self.press_cal_btn, checked))
        self._sync_cal_toggle_text(self.press_cal_btn, False)
        press_grid.addWidget(self.press_cal_btn, 0, 1)

        press_grid.addWidget(QLabel("参考压力 (kPa):"), 0, 2)
        self.ref_press_spin = QDoubleSpinBox(); self.ref_press_spin.setRange(0, 5000); self.ref_press_spin.setValue(101.3); self.ref_press_spin.setDecimals(1)
        press_grid.addWidget(self.ref_press_spin, 0, 3)

        press_grid.addWidget(QLabel("压力范围:"), 1, 0)
        press_range_row = QFrame()
        press_range_layout = QHBoxLayout(press_range_row)
        press_range_layout.setContentsMargins(0, 0, 0, 0)
        press_range_layout.setSpacing(8)
        self.press_range_start_spin = QDoubleSpinBox(); self.press_range_start_spin.setRange(0, 5000); self.press_range_start_spin.setDecimals(1); self.press_range_start_spin.setValue(101.3)
        self.press_range_step_spin = QDoubleSpinBox(); self.press_range_step_spin.setRange(0.1, 500); self.press_range_step_spin.setDecimals(1); self.press_range_step_spin.setValue(5.0)
        self.press_range_end_spin = QDoubleSpinBox(); self.press_range_end_spin.setRange(0, 5000); self.press_range_end_spin.setDecimals(1); self.press_range_end_spin.setValue(121.3)
        press_range_layout.addWidget(self.press_range_start_spin, 1)
        press_range_layout.addWidget(QLabel(":"))
        press_range_layout.addWidget(self.press_range_step_spin, 1)
        press_range_layout.addWidget(QLabel(":"))
        press_range_layout.addWidget(self.press_range_end_spin, 1)
        press_range_layout.addWidget(QLabel("(kPa)"))
        press_grid.addWidget(press_range_row, 1, 1, 1, 3)

        press_grid.addWidget(QLabel("压力波动阈值 (kPa):"), 2, 0)
        self.press_fluc_spin = QDoubleSpinBox(); self.press_fluc_spin.setRange(0, 100); self.press_fluc_spin.setDecimals(2); self.press_fluc_spin.setValue(0.50)
        press_grid.addWidget(self.press_fluc_spin, 2, 1)
        press_grid.addWidget(QLabel("压稳时间 (s):"), 2, 2)
        self.press_stable_time_spin = QSpinBox(); self.press_stable_time_spin.setRange(0, 3600); self.press_stable_time_spin.setValue(300)
        press_grid.addWidget(self.press_stable_time_spin, 2, 3)

        press_grid.addWidget(QLabel("控制模式:"), 3, 0)
        self.press_mode_combo = QComboBox(); self.press_mode_combo.addItem("Ramp", "RAMP"); self.press_mode_combo.addItem("Hold", "HOLD")
        press_grid.addWidget(self.press_mode_combo, 3, 1)
        press_grid.addWidget(QLabel("升压速率 (kPa/min):"), 3, 2)
        self.press_auto_rate_spin = QDoubleSpinBox(); self.press_auto_rate_spin.setRange(0.1, 1000); self.press_auto_rate_spin.setDecimals(2); self.press_auto_rate_spin.setValue(5.0)
        press_grid.addWidget(self.press_auto_rate_spin, 3, 3)

        press_box.setLayout(press_grid)

        layout.addWidget(temp_box, 1)
        layout.addWidget(press_box, 1)
        box.setLayout(layout)
        return box

    def _build_hotplate_panel(self):
        hotplate = PanelBox("标定台手动控制")
        hotplate_grid = QGridLayout()
        hotplate_grid.setHorizontalSpacing(12)
        hotplate_grid.setVerticalSpacing(10)
        hotplate_grid.setColumnStretch(0, 1)
        hotplate_grid.setColumnStretch(1, 1)
        hotplate_grid.setColumnMinimumWidth(1, 140)

        hotplate_grid.addWidget(QLabel("TF (℃):"), 0, 0)
        self.hotplate_tf_spin = QDoubleSpinBox(); self.hotplate_tf_spin.setRange(-100, 600); self.hotplate_tf_spin.setDecimals(1); self.hotplate_tf_spin.setValue(25.0)
        hotplate_grid.addWidget(self.hotplate_tf_spin, 0, 1)

        hotplate_grid.addWidget(QLabel("Rate (℃/min):"), 1, 0)
        self.hotplate_rate_spin = QDoubleSpinBox(); self.hotplate_rate_spin.setRange(0.1, 100); self.hotplate_rate_spin.setDecimals(2); self.hotplate_rate_spin.setValue(5.0)
        hotplate_grid.addWidget(self.hotplate_rate_spin, 1, 1)

        self.hotplate_hold_btn = QPushButton("T-Hold")
        self.hotplate_ramp_btn = QPushButton("T-Ramp")
        self.hotplate_stop_btn = QPushButton("T-Stop")
        self.hotplate_phold_btn = QPushButton("P-Hold")
        self.hotplate_pramp_btn = QPushButton("P-Ramp")
        self.hotplate_pstop_btn = QPushButton("P-Stop")
        self.hotplate_stop_btn.setObjectName("StopButton")
        self.hotplate_pstop_btn.setObjectName("StopButton")

        for btn in (
            self.hotplate_hold_btn, self.hotplate_ramp_btn, self.hotplate_stop_btn,
            self.hotplate_phold_btn, self.hotplate_pramp_btn, self.hotplate_pstop_btn,
        ):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        hotplate_grid.addWidget(self.hotplate_hold_btn, 2, 0)
        hotplate_grid.addWidget(self.hotplate_ramp_btn, 2, 1)
        hotplate_grid.addWidget(self.hotplate_stop_btn, 3, 0, 1, 2)

        hotplate_grid.addWidget(QLabel("PF (kPa):"), 4, 0)
        self.hotplate_pf_spin = QDoubleSpinBox(); self.hotplate_pf_spin.setRange(0, 3000); self.hotplate_pf_spin.setDecimals(1); self.hotplate_pf_spin.setValue(101.3)
        hotplate_grid.addWidget(self.hotplate_pf_spin, 4, 1)

        hotplate_grid.addWidget(QLabel("P-Rate (kPa/min):"), 5, 0)
        self.hotplate_prate_spin = QDoubleSpinBox(); self.hotplate_prate_spin.setRange(0.1, 1000); self.hotplate_prate_spin.setDecimals(2); self.hotplate_prate_spin.setValue(5.0)
        hotplate_grid.addWidget(self.hotplate_prate_spin, 5, 1)

        hotplate_grid.addWidget(self.hotplate_phold_btn, 6, 0)
        hotplate_grid.addWidget(self.hotplate_pramp_btn, 6, 1)
        hotplate_grid.addWidget(self.hotplate_pstop_btn, 7, 0, 1, 2)

        hotplate.setLayout(hotplate_grid)
        hotplate.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        return hotplate

    def build_middle_section(self):
        container = QFrame()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        left = PanelBox("标定流程设置")
        left_grid = QGridLayout()
        left_grid.setHorizontalSpacing(12)
        left_grid.setVerticalSpacing(10)
        left_grid.setColumnStretch(1, 2)
        left_grid.setColumnStretch(3, 1)
        left_grid.setColumnStretch(5, 1)

        row = 0
        left_grid.addWidget(QLabel("标定模式:"), row, 0)
        mode_select = QComboBox(); mode_select.addItems(["寿命标定", "待开发"])
        left_grid.addWidget(mode_select, row, 1)

        left_grid.addWidget(QLabel("光源稳定时间 (s):"), row, 2)
        self.stable_time_spin = QSpinBox(); self.stable_time_spin.setRange(0, 3600); self.stable_time_spin.setValue(10)
        left_grid.addWidget(self.stable_time_spin, row, 3)

        left_grid.addWidget(QLabel("循环次数:"), row, 4)
        self.cycles_spin = QSpinBox(); self.cycles_spin.setRange(1, 9999); self.cycles_spin.setValue(10)
        left_grid.addWidget(self.cycles_spin, row, 5)
        row += 1

        left_grid.addWidget(QLabel("选择保存路径:"), row, 0)
        self.save_path_edit = QLineEdit("D:/CalibrationData/LaserLifetime")
        left_grid.addWidget(self.save_path_edit, row, 1, 1, 4)
        self.browse_btn = QPushButton("浏览...")
        left_grid.addWidget(self.browse_btn, row, 5)
        row += 1

        action_row = QFrame()
        action_layout = QHBoxLayout(action_row)
        action_layout.setContentsMargins(0, 0, 0, 0)
        action_layout.setSpacing(8)
        self.init_btn = QPushButton("初始化")
        self.start_btn = QPushButton()
        self.start_btn.setObjectName("StartRunButton")
        self.start_btn.setCheckable(True)
        self.start_btn.setChecked(False)
        self.start_btn.toggled.connect(self._sync_start_btn_text)
        self._sync_start_btn_text(False)
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setObjectName("StopButton")
        action_layout.addWidget(self.init_btn, 1)
        action_layout.addWidget(self.start_btn, 2)
        action_layout.addWidget(self.stop_btn, 1)
        left_grid.addWidget(action_row, row, 0, 1, 6)

        left.setLayout(left_grid)
        left.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

        scope_box = PanelBox("示波器设置")
        right_grid = QGridLayout()
        right_grid.setHorizontalSpacing(12)
        right_grid.setVerticalSpacing(10)

        right_grid.addWidget(QLabel("采样模式:"), 0, 0)
        self.single_mode_btn = QPushButton()
        self.single_mode_btn.setObjectName("SingleModeButton")
        self.single_mode_btn.setCheckable(True)
        self.single_mode_btn.setChecked(True)
        self.single_mode_btn.toggled.connect(self._sync_single_mode_text)
        self._sync_single_mode_text(True)
        self.single_mode_btn.setToolTip("按下=单次采样，未按下=连续采样")
        right_grid.addWidget(self.single_mode_btn, 0, 1)

        right_grid.addWidget(QLabel("保存通道:"), 1, 0)
        self.channel_combo = QComboBox(); self.channel_combo.addItems(["CH1", "CH2", "CH3", "CH4", "CH1 + CH2"])
        right_grid.addWidget(self.channel_combo, 1, 1)

        right_grid.addWidget(QLabel("是否平均:"), 0, 2)
        self.avg_checkbox = QPushButton()
        self.avg_checkbox.setObjectName("AvgModeButton")
        self.avg_checkbox.setCheckable(True)
        self.avg_checkbox.setChecked(True)
        self.avg_checkbox.toggled.connect(self._sync_avg_mode_text)
        self._sync_avg_mode_text(True)
        right_grid.addWidget(self.avg_checkbox, 0, 3)

        self.avg_count_label = QLabel("平均次数:")
        right_grid.addWidget(self.avg_count_label, 1, 2)
        self.avg_times_spin = QSpinBox(); self.avg_times_spin.setRange(1, 4096); self.avg_times_spin.setValue(64)
        right_grid.addWidget(self.avg_times_spin, 1, 3)

        scope_box.setLayout(right_grid)
        scope_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        self.hotplate_ctrl_btn = QPushButton("标定台手动控制")
        self.hotplate_ctrl_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.fit_enable_btn = QPushButton()
        self.fit_enable_btn.setObjectName("FitModeButton")
        self.fit_enable_btn.setCheckable(True)
        self.fit_enable_btn.setChecked(True)
        self.fit_enable_btn.toggled.connect(self._sync_fit_mode_text)
        self._sync_fit_mode_text(True)
        self.fit_enable_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.fit_settings_btn = QPushButton("拟合设置")
        self.fit_settings_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.hotplate_panel = self._build_hotplate_panel()
        self.hotplate_plot_panel = self._build_hotplate_plot_panel()

        right_column = QFrame()
        right_column_layout = QVBoxLayout(right_column)
        right_column_layout.setContentsMargins(0, 0, 0, 0)
        right_column_layout.setSpacing(8)
        right_column_layout.addWidget(scope_box)
        hotplate_row = QFrame()
        hotplate_row_layout = QHBoxLayout(hotplate_row)
        hotplate_row_layout.setContentsMargins(0, 0, 0, 0)
        hotplate_row_layout.setSpacing(8)
        hotplate_row_layout.addWidget(self.hotplate_ctrl_btn, 1)
        hotplate_row_layout.addWidget(self.fit_enable_btn, 1)
        hotplate_row_layout.addWidget(self.fit_settings_btn, 1)
        right_column_layout.addWidget(hotplate_row)
        right_column_layout.addStretch(1)

        layout.addWidget(right_column, 4, Qt.AlignTop)
        layout.addWidget(left, 5, Qt.AlignTop)
        return container

    def build_bottom_section(self):
        container = QFrame()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        log1 = PanelBox("温压过程日志")
        v1 = QVBoxLayout()
        self.sensor_log_text = QTextEdit()
        self.sensor_log_text.setReadOnly(True)
        self.sensor_log_text.setPlainText("[说明] 记录实时温度、压力、稳定判定进度")
        v1.addWidget(self.sensor_log_text)
        log1.setLayout(v1)

        log2 = PanelBox("运行事件日志")
        v2 = QVBoxLayout()
        self.status_log_text = QTextEdit()
        self.status_log_text.setReadOnly(True)
        self.status_log_text.setPlainText("[说明] 记录系统事件、设备状态、任务流程")
        v2.addWidget(self.status_log_text)
        log2.setLayout(v2)

        layout.addWidget(log1, 1)
        layout.addWidget(log2, 1)
        container.setMinimumHeight(320)
        return container

    def apply_styles(self):
        font = QFont("Microsoft YaHei UI", 10)
        QApplication.instance().setFont(font)
        self.setStyleSheet(
            """
            QWidget {
                background: #f3f5f7;
                color: #222;
            }
            QGroupBox#PanelBox {
                border: 1px solid #b8c4d0;
                border-radius: 6px;
                margin-top: 12px;
                background: #ffffff;
                font-weight: 700;
                padding-top: 10px;
            }
            QGroupBox#PanelBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #244a73;
            }
            QLabel {
                min-height: 28px;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit {
                background: #fcfcfd;
                border: 1px solid #b9c2cc;
                border-radius: 4px;
                padding: 4px 6px;
                min-height: 28px;
            }
            QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {
                background: #eef1f4;
                color: #8e98a3;
                border: 1px dashed #c3ccd6;
            }
            QTextEdit {
                min-height: 300px;
            }
            QPushButton {
                min-height: 32px;
                padding: 4px 14px;
                border-radius: 4px;
                border: 1px solid #8ea5bb;
                background: #e9eef3;
            }
            QPushButton:hover {
                background: #dfe8f1;
            }
            QPushButton#AccentButton {
                background: #2d79c7;
                color: white;
                border: 1px solid #2569ad;
                font-weight: 700;
            }
            QPushButton#AccentButton:hover {
                background: #256fb9;
            }
            QPushButton#CalToggleButton {
                background: #eceff3;
                border: 1px solid #9bb0c5;
                font-weight: 600;
                min-width: 70px;
            }
            QPushButton#CalToggleButton:checked {
                background: #2d79c7;
                color: #ffffff;
                border: 1px solid #2569ad;
            }
            QPushButton#SingleModeButton,
            QPushButton#AvgModeButton,
            QPushButton#FitModeButton,
            QPushButton#StartRunButton {
                background: #edf2f7;
                border: 1px solid #91a5ba;
                font-weight: 600;
            }
            QPushButton#SingleModeButton:checked,
            QPushButton#AvgModeButton:checked,
            QPushButton#FitModeButton:checked,
            QPushButton#StartRunButton:checked {
                background: #2d79c7;
                color: #ffffff;
                border: 1px solid #2569ad;
            }
            QPushButton#StopButton {
                background: #fbe9ea;
                color: #9f2a33;
                border: 1px solid #d79ca1;
                font-weight: 700;
            }
            QPushButton#StopButton:hover {
                background: #f8d9dc;
            }
            QToolButton#SettingsMenuButton {
                min-height: 30px;
                padding: 4px 12px;
                border-radius: 4px;
                border: 1px solid #8ea5bb;
                background: #e9eef3;
            }
            QToolButton#SettingsMenuButton:hover {
                background: #dfe8f1;
            }
            """
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LaserLifetimeCalibrationUI()
    window.show()
    sys.exit(app.exec_())
