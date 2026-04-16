import os
import sys
try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QGroupBox,
    QGridLayout, QVBoxLayout, QHBoxLayout, QTextEdit, QSpinBox,
    QDoubleSpinBox, QCheckBox, QComboBox, QFrame, QToolButton,
    QMenu, QDialog, QDialogButtonBox, QMessageBox, QAbstractSpinBox
)


class PanelBox(QGroupBox):
    def __init__(self, title=''):
        super().__init__(title)
        self.setObjectName('PanelBox')


class LaserLifetimeCalibrationUI(QWidget):
    def __init__(self):
        super().__init__()
        self.scope_com_port = 'COM1'
        self.hotplate_com_port = 'COM2'
        self.setWindowTitle('激光-寿命标定系统界面')
        self.resize(1180, 860)
        self.setup_ui()
        self.apply_styles()

    def setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        header_layout = QGridLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setHorizontalSpacing(8)
        header_layout.setColumnStretch(1, 1)

        settings_button = QToolButton()
        settings_button.setObjectName('SettingsMenuButton')
        settings_button.setText('menu-settings')
        settings_button.setPopupMode(QToolButton.InstantPopup)
        settings_menu = QMenu(settings_button)
        self.port_settings_action = settings_menu.addAction('COM端口设置...')
        self.port_settings_action.triggered.connect(self.open_port_settings_dialog)
        settings_button.setMenu(settings_menu)
        self.settings_button = settings_button
        self.update_settings_tooltip()

        title = QLabel('激光-寿命标定')
        title.setAlignment(Qt.AlignCenter)
        title.setObjectName('MainTitle')
        dummy = QLabel('')
        dummy.setFixedWidth(settings_button.sizeHint().width())
        header_layout.addWidget(settings_button, 0, 0, Qt.AlignLeft | Qt.AlignVCenter)
        header_layout.addWidget(title, 0, 1)
        header_layout.addWidget(dummy, 0, 2)
        main_layout.addLayout(header_layout)

        main_layout.addWidget(self.build_top_section())
        main_layout.addWidget(self.build_middle_section())
        main_layout.addWidget(self.build_bottom_section(), 1)
        self.hide_spin_arrows()

    def hide_spin_arrows(self):
        for spin in self.findChildren(QAbstractSpinBox):
            spin.setButtonSymbols(QAbstractSpinBox.NoButtons)

    def _sync_cal_toggle_text(self, btn, checked):
        btn.setText('开启' if checked else '关闭')

    def _sync_single_mode_text(self, checked):
        self.single_mode_btn.setText('Single' if checked else '连续')


    def update_settings_tooltip(self):
        self.settings_button.setToolTip(
            f'示波器: {self.scope_com_port} | 加热台: {self.hotplate_com_port}'
        )

    def get_available_com_ports(self):
        if list_ports is None:
            return []
        ports = []
        for port_info in list_ports.comports():
            port_name = port_info.device
            description = (port_info.description or '').strip()
            if description and description != port_name:
                label = f'{port_name} ({description})'
            else:
                label = port_name
            ports.append((label, port_name))
        ports.sort(key=lambda item: item[1])
        return ports

    def open_port_settings_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle('COM端口设置')
        dialog.setMinimumWidth(380)

        layout = QVBoxLayout(dialog)
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(10)

        form.addWidget(QLabel('示波器 COM口:'), 0, 0)
        scope_combo = QComboBox()
        form.addWidget(scope_combo, 0, 1)

        form.addWidget(QLabel('加热台 COM口:'), 1, 0)
        hotplate_combo = QComboBox()
        form.addWidget(hotplate_combo, 1, 1)

        port_items = self.get_available_com_ports()
        if not port_items:
            for fallback in [self.scope_com_port, self.hotplate_com_port]:
                if fallback and fallback not in [item[1] for item in port_items]:
                    port_items.append((f'{fallback} (当前配置)', fallback))

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
            QMessageBox.information(
                self,
                '提示',
                '未安装 pyserial，无法读取本机真实串口。\\n请先安装: pip install pyserial'
            )

        if dialog.exec_() == QDialog.Accepted:
            self.scope_com_port = scope_combo.currentData() or self.scope_com_port
            self.hotplate_com_port = hotplate_combo.currentData() or self.hotplate_com_port
            self.update_settings_tooltip()

    def build_top_section(self):
        box = PanelBox('基础参数区')
        layout = QHBoxLayout()
        layout.setSpacing(10)

        temp_box = PanelBox('温度标定')
        temp_grid = QGridLayout()
        temp_grid.setHorizontalSpacing(14)
        temp_grid.setVerticalSpacing(10)

        temp_grid.addWidget(QLabel('温度标定:'), 0, 0)
        self.temp_cal_btn = QPushButton()
        self.temp_cal_btn.setObjectName('CalToggleButton')
        self.temp_cal_btn.setCheckable(True)
        self.temp_cal_btn.setChecked(True)
        self.temp_cal_btn.toggled.connect(
            lambda checked: self._sync_cal_toggle_text(self.temp_cal_btn, checked)
        )
        self._sync_cal_toggle_text(self.temp_cal_btn, True)
        temp_grid.addWidget(self.temp_cal_btn, 0, 1)

        temp_grid.addWidget(QLabel('参考温度 (°C):'), 0, 2)
        self.ref_temp_spin = QDoubleSpinBox()
        self.ref_temp_spin.setRange(-50, 500)
        self.ref_temp_spin.setValue(25.0)
        self.ref_temp_spin.setDecimals(1)
        temp_grid.addWidget(self.ref_temp_spin, 0, 3)

        temp_grid.addWidget(QLabel('温度范围:'), 1, 0)
        temp_range_row = QFrame()
        temp_range_layout = QHBoxLayout(temp_range_row)
        temp_range_layout.setContentsMargins(0, 0, 0, 0)
        temp_range_layout.setSpacing(8)

        temp_start_label = QLabel('起始温度(℃):')
        temp_start = QDoubleSpinBox()
        temp_start.setRange(-50, 500)
        temp_start.setDecimals(1)
        temp_start.setValue(50.0)

        temp_step_label = QLabel('间隔(℃):')
        temp_step = QDoubleSpinBox()
        temp_step.setRange(0.1, 200)
        temp_step.setDecimals(1)
        temp_step.setValue(50.0)

        temp_end_label = QLabel('最终温度(℃):')
        temp_end = QDoubleSpinBox()
        temp_end.setRange(-50, 800)
        temp_end.setDecimals(1)
        temp_end.setValue(600.0)

        temp_range_layout.addWidget(temp_start_label)
        temp_range_layout.addWidget(temp_start, 1)
        temp_range_layout.addWidget(temp_step_label)
        temp_range_layout.addWidget(temp_step, 1)
        temp_range_layout.addWidget(temp_end_label)
        temp_range_layout.addWidget(temp_end, 1)
        temp_grid.addWidget(temp_range_row, 1, 1, 1, 3)

        temp_grid.addWidget(QLabel('温度波动阈值 (°C):'), 2, 0)
        self.temp_fluc_spin = QDoubleSpinBox()
        self.temp_fluc_spin.setRange(0, 50)
        self.temp_fluc_spin.setDecimals(2)
        self.temp_fluc_spin.setValue(0.20)
        temp_grid.addWidget(self.temp_fluc_spin, 2, 1)

        temp_grid.addWidget(QLabel('温度状态:'), 2, 2)
        self.current_temp_display = QLineEdit('待稳')
        self.current_temp_display.setReadOnly(True)
        temp_grid.addWidget(self.current_temp_display, 2, 3)

        temp_box.setLayout(temp_grid)

        press_box = PanelBox('压力标定')
        press_grid = QGridLayout()
        press_grid.setHorizontalSpacing(14)
        press_grid.setVerticalSpacing(10)

        press_grid.addWidget(QLabel('压力标定:'), 0, 0)
        self.press_cal_btn = QPushButton()
        self.press_cal_btn.setObjectName('CalToggleButton')
        self.press_cal_btn.setCheckable(True)
        self.press_cal_btn.setChecked(False)
        self.press_cal_btn.toggled.connect(
            lambda checked: self._sync_cal_toggle_text(self.press_cal_btn, checked)
        )
        self._sync_cal_toggle_text(self.press_cal_btn, False)
        press_grid.addWidget(self.press_cal_btn, 0, 1)

        press_grid.addWidget(QLabel('参考压力 (kPa):'), 0, 2)
        self.ref_press_spin = QDoubleSpinBox()
        self.ref_press_spin.setRange(0, 5000)
        self.ref_press_spin.setValue(101.3)
        self.ref_press_spin.setDecimals(1)
        press_grid.addWidget(self.ref_press_spin, 0, 3)

        press_grid.addWidget(QLabel('压力范围:'), 1, 0)
        press_range_row = QFrame()
        press_range_layout = QHBoxLayout(press_range_row)
        press_range_layout.setContentsMargins(0, 0, 0, 0)
        press_range_layout.setSpacing(8)

        press_start_label = QLabel('起始压力(kPa):')
        press_start = QDoubleSpinBox()
        press_start.setRange(0, 5000)
        press_start.setDecimals(1)
        press_start.setValue(101.3)

        press_step_label = QLabel('间隔(kPa):')
        press_step = QDoubleSpinBox()
        press_step.setRange(0.1, 500)
        press_step.setDecimals(1)
        press_step.setValue(5.0)

        press_end_label = QLabel('最终压力(kPa):')
        press_end = QDoubleSpinBox()
        press_end.setRange(0, 5000)
        press_end.setDecimals(1)
        press_end.setValue(121.3)

        press_range_layout.addWidget(press_start_label)
        press_range_layout.addWidget(press_start, 1)
        press_range_layout.addWidget(press_step_label)
        press_range_layout.addWidget(press_step, 1)
        press_range_layout.addWidget(press_end_label)
        press_range_layout.addWidget(press_end, 1)
        press_grid.addWidget(press_range_row, 1, 1, 1, 3)

        press_grid.addWidget(QLabel('压力波动阈值 (kPa):'), 2, 0)
        self.press_fluc_spin = QDoubleSpinBox()
        self.press_fluc_spin.setRange(0, 100)
        self.press_fluc_spin.setDecimals(2)
        self.press_fluc_spin.setValue(0.50)
        press_grid.addWidget(self.press_fluc_spin, 2, 1)

        press_grid.addWidget(QLabel('压力状态:'), 2, 2)
        self.current_press_display = QLineEdit('待稳')
        self.current_press_display.setReadOnly(True)
        press_grid.addWidget(self.current_press_display, 2, 3)

        press_box.setLayout(press_grid)

        layout.addWidget(temp_box, 1)
        layout.addWidget(press_box, 1)
        box.setLayout(layout)
        return box

    def build_middle_section(self):
        container = QFrame()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        left = PanelBox('标定流程设置')
        left_grid = QGridLayout()
        left_grid.setHorizontalSpacing(12)
        left_grid.setVerticalSpacing(10)

        row = 0
        left_grid.addWidget(QLabel('标定模式:'), row, 0)
        mode_select = QComboBox(); mode_select.addItems(['寿命标定', '光谱标定'])
        left_grid.addWidget(mode_select, row, 1, 1, 3)
        row += 1

        left_grid.addWidget(QLabel('选择保存路径:'), row, 0)
        self.save_path_edit = QLineEdit('D:/CalibrationData/LaserLifetime')
        left_grid.addWidget(self.save_path_edit, row, 1, 1, 2)
        self.browse_btn = QPushButton('浏览...')
        left_grid.addWidget(self.browse_btn, row, 3)
        row += 1

        left_grid.addWidget(QLabel('光源稳定时间 (s):'), row, 0)
        self.stable_time_spin = QSpinBox(); self.stable_time_spin.setRange(0, 3600); self.stable_time_spin.setValue(10)
        left_grid.addWidget(self.stable_time_spin, row, 1)
        left_grid.addWidget(QLabel('温稳时间 (s):'), row, 2)
        self.temp_stable_time_spin = QSpinBox(); self.temp_stable_time_spin.setRange(0, 3600); self.temp_stable_time_spin.setValue(30)
        left_grid.addWidget(self.temp_stable_time_spin, row, 3)
        row += 1

        left_grid.addWidget(QLabel('压稳时间 (s):'), row, 0)
        self.press_stable_time_spin = QSpinBox(); self.press_stable_time_spin.setRange(0, 3600); self.press_stable_time_spin.setValue(30)
        left_grid.addWidget(self.press_stable_time_spin, row, 1)
        left_grid.addWidget(QLabel('系统预热/延时'), row, 2)
        row += 1

        action_row = QFrame()
        action_layout = QHBoxLayout(action_row)
        action_layout.setContentsMargins(0, 0, 0, 0)
        action_layout.setSpacing(8)
        self.init_btn = QPushButton('初始化')
        self.start_btn = QPushButton('开始')
        self.start_btn.setObjectName('AccentButton')
        action_layout.addWidget(self.init_btn, 1)
        action_layout.addWidget(self.start_btn, 2)
        left_grid.addWidget(action_row, row, 0, 1, 4)

        left.setLayout(left_grid)

        scope_box = PanelBox('示波器设置')
        right_grid = QGridLayout()
        right_grid.setHorizontalSpacing(12)
        right_grid.setVerticalSpacing(10)

        right_grid.addWidget(QLabel('采样模式:'), 0, 0)
        self.single_mode_btn = QPushButton()
        self.single_mode_btn.setObjectName('SingleModeButton')
        self.single_mode_btn.setCheckable(True)
        self.single_mode_btn.setChecked(True)
        self.single_mode_btn.toggled.connect(self._sync_single_mode_text)
        self._sync_single_mode_text(True)
        self.single_mode_btn.setToolTip('按下=单次采样，未按下=连续采样')
        right_grid.addWidget(self.single_mode_btn, 0, 1)

        right_grid.addWidget(QLabel('循环次数:'), 0, 2)
        self.cycles_spin = QSpinBox(); self.cycles_spin.setRange(1, 9999); self.cycles_spin.setValue(10)
        right_grid.addWidget(self.cycles_spin, 0, 3)

        right_grid.addWidget(QLabel('保存通道:'), 1, 0)
        self.channel_combo = QComboBox(); self.channel_combo.addItems(['CH1', 'CH2', 'CH3', 'CH4', 'CH1 + CH2'])
        right_grid.addWidget(self.channel_combo, 1, 1)

        right_grid.addWidget(QLabel('是否平均:'), 1, 2)
        self.avg_checkbox = QCheckBox('启用平均')
        self.avg_checkbox.setChecked(True)
        right_grid.addWidget(self.avg_checkbox, 1, 3)

        self.avg_count_label = QLabel('平均次数:')
        right_grid.addWidget(self.avg_count_label, 2, 0)
        self.avg_times_spin = QSpinBox(); self.avg_times_spin.setRange(1, 4096); self.avg_times_spin.setValue(64)
        right_grid.addWidget(self.avg_times_spin, 2, 1)

        scope_box.setLayout(right_grid)

        hotplate = PanelBox('加热台手动控制')
        hotplate_grid = QGridLayout()
        hotplate_grid.setHorizontalSpacing(12)
        hotplate_grid.setVerticalSpacing(10)

        hotplate_grid.addWidget(QLabel('TF (℃):'), 0, 0)
        self.hotplate_tf_spin = QDoubleSpinBox()
        self.hotplate_tf_spin.setRange(-100, 600)
        self.hotplate_tf_spin.setDecimals(1)
        self.hotplate_tf_spin.setValue(25.0)
        hotplate_grid.addWidget(self.hotplate_tf_spin, 0, 1)

        self.hotplate_hold_btn = QPushButton('Hold')
        hotplate_grid.addWidget(self.hotplate_hold_btn, 0, 2)

        hotplate_grid.addWidget(QLabel('Rate (℃/min):'), 1, 0)
        self.hotplate_rate_spin = QDoubleSpinBox()
        self.hotplate_rate_spin.setRange(0.1, 100)
        self.hotplate_rate_spin.setDecimals(2)
        self.hotplate_rate_spin.setValue(5.0)
        hotplate_grid.addWidget(self.hotplate_rate_spin, 1, 1)

        self.hotplate_ramp_btn = QPushButton('Ramp')
        hotplate_grid.addWidget(self.hotplate_ramp_btn, 1, 2)

        self.hotplate_stop_btn = QPushButton('Stop')
        self.hotplate_stop_btn.setObjectName('StopButton')
        hotplate_grid.addWidget(self.hotplate_stop_btn, 2, 0, 1, 3)

        hotplate.setLayout(hotplate_grid)

        right_column = QFrame()
        right_column_layout = QVBoxLayout(right_column)
        right_column_layout.setContentsMargins(0, 0, 0, 0)
        right_column_layout.setSpacing(10)
        right_column_layout.addWidget(scope_box, 3)
        right_column_layout.addWidget(hotplate, 2)

        layout.addWidget(left, 3)
        layout.addWidget(right_column, 2)
        return container

    def build_bottom_section(self):
        container = QFrame()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        log1 = PanelBox('温压过程日志')
        v1 = QVBoxLayout()
        self.sensor_log_text = QTextEdit()
        self.sensor_log_text.setReadOnly(True)
        self.sensor_log_text.setPlainText(
            '[说明] 记录实时温度、压力、稳定判定进度'
        )
        v1.addWidget(self.sensor_log_text)
        log1.setLayout(v1)

        log2 = PanelBox('运行事件日志')
        v2 = QVBoxLayout()
        self.status_log_text = QTextEdit()
        self.status_log_text.setReadOnly(True)
        self.status_log_text.setPlainText(
            '[说明] 记录系统事件、设备状态、任务流程'
        )
        v2.addWidget(self.status_log_text)
        log2.setLayout(v2)

        layout.addWidget(log1, 1)
        layout.addWidget(log2, 1)
        return container

    def apply_styles(self):
        font = QFont('Microsoft YaHei UI', 10)
        QApplication.instance().setFont(font)
        self.setStyleSheet('''
            QWidget {
                background: #f3f5f7;
                color: #222;
            }
            #MainTitle {
                font-size: 22px;
                font-weight: 700;
                color: #1f3a56;
                padding: 6px 0 10px 0;
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
                min-height: 220px;
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
            QPushButton#SingleModeButton {
                background: #edf2f7;
                border: 1px solid #91a5ba;
                font-weight: 600;
            }
            QPushButton#SingleModeButton:checked {
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
            QCheckBox {
                min-height: 28px;
            }
        ''')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = LaserLifetimeCalibrationUI()
    window.show()
    sys.exit(app.exec_())
