from __future__ import annotations

import re
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from serial.tools import list_ports

BAUD_CANDIDATES = (9600, 115200)
IDN_SIGNATURES = ("instec,mk2000b",)

POLL_INTERVAL_S = 0.25
PLOT_TICK_S = 0.2
TIMEOUT_S = 0.1
PROBE_WAIT_S = 0.1
POLL_WAIT_S = 0.35
TARGET_TOL_C = 0.5
TARGET_HOLD_S = 20.0
PRESS_CONTROL_SLAVE = 2


class TempControllerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Virtual Com Port for MK2000 and MK2000B (SCPI)")
        self.root.geometry("1280x760")

        self.ser: serial.Serial | None = None
        self.ser_lock = threading.Lock()
        self.query_lock = threading.RLock()

        self.connect_thread: threading.Thread | None = None
        self.connect_busy = False
        self.poll_thread: threading.Thread | None = None
        self.stop_poll = threading.Event()

        self.active_baud: int | None = None
        self.cmd_term = "\r"
        self.control_slave = 1
        self.press_control_slave = PRESS_CONTROL_SLAVE

        self.port_var = tk.StringVar(value="")
        self.sn_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Status: Not connected")
        self.tc_var = tk.StringVar(value="0.00")
        self.pk_var = tk.StringVar(value="0.000")
        self.tf_var = tk.StringVar(value="25")
        self.rate_var = tk.StringVar(value="5")
        self.pf_var = tk.StringVar(value="101.3")
        self.prate_var = tk.StringVar(value="5")
        self.cmd_var = tk.StringVar(value="*IDN?")
        self.data_var = tk.StringVar(value="")

        self.t0 = time.time()
        self.temp_time_data: list[float] = []
        self.tc_data: list[float] = []
        self.tf_data: list[float] = []
        self.press_time_data: list[float] = []
        self.pk_data: list[float] = []
        self.pf_data: list[float] = []
        self.sim_tf_current = 25.0
        self.sim_tf_target = 25.0
        self.sim_rate_cpm = 0.0
        self.sim_temp_mode = "hold"  # "hold" | "ramp"
        self.sim_pf_current = 101.3
        self.sim_pf_target = 101.3
        self.sim_prate_kpm = 0.0
        self.sim_press_mode = "hold"  # "hold" | "ramp"
        self.last_sim_ts = time.time()
        self.last_tc_value = 0.0
        self.last_pk_value = 0.0
        self.poll_count = 0
        self.io_seq = 0
        self.last_s_status: int | None = None
        self.ramp_target_temp: float | None = None
        self.reach_inband_since: float | None = None
        self.reach_confirmed = False

        self._build_ui()
        self.refresh_ports(auto_select=True)
        self._schedule_status_refresh()
        self._schedule_plot_tick()

    def _build_ui(self) -> None:
        self.root.grid_columnconfigure(0, weight=4)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="nsew")
        right = ttk.Frame(self.root, padding=(0, 10, 10, 10))
        right.grid(row=0, column=1, sticky="nsew")

        top_bar = ttk.Frame(left)
        top_bar.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(
            top_bar,
            text="Virtual Com Port for MK2000 and MK2000B(SCPI)",
            font=("Segoe UI", 26, "bold"),
        ).pack(side=tk.LEFT, anchor="w")

        self.chart_tabs = ttk.Notebook(left)
        self.chart_tabs.pack(fill=tk.BOTH, expand=True)

        self.temp_page = ttk.Frame(self.chart_tabs)
        temp_fig = Figure(figsize=(8, 5), dpi=100)
        self.temp_ax = temp_fig.add_subplot(111)
        self.temp_ax.set_facecolor("black")
        temp_fig.patch.set_facecolor("#d9d9d9")
        self.temp_ax.grid(color="#00aa00", linestyle="-", linewidth=0.6)
        self.temp_ax.set_xlabel("Time (s)")
        self.temp_ax.set_ylabel("Temperature (C)")
        self.line_tc, = self.temp_ax.plot([], [], color="#ffffff", linewidth=1.8, marker="o", markersize=2.5, label="TC")
        self.line_tf, = self.temp_ax.plot([], [], color="#ff5555", linewidth=1.8, marker="o", markersize=2.5, label="TF")
        self.temp_ax.legend(loc="upper right")
        self.temp_canvas = FigureCanvasTkAgg(temp_fig, master=self.temp_page)
        self.temp_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.press_page = ttk.Frame(self.chart_tabs)
        press_fig = Figure(figsize=(8, 5), dpi=100)
        self.press_ax = press_fig.add_subplot(111)
        self.press_ax.set_facecolor("black")
        press_fig.patch.set_facecolor("#d9d9d9")
        self.press_ax.grid(color="#00aa00", linestyle="-", linewidth=0.6)
        self.press_ax.set_xlabel("Time (s)")
        self.press_ax.set_ylabel("Pressure (kPa)")
        self.line_pk, = self.press_ax.plot([], [], color="#66ccff", linewidth=1.8, marker="o", markersize=2.5, label="PK")
        self.line_pf, = self.press_ax.plot([], [], color="#ffaa44", linewidth=1.8, marker="o", markersize=2.5, label="PF")
        self.press_ax.legend(loc="upper right")
        self.press_canvas = FigureCanvasTkAgg(press_fig, master=self.press_page)
        self.press_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.chart_tabs.add(self.temp_page, text="温度曲线图")
        self.chart_tabs.add(self.press_page, text="压力曲线图")

        sec_port = ttk.LabelFrame(right, text="Com Port select", padding=10)
        sec_port.pack(fill=tk.X, pady=(0, 8))
        self.port_combo = ttk.Combobox(sec_port, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        sec_port.grid_columnconfigure(0, weight=1)
        sec_port.grid_columnconfigure(1, weight=1)
        sec_port.grid_columnconfigure(2, weight=1)
        sec_port.grid_columnconfigure(3, weight=1)
        self.btn_refresh = ttk.Button(sec_port, text="Refresh", command=lambda: self.refresh_ports(auto_select=False))
        self.btn_refresh.grid(row=1, column=0, sticky="ew")
        self.btn_connect = ttk.Button(sec_port, text="Connect", command=self.connect)
        self.btn_connect.grid(row=1, column=1, sticky="ew", padx=4)
        self.btn_auto = ttk.Button(sec_port, text="Auto Detect", command=self.auto_detect_and_connect)
        self.btn_auto.grid(row=1, column=2, sticky="ew")
        self.btn_disconnect = ttk.Button(sec_port, text="Disconnect", command=self.disconnect)
        self.btn_disconnect.grid(row=1, column=3, sticky="ew")
        ttk.Label(sec_port, text="SN").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(sec_port, textvariable=self.sn_var, state="readonly").grid(row=3, column=0, columnspan=4, sticky="ew")

        sec_tc = ttk.LabelFrame(right, text="Current", padding=10)
        sec_tc.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(sec_tc, text="TC", font=("Segoe UI", 38, "bold"), foreground="red").grid(row=0, column=0, sticky="w")
        ttk.Label(sec_tc, textvariable=self.tc_var, font=("Segoe UI", 42, "bold"), foreground="red").grid(row=0, column=1, sticky="w", padx=(12, 0))
        ttk.Label(sec_tc, text="PK(kPa)", font=("Segoe UI", 12, "bold")).grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(sec_tc, textvariable=self.pk_var, font=("Segoe UI", 14, "bold")).grid(row=1, column=1, sticky="w", padx=(12, 0), pady=(6, 0))

        sec_ctrl = ttk.LabelFrame(right, text="Control", padding=10)
        sec_ctrl.pack(fill=tk.X, pady=(0, 8))
        sec_ctrl.grid_columnconfigure(0, weight=1)
        sec_ctrl.grid_columnconfigure(1, weight=1)

        temp_ctrl = ttk.LabelFrame(sec_ctrl, text="Temperature", padding=8)
        temp_ctrl.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(temp_ctrl, text="TF").grid(row=0, column=0, sticky="w")
        ttk.Entry(temp_ctrl, textvariable=self.tf_var, width=10).grid(row=0, column=1, sticky="w")
        ttk.Button(temp_ctrl, text="Hold", command=self.send_hold).grid(row=0, column=2, sticky="ew", padx=(10, 0))
        ttk.Label(temp_ctrl, text="Rate").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(temp_ctrl, textvariable=self.rate_var, width=10).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Button(temp_ctrl, text="Ramp", command=self.send_ramp).grid(row=1, column=2, sticky="ew", padx=(10, 0), pady=(8, 0))
        ttk.Button(temp_ctrl, text="Stop", command=self.send_stop).grid(row=2, column=2, sticky="ew", padx=(10, 0), pady=(8, 0))

        press_ctrl = ttk.LabelFrame(sec_ctrl, text="Pressure", padding=8)
        press_ctrl.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ttk.Label(press_ctrl, text="PF(kPa)").grid(row=0, column=0, sticky="w")
        ttk.Entry(press_ctrl, textvariable=self.pf_var, width=10).grid(row=0, column=1, sticky="w")
        ttk.Button(press_ctrl, text="P-Hold", command=self.send_press_hold).grid(row=0, column=2, sticky="ew", padx=(10, 0))
        ttk.Label(press_ctrl, text="P-Rate").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(press_ctrl, textvariable=self.prate_var, width=10).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Button(press_ctrl, text="P-Ramp", command=self.send_press_ramp).grid(row=1, column=2, sticky="ew", padx=(10, 0), pady=(8, 0))
        ttk.Button(press_ctrl, text="P-Stop", command=self.send_press_stop).grid(row=2, column=2, sticky="ew", padx=(10, 0), pady=(8, 0))

        sec_cmd = ttk.LabelFrame(right, text="Command", padding=10)
        sec_cmd.pack(fill=tk.BOTH, expand=True)
        ttk.Entry(sec_cmd, textvariable=self.cmd_var).pack(fill=tk.X)
        ttk.Button(sec_cmd, text="Send Command", command=self.send_custom_cmd).pack(anchor="w", pady=(8, 8))
        ttk.Label(sec_cmd, text="data").pack(anchor="w")
        ttk.Entry(sec_cmd, textvariable=self.data_var).pack(fill=tk.X)
        ttk.Label(sec_cmd, textvariable=self.status_var, wraplength=280, foreground="#444").pack(anchor="w", pady=(8, 0))

        log_frame = ttk.Frame(sec_cmd)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.log = tk.Text(log_frame, height=10)
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=sb.set)

        self.append_log("UI ready. Fast detect mode: 100ms probe + one force-clear retry.")

    def append_log(self, msg: str) -> None:
        if threading.current_thread() is not threading.main_thread():
            self.root.after(0, lambda m=msg: self.append_log(m))
            return
        ts = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{ts}] {msg}\n")
        self.log.see(tk.END)

    def detect_ports(self) -> list[str]:
        return sorted([p.device for p in list_ports.comports()])

    def refresh_ports(self, auto_select: bool) -> None:
        ports = self.detect_ports()
        self.port_combo["values"] = ports
        if auto_select:
            if ports and self.port_var.get() not in ports:
                self.port_var.set(ports[0])
            if not ports:
                self.port_var.set("")

    def _schedule_status_refresh(self) -> None:
        ports = self.detect_ports()
        shown = ", ".join(ports) if ports else "(none)"
        if self.is_connected():
            self.status_var.set(f"Status: connected @ {self.port_var.get()} ({self.active_baud}bps) | ports: {shown}")
        else:
            self.status_var.set(f"Status: not connected | ports: {shown}")
        self.root.after(1200, self._schedule_status_refresh)

    def _schedule_plot_tick(self) -> None:
        # Keep TF/PF as simulated setpoint curves independent of device reply format.
        now = time.time()
        dt = max(0.0, now - self.last_sim_ts)
        self.last_sim_ts = now

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

        # Keep curve alive even when no fresh poll arrives.
        self._push_temp_plot(self.last_tc_value, self.sim_tf_current)
        self._push_press_plot(self.last_pk_value, self.sim_pf_current)
        self.root.after(int(PLOT_TICK_S * 1000), self._schedule_plot_tick)

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _set_connect_busy(self, busy: bool) -> None:
        self.connect_busy = busy
        s = "disabled" if busy else "normal"
        self.btn_connect.configure(state=s)
        self.btn_auto.configure(state=s)
        self.btn_refresh.configure(state=s)
        self.btn_disconnect.configure(state=s)
        self.port_combo.configure(state="disabled" if busy else "readonly")

    def _next_io_seq(self) -> int:
        self.io_seq += 1
        return self.io_seq

    def connect(self) -> None:
        if self.connect_busy:
            self.append_log("Connect is running. Please wait.")
            return
        port = self.port_var.get().strip()
        if not port:
            self.append_log("No COM selected.")
            return
        self._set_connect_busy(True)
        self.connect_thread = threading.Thread(target=self._connect_worker, args=([port],), daemon=True)
        self.connect_thread.start()

    def auto_detect_and_connect(self) -> None:
        if self.connect_busy:
            self.append_log("Connect is running. Please wait.")
            return
        ports = self.detect_ports()
        if not ports:
            self.append_log("No COM ports detected.")
            return
        self._set_connect_busy(True)
        self.connect_thread = threading.Thread(target=self._connect_worker, args=(ports,), daemon=True)
        self.connect_thread.start()

    def _connect_worker(self, ports: list[str]) -> None:
        self.disconnect(log_it=False)
        self.append_log(f"Connect scan: {', '.join(ports)}")
        for port in ports:
            if self._try_connect_port(port):
                self.root.after(0, lambda p=port: self.port_var.set(p))
                self.root.after(0, self._reset_plot_after_connect)
                self.append_log(f"Connected on {port}")
                self.start_polling()
                self.root.after(0, lambda: self._set_connect_busy(False))
                return
        self.append_log("No MK2000B found on scanned ports.")
        self.root.after(0, lambda: self._set_connect_busy(False))

    def _reset_plot_after_connect(self) -> None:
        self.t0 = time.time()
        self.temp_time_data.clear()
        self.tc_data.clear()
        self.tf_data.clear()
        self.press_time_data.clear()
        self.pk_data.clear()
        self.pf_data.clear()
        self.line_tc.set_data([], [])
        self.line_tf.set_data([], [])
        self.line_pk.set_data([], [])
        self.line_pf.set_data([], [])
        self.temp_ax.set_xlim(0, 1)
        self.press_ax.set_xlim(0, 1)
        tf_now = self._get_tf_value()
        t_lo, t_hi = self._compute_y_limits([tf_now, tf_now])
        self.temp_ax.set_ylim(t_lo, t_hi)
        try:
            pf_now = float(self.pf_var.get().strip())
        except Exception:
            pf_now = 101.3
        p_lo, p_hi = self._compute_y_limits([pf_now, pf_now])
        self.press_ax.set_ylim(p_lo, p_hi)
        self.temp_canvas.draw_idle()
        self.press_canvas.draw_idle()

    def _try_connect_port(self, port: str) -> bool:
        for baud in BAUD_CANDIDATES:
            try:
                self.ser = serial.Serial(
                    port,
                    baud,
                    timeout=TIMEOUT_S,
                    write_timeout=0.3,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.1)
                self.append_log(f"Opened {port} @ {baud}, probing...")
                if self._probe_idn_fast() or self._probe_rtinfo_fast():
                    self.active_baud = baud
                    self.ser.timeout = 0.2
                    base_tf = self._get_tf_value()
                    try:
                        base_pf = float(self.pf_var.get().strip())
                    except Exception:
                        base_pf = 101.3
                    self.sim_tf_current = base_tf
                    self.sim_tf_target = base_tf
                    self.sim_rate_cpm = 0.0
                    self.sim_temp_mode = "hold"
                    self.sim_pf_current = base_pf
                    self.sim_pf_target = base_pf
                    self.sim_prate_kpm = 0.0
                    self.sim_press_mode = "hold"
                    return True
                self.disconnect(log_it=False)
            except Exception as e:
                self.append_log(f"Open failed {port}@{baud}: {e}")
                self.ser = None
        return False

    def _probe_idn_fast(self) -> bool:
        resp = self.query("*IDN?", term="\r", max_wait_s=PROBE_WAIT_S)
        self.append_log(f"IDN probe[1] => {resp!r}")
        if any(sig in resp.lower() for sig in IDN_SIGNATURES):
            self.root.after(0, lambda s=resp: self.sn_var.set(s))
            return True

        self._force_clear_once()
        resp = self.query("*IDN?", term="\r", max_wait_s=PROBE_WAIT_S)
        self.append_log(f"IDN probe[2] => {resp!r}")
        if any(sig in resp.lower() for sig in IDN_SIGNATURES):
            self.root.after(0, lambda s=resp: self.sn_var.set(s))
            return True
        return False

    def _probe_rtinfo_fast(self) -> bool:
        resp = self.query("TEMPerature:RTINformation?", term="\r", max_wait_s=PROBE_WAIT_S)
        self.append_log(f"RT probe[1] => {resp!r}")
        tc, pk, tf = self._extract_temp_pressure(resp)
        if tc is not None:
            self._set_live_values(tc, pk)
            if tf is not None:
                self.root.after(0, lambda v=f"{tf:.3f}": self.tf_var.set(v))
            if not self.sn_var.get().strip():
                self.root.after(0, lambda: self.sn_var.set("MK2000B (RTINformation)"))
            return True

        self._force_clear_once()
        resp = self.query("TEMPerature:RTINformation?", term="\r", max_wait_s=PROBE_WAIT_S)
        self.append_log(f"RT probe[2] => {resp!r}")
        tc, pk, tf = self._extract_temp_pressure(resp)
        if tc is not None:
            self._set_live_values(tc, pk)
            if tf is not None:
                self.root.after(0, lambda v=f"{tf:.3f}": self.tf_var.set(v))
            if not self.sn_var.get().strip():
                self.root.after(0, lambda: self.sn_var.set("MK2000B (RTINformation)"))
            return True
        return False

    def _force_clear_once(self) -> None:
        if not self.is_connected():
            return
        assert self.ser is not None
        with self.ser_lock:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.flush()

    def disconnect(self, log_it: bool = True) -> None:
        self.stop_poll.set()
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.active_baud = None
        if log_it:
            self.append_log("Disconnected.")

    def _send_cmd_unlocked(self, cmd: str, term: str | None = None, source: str = "io") -> int:
        if not self.is_connected():
            raise RuntimeError("Device not connected")
        assert self.ser is not None
        actual_term = "\r" if term is None else term
        payload = (cmd + actual_term).encode("ascii")
        seq = self._next_io_seq()
        self.append_log(f"TX#{seq} [{source}] {cmd!r}")
        with self.ser_lock:
            self.ser.reset_input_buffer()
            self.ser.write(payload)
        return seq

    def send_cmd(self, cmd: str, term: str | None = None, source: str = "manual") -> None:
        with self.query_lock:
            self._send_cmd_unlocked(cmd, term=term, source=source)

    def _read_response(self, max_wait_s: float, tail_wait_s: float = 0.08) -> str:
        assert self.ser is not None
        deadline = time.time() + max_wait_s
        chunks: list[bytes] = []
        last = 0.0

        while time.time() < deadline:
            with self.ser_lock:
                n = self.ser.in_waiting
                if n > 0:
                    chunks.append(self.ser.read(n))
                    last = time.time()
            if chunks and (time.time() - last) > tail_wait_s:
                break
            time.sleep(0.01)

        if not chunks:
            with self.ser_lock:
                b = self.ser.read(128)
            if b:
                chunks.append(b)

        return b"".join(chunks).decode("ascii", errors="ignore").strip("\x00\r\n ")

    def query(self, cmd: str, term: str | None = None, max_wait_s: float = POLL_WAIT_S, source: str = "query") -> str:
        if not self.is_connected():
            return ""
        with self.query_lock:
            seq = self._send_cmd_unlocked(cmd, term=term, source=source)
            resp = self._read_response(max_wait_s=max_wait_s)
            self.append_log(f"RX#{seq} [{source}] {resp!r}")
            return resp

    def start_polling(self) -> None:
        self.stop_poll.clear()
        if self.poll_thread and self.poll_thread.is_alive():
            return
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def _poll_loop(self) -> None:
        while not self.stop_poll.is_set() and self.is_connected():
            try:
                rt_resp = self.query("TEMPerature:RTINformation?", max_wait_s=POLL_WAIT_S, source="poll-rt")
                tc_rt, pk_rt, tf_rt, tt_rt, s_status, err_code = self._extract_rt_channels(rt_resp, self.control_slave)

                # Temperature uses dedicated command as primary source to avoid field-order ambiguity.
                ct_resp = self.query("TEMPerature:CTEMperature?", max_wait_s=POLL_WAIT_S, source="poll-ct")
                ct_vals = self._extract_float_list(ct_resp)
                tc_ct = ct_vals[0] if len(ct_vals) >= 1 else None
                # Per your device behavior: CTEMperature? second item is pressure (kPa).
                pk_ct = ct_vals[1] if len(ct_vals) >= 2 else None
                tc_final = tc_ct if tc_ct is not None else tc_rt
                pk_final = pk_ct if pk_ct is not None else pk_rt

                if tc_final is not None:
                    self._set_live_values(tc_final, pk_final)
                    # keep TF curve from simulation logic to avoid step jumps

                self._update_reach_judge(tc_final, s_status, tf_rt, tt_rt, err_code)

                if s_status is not None and s_status != self.last_s_status:
                    self.append_log(f"STATE s_status changed: {self.last_s_status} -> {s_status} (0-stop,1-hold,2-ramp)")
                    if self.last_s_status == 2 and s_status == 0:
                        self._run_async_io(self._diagnose_unexpected_stop, "StopDiag")
                    self.last_s_status = s_status

                self.poll_count += 1
                if self.poll_count % 8 == 0:
                    self.append_log(
                        f"DBG RT={rt_resp!r} -> tc1={tc_rt}, pk2={pk_rt}, tf1={tf_rt}, tt1={tt_rt}, s={s_status}, err={err_code}; "
                        f"CT={ct_resp!r} -> vals={ct_vals}; final_tc={tc_final}, final_pk={pk_final}"
                    )
            except Exception as e:
                self.append_log(f"Poll error: {e}")
            time.sleep(POLL_INTERVAL_S)

    def _set_live_values(self, tc: float, pk: float) -> None:
        self.last_tc_value = tc
        self.last_pk_value = pk
        self.root.after(0, lambda v=f"{tc:.2f}": self.tc_var.set(v))
        self.root.after(0, lambda v=f"{pk:.3f}": self.pk_var.set(v))
        # Update both pages at the same data point as numeric display.
        self._push_temp_plot(tc, self.sim_tf_current)
        self._push_press_plot(pk, self.sim_pf_current)

    def _get_tf_value(self) -> float:
        try:
            return float(self.tf_var.get().strip())
        except Exception:
            return 25.0

    def _push_temp_plot(self, tc: float, tf: float) -> None:
        t = time.time() - self.t0
        self.temp_time_data.append(t)
        self.tc_data.append(tc)
        self.tf_data.append(tf)
        self.root.after(0, self._redraw_temp_plot)

    def _push_press_plot(self, pk: float, pf: float) -> None:
        t = time.time() - self.t0
        self.press_time_data.append(t)
        self.pk_data.append(pk)
        self.pf_data.append(pf)
        self.root.after(0, self._redraw_press_plot)

    def _redraw_temp_plot(self) -> None:
        if not self.temp_time_data:
            return

        xs = list(self.temp_time_data)
        ys_tc = list(self.tc_data)
        ys_tf = list(self.tf_data)

        if len(xs) == 1:
            xs = [xs[0] - 0.1, xs[0]]
            ys_tc = [ys_tc[0], ys_tc[0]]
            ys_tf = [ys_tf[0], ys_tf[0]]

        self.line_tc.set_data(xs, ys_tc)
        self.line_tf.set_data(xs, ys_tf)
        # Keep timeline continuously extending; do not use rolling-window refresh.
        self.temp_ax.set_xlim(0, xs[-1] + 1)

        y_min, y_max = self._compute_y_limits(ys_tc + ys_tf)
        self.temp_ax.set_ylim(y_min, y_max)
        self.temp_canvas.draw_idle()

    def _redraw_press_plot(self) -> None:
        if not self.press_time_data:
            return

        xs = list(self.press_time_data)
        ys_pk = list(self.pk_data)
        ys_pf = list(self.pf_data)

        if len(xs) == 1:
            xs = [xs[0] - 0.1, xs[0]]
            ys_pk = [ys_pk[0], ys_pk[0]]
            ys_pf = [ys_pf[0], ys_pf[0]]

        self.line_pk.set_data(xs, ys_pk)
        self.line_pf.set_data(xs, ys_pf)
        self.press_ax.set_xlim(0, xs[-1] + 1)

        y_min, y_max = self._compute_y_limits(ys_pk + ys_pf)
        self.press_ax.set_ylim(y_min, y_max)
        self.press_canvas.draw_idle()

    @staticmethod
    def _compute_y_limits(values: list[float]) -> tuple[float, float]:
        if not values:
            return 0.0, 1.0
        raw_min = min(values)
        raw_max = max(values)
        span = max(raw_max - raw_min, 1.0)

        # Keep more headroom on top so curves do not stick to the upper edge.
        top_pad = max(span * 0.22, abs(raw_max) * 0.02, 1.5)
        bottom_pad = max(span * 0.12, abs(raw_min) * 0.01, 1.0)
        lo = raw_min - bottom_pad
        hi = raw_max + top_pad
        if hi <= lo:
            hi = lo + 1.0
        return lo, hi

    @staticmethod
    def _extract_temp_pressure(resp: str) -> tuple[float | None, float, float | None]:
        if not resp:
            return None, 0.0, None

        text = resp.replace("\x00", " ").strip()
        # Read only MK data frames, so mixed replies like "...\\r\\nMK:..." remain parseable.
        mk_frames = re.findall(r"MK:\d+:[^\r\n]*", text, flags=re.IGNORECASE)
        candidates = list(reversed(mk_frames)) if mk_frames else [text]

        # Protocol rule used here: first numeric field is temperature (TC), second is pressure (PK).
        # Keep TF as optional third numeric field when present.
        for item in candidates:
            m = re.search(r"MK:\d+:([^\r\n]*)", item, flags=re.IGNORECASE)
            payload = m.group(1) if m else item

            nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", payload)
            if len(nums) >= 2:
                tc = float(nums[0])
                pk = float(nums[1])
                tf = float(nums[2]) if len(nums) >= 3 else None
                return tc, pk, tf
            if len(nums) == 1:
                return float(nums[0]), 0.0, None

        return None, 0.0, None

    @staticmethod
    def _extract_first_float(resp: str) -> float | None:
        if not resp:
            return None
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)
        if not nums:
            return None
        return float(nums[0])

    @staticmethod
    def _extract_float_list(resp: str) -> list[float]:
        if not resp:
            return []
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)
        return [float(x) for x in nums]

    @staticmethod
    def _extract_s_status(resp: str) -> int | None:
        if not resp:
            return None
        # Parse last MK frame: MK:sx:tc:tm:tf:tt:rt:pp:s_status:...
        frames = re.findall(r"MK:\d+:[^\r\n]*", resp, flags=re.IGNORECASE)
        if not frames:
            return None
        parts = frames[-1].split(":")
        if len(parts) < 9:
            return None
        try:
            return int(float(parts[8]))
        except Exception:
            return None

    @staticmethod
    def _extract_rt_channels(resp: str, control_slave: int = 1) -> tuple[float | None, float | None, float | None, float | None, int | None, int | None]:
        if not resp:
            return None, None, None, None, None, None

        frames = re.findall(r"MK:\d+:[^\r\n]*", resp, flags=re.IGNORECASE)
        by_slave: dict[int, list[str]] = {}
        for fr in frames:
            parts = fr.split(":")
            if len(parts) < 10:
                continue
            try:
                sx = int(float(parts[1]))
            except Exception:
                continue
            by_slave[sx] = parts

        tc1: float | None = None
        tf1: float | None = None
        tt1: float | None = None
        s_status: int | None = None
        err_code: int | None = None
        pk2: float | None = None

        p1 = by_slave.get(control_slave)
        if p1 is not None:
            try:
                tc1 = float(p1[2])  # tc
            except Exception:
                pass
            try:
                tf1 = float(p1[4])  # tf
            except Exception:
                pass
            try:
                tt1 = float(p1[5])  # tt
            except Exception:
                pass
            try:
                s_status = int(float(p1[8]))  # s_status
            except Exception:
                pass
            try:
                err_code = int(float(p1[10].split(",")[-1])) if len(p1) > 10 else None
            except Exception:
                pass

        p2 = by_slave.get(2)
        if p2 is not None:
            try:
                pk2 = float(p2[2])  # tc field on slave2 is pressure in your setup
            except Exception:
                pass

        return tc1, pk2, tf1, tt1, s_status, err_code

    def _update_reach_judge(
        self,
        tc_now: float | None,
        s_status: int | None,
        tf_rt: float | None,
        tt_rt: float | None,
        err_code: int | None,
    ) -> None:
        if self.ramp_target_temp is None:
            return
        if tc_now is None:
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

            self.append_log(
                f"RAMP-JUDGE tc={tc_now:.3f}, target={target:.3f}, diff={diff:.3f}, "
                f"inband={inband_sec:.1f}/{TARGET_HOLD_S:.0f}s, tf={tf_rt}, tt={tt_rt}, err={err_code}"
            )
            if (not self.reach_confirmed) and inband_sec >= TARGET_HOLD_S:
                self.reach_confirmed = True
                self.append_log(
                    f"RAMP-JUDGE reached: |TC-target| <= {TARGET_TOL_C:.1f}C for {TARGET_HOLD_S:.0f}s"
                )
        elif s_status == 0 and not self.reach_confirmed:
            inband_sec = 0.0 if self.reach_inband_since is None else (now - self.reach_inband_since)
            self.append_log(
                f"WARN ramp stopped before reach-confirmed: tc={tc_now:.3f}, target={target:.3f}, "
                f"diff={diff:.3f}, inband={inband_sec:.1f}/{TARGET_HOLD_S:.0f}s, tf={tf_rt}, tt={tt_rt}, err={err_code}"
            )

    def _run_async_io(self, fn, name: str) -> None:
        def runner() -> None:
            try:
                fn()
            except Exception as e:
                self.append_log(f"{name} failed: {e}")

        threading.Thread(target=runner, daemon=True).start()

    def _diagnose_unexpected_stop(self) -> None:
        if not self.is_connected():
            return
        # Snapshot controller state immediately after unexpected ramp->stop transition.
        try:
            st = self.query("TEMPerature:STATus?", max_wait_s=POLL_WAIT_S, source="diag")
            er = self.query("TEMPerature:ERRor?", max_wait_s=POLL_WAIT_S, source="diag")
            ch = self.query("TEMPerature:CHSWitch?", max_wait_s=POLL_WAIT_S, source="diag")
            rg = self.query("TEMPerature:RANGe?", max_wait_s=POLL_WAIT_S, source="diag")
            op = self.query("TEMPerature:OPSLave?", max_wait_s=POLL_WAIT_S, source="diag")
            self.append_log(
                f"DIAG stop snapshot: STATus={st!r}, ERRor={er!r}, CHSWitch={ch!r}, "
                f"RANGe={rg!r}, OPSlave={op!r}"
            )
        except Exception as e:
            self.append_log(f"DIAG query failed: {e}")

    def send_hold(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        try:
            tf = float(self.tf_var.get().strip())
        except ValueError:
            self.append_log("Invalid TF.")
            return
        # Hold should set TF to target immediately (step change on plot).
        self.sim_tf_target = tf
        self.sim_tf_current = tf
        self.sim_rate_cpm = 0.0
        self.sim_temp_mode = "hold"
        self.ramp_target_temp = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        def hold_task() -> None:
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.control_slave}", source="ctrl-hold")
                self._send_cmd_unlocked(f"TEMPerature:HOLD {tf}", source="ctrl-hold")
        self._run_async_io(hold_task, "Hold")
        self.append_log(f"HOLD sent: {tf}")

    def send_ramp(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        try:
            tf = float(self.tf_var.get().strip())
            rt = float(self.rate_var.get().strip())
        except ValueError:
            self.append_log("Invalid TF/Rate.")
            return
        if rt <= 0:
            self.append_log("Rate must be > 0 C/min.")
            return
        # Match official UI behavior: ramp line starts from current TC.
        self.sim_tf_target = tf
        self.sim_tf_current = self.last_tc_value
        self.sim_rate_cpm = abs(rt)
        self.sim_temp_mode = "ramp"
        self.ramp_target_temp = tf
        self.reach_inband_since = None
        self.reach_confirmed = False
        def ramp_task() -> None:
            # Keep OPSlave + RAMP + verify in one serial transaction.
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.control_slave}", source="ctrl-ramp")
                self._send_cmd_unlocked(f"TEMPerature:RAMP {tf},{rt}", source="ctrl-ramp")
                seq = self._send_cmd_unlocked("TEMPerature:RTINformation?", source="ctrl-ramp-verify")
                st = self._read_response(max_wait_s=POLL_WAIT_S)
                self.append_log(f"RX#{seq} [ctrl-ramp-verify] {st!r}")
            self.append_log(f"RAMP verify RT => {st!r}")
        self._run_async_io(ramp_task, "Ramp")
        self.append_log(f"RAMP sent: tf={tf}, rate={rt}")

    def send_stop(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        self.ramp_target_temp = None
        self.reach_inband_since = None
        self.reach_confirmed = False
        def stop_task() -> None:
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.control_slave}", source="ctrl-stop")
                self._send_cmd_unlocked("TEMPerature:STOP", source="ctrl-stop")
        self._run_async_io(stop_task, "Stop")
        self.append_log("STOP sent")

    def send_press_hold(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        try:
            pf = float(self.pf_var.get().strip())
        except ValueError:
            self.append_log("Invalid PF.")
            return
        self.sim_pf_target = pf
        self.sim_pf_current = pf
        self.sim_prate_kpm = 0.0
        self.sim_press_mode = "hold"

        def hold_task() -> None:
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.press_control_slave}", source="ctrl-phold")
                self._send_cmd_unlocked(f"TEMPerature:HOLD {pf}", source="ctrl-phold")

        self._run_async_io(hold_task, "Pressure Hold")
        self.append_log(f"P-HOLD sent: {pf}")

    def send_press_ramp(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        try:
            pf = float(self.pf_var.get().strip())
            pr = float(self.prate_var.get().strip())
        except ValueError:
            self.append_log("Invalid PF/P-Rate.")
            return
        if pr <= 0:
            self.append_log("P-Rate must be > 0 kPa/min.")
            return
        self.sim_pf_target = pf
        self.sim_pf_current = self.last_pk_value
        self.sim_prate_kpm = abs(pr)
        self.sim_press_mode = "ramp"

        def ramp_task() -> None:
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.press_control_slave}", source="ctrl-pramp")
                self._send_cmd_unlocked(f"TEMPerature:RAMP {pf},{pr}", source="ctrl-pramp")
                seq = self._send_cmd_unlocked("TEMPerature:RTINformation?", source="ctrl-pramp-verify")
                st = self._read_response(max_wait_s=POLL_WAIT_S)
                self.append_log(f"RX#{seq} [ctrl-pramp-verify] {st!r}")
            self.append_log(f"P-RAMP verify RT => {st!r}")

        self._run_async_io(ramp_task, "Pressure Ramp")
        self.append_log(f"P-RAMP sent: pf={pf}, rate={pr}")

    def send_press_stop(self) -> None:
        if not self.is_connected():
            self.append_log("Not connected.")
            return
        self.sim_press_mode = "hold"
        self.sim_prate_kpm = 0.0

        def stop_task() -> None:
            with self.query_lock:
                self._send_cmd_unlocked(f"TEMPerature:OPSLave {self.press_control_slave}", source="ctrl-pstop")
                self._send_cmd_unlocked("TEMPerature:STOP", source="ctrl-pstop")

        self._run_async_io(stop_task, "Pressure Stop")
        self.append_log("P-STOP sent")

    def send_custom_cmd(self) -> None:
        cmd = self.cmd_var.get().strip()
        if not cmd:
            self.append_log("Command is empty.")
            return
        if not self.is_connected():
            self.append_log("Not connected.")
            return

        def task() -> None:
            if "?" in cmd:
                resp = self.query(cmd, max_wait_s=POLL_WAIT_S)
                self.root.after(0, lambda r=resp: self.data_var.set(r))
                self.append_log(f"{cmd} => {resp!r}")
            else:
                self.send_cmd(cmd)
                self.root.after(0, lambda: self.data_var.set("(sent)"))
                self.append_log(f"{cmd} => sent")

        self._run_async_io(task, "Send Command")


if __name__ == "__main__":
    root = tk.Tk()
    app = TempControllerApp(root)

    def on_close() -> None:
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
