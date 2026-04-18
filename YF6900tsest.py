import tkinter as tk
from tkinter import ttk
import serial
import time

PORT = "COM5"
BAUD = 115200

class FY6900Controller:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        time.sleep(0.2)

    def query(self, cmd):
        self.ser.reset_input_buffer()
        self.send(cmd)
        return self.ser.readline().decode().strip()

    def ch1_on(self):
        self.send("WMN1")

    def ch1_off(self):
        self.send("WMN0")

    def ch2_on(self):
        self.send("WFN1")

    def ch2_off(self):
        self.send("WFN0")

    def read_status(self):
        ch1 = self.query("RMN")
        ch2 = self.query("RFN")
        return ch1, ch2


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("FY6900 控制面板")

        self.ctrl = FY6900Controller(PORT, BAUD)

        # 选择通道
        self.ch1_var = tk.BooleanVar()
        self.ch2_var = tk.BooleanVar()

        ttk.Checkbutton(root, text="CH1", variable=self.ch1_var).grid(row=0, column=0, padx=10, pady=10)
        ttk.Checkbutton(root, text="CH2", variable=self.ch2_var).grid(row=0, column=1, padx=10, pady=10)

        # 按钮
        ttk.Button(root, text="输出 ON", command=self.output_on).grid(row=1, column=0, padx=10, pady=10)
        ttk.Button(root, text="输出 OFF", command=self.output_off).grid(row=1, column=1, padx=10, pady=10)

        # 状态显示
        self.status_label = ttk.Label(root, text="状态：未知")
        self.status_label.grid(row=2, column=0, columnspan=2, pady=10)

        ttk.Button(root, text="刷新状态", command=self.update_status).grid(row=3, column=0, columnspan=2, pady=5)

    def output_on(self):
        if self.ch1_var.get():
            self.ctrl.ch1_on()
        if self.ch2_var.get():
            self.ctrl.ch2_on()
        self.update_status()

    def output_off(self):
        if self.ch1_var.get():
            self.ctrl.ch1_off()
        if self.ch2_var.get():
            self.ctrl.ch2_off()
        self.update_status()

    def update_status(self):
        ch1, ch2 = self.ctrl.read_status()

        def parse(val):
            if val in ("255", "0000000255"):
                return "ON"
            elif val in ("0", "0000000000"):
                return "OFF"
            else:
                return f"?({val})"

        text = f"CH1: {parse(ch1)} | CH2: {parse(ch2)}"
        self.status_label.config(text=text)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()