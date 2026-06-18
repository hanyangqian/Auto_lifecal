# -*- coding: utf-8 -*-
"""
NPZ -> CSV converter for Tek/JAKA waveform files.

Run without arguments to open a small Windows file-dialog UI:
    python npz_to_csv_converter.py

Command line usage is still supported:
    python npz_to_csv_converter.py "D:/Data/points_id0_T0000CH1.NPZ"
    python npz_to_csv_converter.py "D:/Data" -o "D:/Data_csv" --overwrite
"""

from __future__ import annotations

import argparse
import json
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

import numpy as np
import pandas as pd


PREFERRED_COLUMNS = ["TIME", "CH1", "CH2", "CH3", "CH4"]


def load_npz_as_dataframe(npz_path: Path) -> tuple[pd.DataFrame, dict | None]:
    """
    Read one NPZ file and convert its one-dimensional arrays to a DataFrame.

    __meta_json__ is treated as metadata only and is not written to CSV.
    TIME/CH1/CH2/CH3/CH4 are written first when present; other arrays follow.
    """
    with np.load(npz_path, allow_pickle=False) as data:
        keys = list(data.files)

        meta = None
        if "__meta_json__" in keys:
            try:
                meta_text = str(data["__meta_json__"].item())
                meta = json.loads(meta_text)
            except Exception:
                meta = None

        data_keys = [key for key in keys if key != "__meta_json__"]
        ordered_keys = [key for key in PREFERRED_COLUMNS if key in data_keys]
        ordered_keys += [key for key in data_keys if key not in ordered_keys]

        if not ordered_keys:
            raise ValueError(f"{npz_path} 中没有可转换的数据数组")

        arrays = {}
        expected_len = None
        for key in ordered_keys:
            arr = np.asarray(data[key])

            if arr.ndim != 1:
                raise ValueError(f"{npz_path.name} 中的 {key} 不是一维数组，shape={arr.shape}")

            if expected_len is None:
                expected_len = len(arr)
            elif len(arr) != expected_len:
                raise ValueError(
                    f"{npz_path.name} 中列长度不一致：{key} 长度={len(arr)}, 期望={expected_len}"
                )

            arrays[key] = arr

    return pd.DataFrame(arrays), meta


def convert_one(npz_path: Path, output_dir: Path | None = None, overwrite: bool = False) -> Path:
    npz_path = npz_path.resolve()
    if not npz_path.is_file():
        raise FileNotFoundError(f"文件不存在：{npz_path}")
    if npz_path.suffix.lower() != ".npz":
        raise ValueError(f"不是 NPZ 文件：{npz_path}")

    if output_dir is None:
        csv_path = npz_path.with_suffix(".CSV")
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / (npz_path.stem + ".CSV")

    if csv_path.exists() and not overwrite:
        raise FileExistsError(f"CSV 已存在，跳过以避免覆盖：{csv_path}")

    df, meta = load_npz_as_dataframe(npz_path)
    df.to_csv(csv_path, index=False)

    print(
        f"[完成] {npz_path.name} -> {csv_path.name} | "
        f"rows={len(df)} cols={len(df.columns)}"
    )

    if meta:
        print(
            f"       meta: format={meta.get('format', 'N/A')}, "
            f"scope={meta.get('scope_profile', 'N/A')}, "
            f"channels={meta.get('channels', 'N/A')}"
        )

    return csv_path


def iter_npz_files(input_path: Path) -> list[Path]:
    input_path = input_path.resolve()
    if input_path.is_file():
        return [input_path]
    if input_path.is_dir():
        return sorted(
            path for path in input_path.iterdir()
            if path.is_file() and path.suffix.lower() == ".npz"
        )
    raise FileNotFoundError(f"路径不存在：{input_path}")


class NpzToCsvApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("NPZ 转 CSV")
        self.geometry("780x520")
        self.minsize(680, 420)

        self.selected_files: list[Path] = []
        self.selected_folder: Path | None = None
        self.output_dir: Path | None = None
        self.overwrite_var = tk.BooleanVar(value=False)
        self.input_mode_var = tk.StringVar(value="尚未选择输入")
        self.output_dir_var = tk.StringVar(value="输出到源文件所在文件夹")

        self._build_ui()

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)

        input_frame = ttk.LabelFrame(root, text="输入")
        input_frame.pack(fill=tk.X)

        button_row = ttk.Frame(input_frame)
        button_row.pack(fill=tk.X, padx=10, pady=(10, 6))

        ttk.Button(button_row, text="选择单个/多个 NPZ 文件", command=self.select_files).pack(side=tk.LEFT)
        ttk.Button(button_row, text="选择文件夹内所有 NPZ", command=self.select_folder).pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(input_frame, textvariable=self.input_mode_var).pack(
            anchor=tk.W, padx=10, pady=(0, 10)
        )

        output_frame = ttk.LabelFrame(root, text="输出")
        output_frame.pack(fill=tk.X, pady=(10, 0))

        output_row = ttk.Frame(output_frame)
        output_row.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(output_row, text="选择输出文件夹", command=self.select_output_dir).pack(side=tk.LEFT)
        ttk.Button(output_row, text="使用源文件夹", command=self.clear_output_dir).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Checkbutton(output_row, text="覆盖已存在 CSV", variable=self.overwrite_var).pack(side=tk.LEFT, padx=(16, 0))

        ttk.Label(output_frame, textvariable=self.output_dir_var).pack(
            anchor=tk.W, padx=10, pady=(0, 10)
        )

        action_row = ttk.Frame(root)
        action_row.pack(fill=tk.X, pady=(10, 0))

        ttk.Button(action_row, text="开始转换", command=self.convert_selected).pack(side=tk.LEFT)
        ttk.Button(action_row, text="清空日志", command=self.clear_log).pack(side=tk.LEFT, padx=(8, 0))

        log_frame = ttk.LabelFrame(root, text="日志")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.log_text = scrolledtext.ScrolledText(log_frame, height=14, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def select_files(self) -> None:
        file_names = filedialog.askopenfilenames(
            title="选择一个或多个 NPZ 文件",
            filetypes=[("NPZ files", "*.npz *.NPZ"), ("All files", "*.*")],
        )
        if not file_names:
            return

        self.selected_files = [Path(name) for name in file_names]
        self.selected_folder = None
        self.input_mode_var.set(f"已选择 {len(self.selected_files)} 个 NPZ 文件")
        self.log(f"[输入] 已选择文件：{len(self.selected_files)} 个")

    def select_folder(self) -> None:
        folder_name = filedialog.askdirectory(title="选择包含 NPZ 文件的文件夹")
        if not folder_name:
            return

        self.selected_folder = Path(folder_name)
        self.selected_files = []
        count = len(iter_npz_files(self.selected_folder))
        self.input_mode_var.set(f"已选择文件夹：{self.selected_folder}（{count} 个 NPZ）")
        self.log(f"[输入] 已选择文件夹：{self.selected_folder}，找到 {count} 个 NPZ")

    def select_output_dir(self) -> None:
        folder_name = filedialog.askdirectory(title="选择 CSV 输出文件夹")
        if not folder_name:
            return

        self.output_dir = Path(folder_name)
        self.output_dir_var.set(str(self.output_dir))
        self.log(f"[输出] CSV 将保存到：{self.output_dir}")

    def clear_output_dir(self) -> None:
        self.output_dir = None
        self.output_dir_var.set("输出到源文件所在文件夹")
        self.log("[输出] CSV 将保存到每个 NPZ 的源文件夹")

    def convert_selected(self) -> None:
        try:
            files = self._selected_npz_files()
        except Exception as exc:
            messagebox.showerror("输入错误", str(exc))
            return

        if not files:
            messagebox.showinfo("没有文件", "没有找到可转换的 NPZ 文件。")
            return

        ok = 0
        failed = 0
        self.log(f"[开始] 待转换 {len(files)} 个文件")

        for npz_path in files:
            try:
                csv_path = convert_one(
                    npz_path,
                    output_dir=self.output_dir,
                    overwrite=self.overwrite_var.get(),
                )
                ok += 1
                self.log(f"[完成] {npz_path.name} -> {csv_path}")
            except Exception as exc:
                failed += 1
                self.log(f"[失败] {npz_path}: {exc}")

            self.update_idletasks()

        self.log(f"[结束] 成功 {ok} 个，失败 {failed} 个")
        if failed:
            messagebox.showwarning("转换完成", f"成功 {ok} 个，失败 {failed} 个。详情见日志。")
        else:
            messagebox.showinfo("转换完成", f"成功转换 {ok} 个文件。")

    def _selected_npz_files(self) -> list[Path]:
        if self.selected_folder is not None:
            return iter_npz_files(self.selected_folder)
        if self.selected_files:
            return self.selected_files
        raise ValueError("请先选择 NPZ 文件，或选择包含 NPZ 的文件夹。")

    def log(self, message: str) -> None:
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)

    def clear_log(self) -> None:
        self.log_text.delete("1.0", tk.END)


def run_cli(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Convert waveform NPZ files to CSV.")
    parser.add_argument("input", help="NPZ 文件路径，或包含 NPZ 的文件夹路径")
    parser.add_argument("-o", "--output-dir", default="", help="CSV 输出文件夹；留空则输出到源文件夹")
    parser.add_argument("--overwrite", action="store_true", help="允许覆盖已存在的 CSV 文件")

    args = parser.parse_args(argv)

    input_path = Path(args.input)
    output_dir = Path(args.output_dir).resolve() if args.output_dir else None

    files = iter_npz_files(input_path)
    if not files:
        print(f"[提示] 没有找到 NPZ 文件：{input_path}")
        return 0

    ok = 0
    failed = 0

    for npz_path in files:
        try:
            convert_one(npz_path, output_dir=output_dir, overwrite=args.overwrite)
            ok += 1
        except Exception as exc:
            failed += 1
            print(f"[失败] {npz_path}: {exc}", file=sys.stderr)

    print(f"\n转换完成：成功 {ok} 个，失败 {failed} 个")
    return 0 if failed == 0 else 1


def main() -> int:
    if len(sys.argv) == 1:
        app = NpzToCsvApp()
        app.mainloop()
        return 0

    return run_cli(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())