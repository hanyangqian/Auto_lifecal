# -*- coding: utf-8 -*-
"""
Helper for setting an OceanView save folder and watching saved spectra files.

This script does not change OceanView internals. It creates a target folder,
copies the folder path to the clipboard when possible, opens the folder, and
can watch for newly saved spectra files.

Run:
    python oceanview_save_helper.py
    python oceanview_save_helper.py --watch
    python oceanview_save_helper.py --path D:\OceanViewData\test1 --watch
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path


SPECTRUM_EXTENSIONS = {".txt", ".tsv", ".csv", ".dat"}


def default_save_path() -> Path:
    today = datetime.now().strftime("%Y%m%d")
    return Path.cwd() / "oceanview_spectra" / today


def copy_to_clipboard(text: str) -> bool:
    if not sys.platform.startswith("win"):
        return False

    try:
        import subprocess

        completed = subprocess.run(
            ["clip"],
            input=text,
            text=True,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return completed.returncode == 0
    except Exception:
        return False


def open_folder(path: Path) -> None:
    if sys.platform.startswith("win"):
        os.startfile(str(path))  # noqa: S606 - local helper for opening Explorer
        return

    print(f"Folder ready: {path}")


def write_readme(path: Path) -> None:
    readme = path / "OceanView保存说明.txt"
    if readme.exists():
        return

    readme.write_text(
        "OceanView 保存光谱步骤：\n"
        "1. 在 OceanView 图谱窗口顶部点击齿轮/保存参数按钮。\n"
        "2. 在 File Options 里把 Target Directory 设置为本文件夹。\n"
        "3. 根据需要设置 Basename、File suffix、File Format。\n"
        "4. 在 Save Options 里选择保存频率，例如 Save every scan 或 Stop after this many scans。\n"
        "5. 回到图谱窗口点击保存按钮，OceanView 会把光谱文件写入这里。\n",
        encoding="utf-8",
    )


def snapshot_files(path: Path) -> dict[Path, float]:
    files: dict[Path, float] = {}
    for item in path.iterdir():
        if item.is_file() and item.suffix.lower() in SPECTRUM_EXTENSIONS:
            files[item] = item.stat().st_mtime
    return files


def watch_folder(path: Path, interval_s: float) -> None:
    print()
    print(f"Watching for OceanView spectrum files in: {path}")
    print("Press Ctrl+C to stop.")

    known = snapshot_files(path)
    try:
        while True:
            time.sleep(interval_s)
            current = snapshot_files(path)
            for file_path, modified_time in sorted(current.items()):
                if file_path not in known or known[file_path] != modified_time:
                    size = file_path.stat().st_size
                    timestamp = datetime.fromtimestamp(modified_time).strftime("%H:%M:%S")
                    print(f"[{timestamp}] saved/updated: {file_path.name} ({size} bytes)")
            known = current
    except KeyboardInterrupt:
        print("\nStopped watching.")


def print_oceanview_steps(path: Path) -> None:
    print("=" * 78)
    print("OceanView save folder is ready")
    print("=" * 78)
    print(path)
    print()
    print("In OceanView:")
    print("1. Open the graph view for the live spectrum.")
    print("2. Click the gear/save-parameters button at the top of the graph.")
    print("3. Set File Options -> Target Directory to the folder above.")
    print("4. Set Basename/File suffix/File Format as needed.")
    print("5. Set Save Options, then click the save button in OceanView.")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare and watch an OceanView spectrum save folder.")
    parser.add_argument(
        "--path",
        type=Path,
        default=default_save_path(),
        help="Target directory to use in OceanView.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Watch the target directory for newly saved spectrum files.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Watch interval in seconds.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Do not open the target folder in Explorer.",
    )
    args = parser.parse_args()

    save_path = args.path.expanduser().resolve()
    save_path.mkdir(parents=True, exist_ok=True)
    write_readme(save_path)

    copied = copy_to_clipboard(str(save_path))
    print_oceanview_steps(save_path)
    if copied:
        print("The folder path has been copied to the clipboard.")
    else:
        print("Copy the folder path above into OceanView's Target Directory field.")

    if not args.no_open:
        open_folder(save_path)

    if args.watch:
        watch_folder(save_path, max(args.interval, 0.2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
