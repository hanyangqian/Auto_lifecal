# -*- coding: utf-8 -*-
"""
Small USB/VISA discovery script for checking whether a USB spectrometer is visible.

Run:
    python test.py

Optional SCPI identity query for VISA USB instruments:
    python test.py --idn
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from typing import Any


OCEAN_KEYWORDS = ("ocean", "spectrometer", "seabreeze", "spectra", "usb2000", "usb4000")


def print_section(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def looks_relevant(text: str) -> bool:
    lower = text.lower()
    return any(keyword in lower for keyword in OCEAN_KEYWORDS)


def list_visa_resources(query_idn: bool) -> None:
    print_section("PyVISA resources")
    try:
        import pyvisa
    except ImportError as exc:
        print(f"PyVISA is not installed: {exc}")
        return

    try:
        rm = pyvisa.ResourceManager()
        print(f"VISA backend: {rm.visalib}")
        resources = list(rm.list_resources())
    except Exception as exc:
        print(f"Unable to initialize/list VISA resources: {exc}")
        return

    if not resources:
        print("No VISA resources found.")
        return

    for resource in resources:
        marker = "  <-- possible spectrometer" if looks_relevant(resource) else ""
        print(f"- {resource}{marker}")

        if query_idn and resource.upper().startswith("USB") and "::INSTR" in resource.upper():
            try:
                instrument = rm.open_resource(resource)
                instrument.timeout = 1000
                try:
                    idn = instrument.query("*IDN?").strip()
                    print(f"  *IDN?: {idn}")
                finally:
                    instrument.close()
            except Exception as exc:
                print(f"  *IDN? failed: {exc}")


def list_serial_ports() -> None:
    print_section("Serial ports")
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        print(f"pyserial is not installed: {exc}")
        return

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        fields = [
            port.device or "",
            port.description or "",
            f"VID:PID={port.vid:04X}:{port.pid:04X}" if port.vid is not None and port.pid is not None else "",
            f"SN={port.serial_number}" if port.serial_number else "",
        ]
        line = " | ".join(field for field in fields if field)
        marker = "  <-- possible spectrometer" if looks_relevant(line) else ""
        print(f"- {line}{marker}")


def run_powershell_json(command: str) -> list[dict[str, Any]]:
    powershell = shutil.which("powershell") or shutil.which("pwsh")
    if not powershell:
        raise RuntimeError("PowerShell was not found.")

    completed = subprocess.run(
        [powershell, "-NoProfile", "-Command", command],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())

    output = completed.stdout.strip()
    if not output:
        return []

    data = json.loads(output)
    if isinstance(data, dict):
        return [data]
    return data


def list_windows_usb_devices() -> None:
    print_section("Windows USB/PnP devices")
    if not sys.platform.startswith("win"):
        print("Skipped: this check is Windows-only.")
        return

    command = r"""
$devices = Get-CimInstance Win32_PnPEntity |
    Where-Object {
        $_.PNPDeviceID -like 'USB*' -or
        $_.Name -match 'Ocean|Spectrometer|SeaBreeze|Spectra|USB2000|USB4000'
    } |
    Select-Object Name, Manufacturer, Status, PNPDeviceID
$devices | ConvertTo-Json -Depth 3
"""
    try:
        devices = run_powershell_json(command)
    except Exception as exc:
        print(f"Unable to read Windows USB/PnP devices: {exc}")
        return

    if not devices:
        print("No USB/PnP devices found.")
        return

    for device in devices:
        name = str(device.get("Name") or "")
        manufacturer = str(device.get("Manufacturer") or "")
        status = str(device.get("Status") or "")
        pnp_id = str(device.get("PNPDeviceID") or "")
        line = f"{name} | {manufacturer} | {status} | {pnp_id}"
        marker = "  <-- possible spectrometer" if looks_relevant(line) else ""
        print(f"- {line}{marker}")


def main() -> int:
    parser = argparse.ArgumentParser(description="List USB/VISA resources visible to Python.")
    parser.add_argument(
        "--idn",
        action="store_true",
        help="Try *IDN? on USB VISA INSTR resources. Some non-SCPI devices may timeout.",
    )
    args = parser.parse_args()

    list_visa_resources(args.idn)
    list_serial_ports()
    list_windows_usb_devices()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
