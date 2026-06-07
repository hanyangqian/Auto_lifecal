# -*- coding: utf-8 -*-
"""
Capture spectra from an Ocean Optics/Ocean Insight spectrometer and save CSV files.

Install dependency first if needed:
    pip install seabreeze
    seabreeze_os_setup

Examples:
    python oceanfx_capture.py --list
    python oceanfx_capture.py --integration-ms 20 --output oceanview_spectra\python
    python oceanfx_capture.py --count 10 --interval 1 --integration-ms 50
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path


def import_seabreeze(backend: str | None):
    try:
        if backend:
            import seabreeze

            seabreeze.use(backend)

        import seabreeze.spectrometers as sb
    except ImportError as exc:
        print("Missing dependency: seabreeze")
        print()
        print("Install it with:")
        print("    pip install seabreeze")
        print("    seabreeze_os_setup")
        print()
        print("If the device is still not found on Windows, try:")
        print("    pip install seabreeze[pyseabreeze]")
        print("    python oceanfx_capture.py --backend pyseabreeze --list")
        print()
        print(f"Import error: {exc}")
        raise SystemExit(2) from exc

    return sb


def list_devices(sb) -> int:
    devices = sb.list_devices()
    if not devices:
        print("No spectrometers found by seabreeze.")
        return 1

    print("Found spectrometers:")
    for index, device in enumerate(devices):
        print(f"{index}: {device}")
    return 0


def open_spectrometer(sb, serial_number: str | None):
    if serial_number:
        return sb.Spectrometer.from_serial_number(serial_number)
    return sb.Spectrometer.from_first_available()


def save_spectrum_csv(path: Path, wavelengths, intensities, metadata: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        for key, value in metadata.items():
            writer.writerow([f"# {key}", value])
        writer.writerow(["wavelength_nm", "intensity_counts"])
        writer.writerows(zip(wavelengths, intensities))


def averaged_spectrum(spectrometer, scans_to_average: int):
    wavelengths, intensities = spectrometer.spectrum()
    if scans_to_average <= 1:
        return wavelengths, intensities

    summed = [float(value) for value in intensities]
    for _ in range(scans_to_average - 1):
        _, next_intensities = spectrometer.spectrum()
        for index, value in enumerate(next_intensities):
            summed[index] += float(value)

    averaged = [value / scans_to_average for value in summed]
    return wavelengths, averaged


def timestamp_name(prefix: str, index: int, total: int) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    if total == 1:
        return f"{prefix}_{stamp}.csv"
    return f"{prefix}_{stamp}_{index:04d}.csv"


def capture(args) -> int:
    sb = import_seabreeze(args.backend)

    if args.list:
        return list_devices(sb)

    output_dir = args.output.expanduser().resolve()
    integration_us = int(args.integration_ms * 1000)

    try:
        spectrometer = open_spectrometer(sb, args.serial)
    except Exception as exc:
        print(f"Unable to open spectrometer: {exc}")
        print("Try: python oceanfx_capture.py --list")
        return 1

    try:
        model = getattr(spectrometer, "model", "unknown")
        serial = getattr(spectrometer, "serial_number", "unknown")
        print(f"Opened spectrometer: model={model}, serial={serial}")

        try:
            print(f"Integration limits (us): {spectrometer.integration_time_micros_limits}")
        except Exception:
            pass

        spectrometer.integration_time_micros(integration_us)
        print(f"Integration time: {integration_us} us")
        print(f"Scans to average: {args.average}")
        print(f"Output folder: {output_dir}")

        for index in range(1, args.count + 1):
            wavelengths, intensities = averaged_spectrum(spectrometer, args.average)
            file_path = output_dir / timestamp_name(args.prefix, index, args.count)
            metadata = {
                "created_at": datetime.now().isoformat(timespec="milliseconds"),
                "model": str(model),
                "serial_number": str(serial),
                "integration_time_us": str(integration_us),
                "scans_to_average": str(args.average),
            }
            save_spectrum_csv(file_path, wavelengths, intensities, metadata)
            print(f"Saved {file_path}")

            if index < args.count:
                time.sleep(max(args.interval, 0.0))
    finally:
        try:
            spectrometer.close()
        except Exception:
            pass

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Directly capture and save spectra from Ocean spectrometers.")
    parser.add_argument("--list", action="store_true", help="List spectrometers and exit.")
    parser.add_argument("--backend", choices=["cseabreeze", "pyseabreeze"], help="Force a seabreeze backend.")
    parser.add_argument("--serial", help="Open a specific spectrometer serial number.")
    parser.add_argument("--integration-ms", type=float, default=20.0, help="Integration time in milliseconds.")
    parser.add_argument("--average", type=int, default=1, help="Number of scans to average per saved spectrum.")
    parser.add_argument("--count", type=int, default=1, help="Number of spectra to save.")
    parser.add_argument("--interval", type=float, default=1.0, help="Delay between captures in seconds.")
    parser.add_argument("--output", type=Path, default=Path("oceanview_spectra") / "python")
    parser.add_argument("--prefix", default="spectrum", help="Output CSV filename prefix.")
    args = parser.parse_args()

    if args.count < 1:
        print("--count must be at least 1.")
        return 2
    if args.integration_ms <= 0:
        print("--integration-ms must be greater than 0.")
        return 2
    if args.average < 1:
        print("--average must be at least 1.")
        return 2

    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
