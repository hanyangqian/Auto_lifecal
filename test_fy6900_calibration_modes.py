from __future__ import annotations

import argparse
import time

import serial


BAUD = 115200
COMMAND_DELAY_S = 0.2

LIFETIME_FREQ_HZ = 10.0
LIFETIME_AMPLITUDE_VPP = 5.0
LIFETIME_OFFSET_V = 2.5
LIFETIME_DUTY_PERCENT = 50.0
SPECTRUM_DC_V = 5.0


def frequency_code(hz: float) -> str:
    # This FY6900 firmware displays WMF/WFF arguments as integer Hz.
    return f"{int(round(float(hz))):014d}"


def channel_commands(
    channel: str,
    waveform: int,
    frequency_hz: float,
    amplitude_vpp: float,
    offset_v: float,
    duty_percent: float | None = None,
) -> list[str]:
    if channel == "CH1":
        prefix = {"wave": "WMW", "freq": "WMF", "amp": "WMA", "offset": "WMO", "duty": "WMD", "out": "WMN"}
    elif channel == "CH2":
        prefix = {"wave": "WFW", "freq": "WFF", "amp": "WFA", "offset": "WFO", "duty": "WFD", "out": "WFN"}
    else:
        raise ValueError(f"Unsupported channel: {channel}")

    commands = [
        f"{prefix['out']}0",
        f"{prefix['wave']}{int(waveform)}",
        f"{prefix['freq']}{frequency_code(frequency_hz)}",
        f"{prefix['amp']}{float(amplitude_vpp):.3f}",
        f"{prefix['offset']}{float(offset_v):.3f}",
    ]
    if duty_percent is not None:
        commands.append(f"{prefix['duty']}{float(duty_percent):.1f}")
    commands.append(f"{prefix['out']}1")
    return commands


def lifetime_commands(channels: list[str]) -> list[str]:
    commands = []
    for channel in channels:
        commands.extend(
            channel_commands(
                channel,
                waveform=1,
                frequency_hz=LIFETIME_FREQ_HZ,
                amplitude_vpp=LIFETIME_AMPLITUDE_VPP,
                offset_v=LIFETIME_OFFSET_V,
                duty_percent=LIFETIME_DUTY_PERCENT,
            )
        )
    return commands


def spectrum_commands(channels: list[str]) -> list[str]:
    commands = []
    for channel in channels:
        commands.extend(
            channel_commands(
                channel,
                waveform=6 if channel == "CH1" else 5,
                frequency_hz=1.0,
                amplitude_vpp=0.0,
                offset_v=SPECTRUM_DC_V,
            )
        )
    return commands


def off_commands(channels: list[str]) -> list[str]:
    commands = []
    for channel in channels:
        commands.append("WMN0" if channel == "CH1" else "WFN0")
    return commands


def readback_commands(channels: list[str]) -> list[str]:
    commands = []
    for channel in channels:
        commands.append("RMF" if channel == "CH1" else "RFF")
    return commands


def send_commands(port: str, commands: list[str], dry_run: bool, readback: list[str] | None = None) -> None:
    if dry_run:
        for command in commands:
            print(command)
        if readback:
            print("Readback commands:")
            for command in readback:
                print(command)
        return

    with serial.Serial(
        port,
        baudrate=BAUD,
        timeout=0.3,
        write_timeout=0.4,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    ) as ser:
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for command in commands:
            print(f"> {command}")
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            time.sleep(COMMAND_DELAY_S)
        for command in readback or []:
            ser.reset_input_buffer()
            print(f"> {command}")
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            time.sleep(COMMAND_DELAY_S)
            response = ser.readline().decode("ascii", errors="ignore").strip("\x00\r\n ")
            print(f"< {response}")


def parse_channels(text: str) -> list[str]:
    channels = [item.strip().upper() for item in text.split(",") if item.strip()]
    invalid = [channel for channel in channels if channel not in {"CH1", "CH2"}]
    if invalid:
        raise ValueError(f"Invalid channels: {', '.join(invalid)}")
    return channels or ["CH1"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Test FY6900 calibration light-source modes.")
    parser.add_argument("--port", default="COM6", help="FY6900 serial port, for example COM6.")
    parser.add_argument("--channels", default="CH1", help="Comma-separated channels: CH1, CH2, or CH1,CH2.")
    parser.add_argument(
        "--mode",
        choices=["lifetime", "spectrum", "joint", "off"],
        default="joint",
        help="Mode to test.",
    )
    parser.add_argument("--hold-s", type=float, default=5.0, help="Wait time between lifetime and spectrum in joint mode.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without opening the serial port.")
    parser.add_argument("--no-readback", action="store_true", help="Do not query RMF/RFF after setting.")
    args = parser.parse_args()

    channels = parse_channels(args.channels)
    readback = None if args.no_readback else readback_commands(channels)
    if args.mode == "lifetime":
        commands = lifetime_commands(channels)
        print("Setting lifetime mode: square 10 Hz, 5 Vpp, offset 2.5 V")
        send_commands(args.port, commands, args.dry_run, readback)
    elif args.mode == "spectrum":
        commands = spectrum_commands(channels)
        print("Setting spectrum mode: DC 5 V")
        send_commands(args.port, commands, args.dry_run, readback)
    elif args.mode == "off":
        commands = off_commands(channels)
        print("Turning output off")
        send_commands(args.port, commands, args.dry_run, None)
    else:
        print("Joint test: lifetime mode first")
        send_commands(args.port, lifetime_commands(channels), args.dry_run, readback)
        print(f"Waiting {args.hold_s:g} s before switching to spectrum mode")
        if not args.dry_run:
            time.sleep(max(0.0, args.hold_s))
        print("Switching to spectrum mode")
        send_commands(args.port, spectrum_commands(channels), args.dry_run, readback)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
