# tek_screenshot_lan.py
import pyvisa

SCOPE_IP = "10.5.5.88"
RESOURCE = f"TCPIP0::{SCOPE_IP}::inst0::INSTR"
OUT_FILE = "tek_screenshot.png"

def extract_scpi_block(raw: bytes) -> bytes:
    # Tek 可能返回 IEEE block: #<位数><长度><PNG数据>
    if raw.startswith(b"#") and len(raw) >= 2 and raw[1:2].isdigit():
        n_digits = int(raw[1:2])
        if n_digits > 0:
            start = 2 + n_digits
            size = int(raw[2:start])
            return raw[start:start + size]
        return raw[2:].rstrip(b"\r\n")

    # 有些情况下前面可能混入少量头部，直接找 PNG 头
    png_start = raw.find(b"\x89PNG\r\n\x1a\n")
    if png_start >= 0:
        return raw[png_start:].rstrip(b"\r\n")

    return raw.rstrip(b"\r\n")

rm = pyvisa.ResourceManager()
scope = rm.open_resource(RESOURCE)

try:
    scope.timeout = 20000
    scope.read_termination = "\n"
    scope.write_termination = "\n"

    print(scope.query("*IDN?").strip())

    scope.write("SAVe:IMAGe:FILEFormat PNG")

    # 读二进制前关闭 read_termination
    scope.read_termination = None
    scope.write("HARDCopy START")
    raw = scope.read_raw()

    png = extract_scpi_block(raw)
    if not png.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError(f"返回数据不是 PNG，前 32 字节: {png[:32]!r}")

    with open(OUT_FILE, "wb") as f:
        f.write(png)

    print(f"saved: {OUT_FILE}, {len(png)} bytes")

finally:
    try:
        scope.close()
    finally:
        rm.close()