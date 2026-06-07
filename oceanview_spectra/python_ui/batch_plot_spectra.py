from pathlib import Path
from decimal import Decimal, ROUND_HALF_UP
import re
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


# =========================
# User settings
# =========================
SCRIPT_DIR = Path(__file__).resolve().parent

# Set INPUT_DIR to the folder containing OceanView .txt spectra files.
# If INPUT_FILES is not empty, only those files will be processed.
INPUT_DIR = SCRIPT_DIR
INPUT_FILES: List[Path] = []
FILE_PATTERN = "*.txt"

DATA_START_MARKER = ">>>>>Begin Spectral Data<<<<<"
FILE_ENCODING = "gb18030"
WAVELENGTH_QUANTUM = Decimal("0.001")
WAVELENGTH_TOLERANCE = 0.0015
BACKGROUND_PREFIX = "back"
MAIN_WINDOW_SPECTRUM_PREFIX_RE = re.compile(
    r"^(?P<temperature>[+-]?\d+(?:\.\d+)?)C_(?P<pressure>(?:[+-]?\d+(?:\.\d+)?|current))kPa(?:_|$)",
    re.IGNORECASE,
)

# Keep this empty to process BACKGROUND_PREFIX plus numeric temperature prefixes.
# Example: ALLOWED_SAMPLE_PREFIXES = ["25", "50", "75"]
ALLOWED_SAMPLE_PREFIXES: List[str] = []
REQUIRE_NUMERIC_TEMPERATURE_PREFIX = True

OUTPUT_DIR = INPUT_DIR
OUTPUT_NAME = "temperature_spectra.png"

X_MIN = 500
X_MAX = 900
FIGURE_WIDTH = 6.0
FIGURE_HEIGHT = 4.5
LINE_WIDTH = 0.8

MATERIAL_LABEL = r"Al$_2$O$_3$:Cr" + "\n" + "Ex=395 nm"
MATERIAL_LABEL_X = 0.03
MATERIAL_LABEL_Y = 0.95
MATERIAL_LABEL_FONT_SIZE = 12

SHOW_LEGEND = True
LEGEND_LOCATION = "upper right"
LEGEND_FONT_SIZE = 9
LEGEND_HANDLE_LENGTH = 2.4
LEGEND_HANDLE_TEXT_PAD = 0.6
LEGEND_BORDER_PAD = 0.2
LEGEND_FRAME_ON = False
LEGEND_LABEL_TEMPLATE = r"{temperature} $^\circ$C"

X_LABEL = "Wavelength (nm)"
Y_LABEL = "Counts"
AXIS_LABEL_FONT_SIZE = 12
TICK_LABEL_SIZE = 10

SAVE_DPI = 300

COLORS = [
    "#1A1A1A",
    "#1F77B4",
    "#D62728",
    "#2CA02C",
    "#9467BD",
    "#FF7F0E",
    "#17BECF",
    "#8C564B",
    "#E377C2",
    "#BCBD22",
]


# =========================
# Processing logic
# =========================
def resolve_path(path: Path) -> Path:
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


def collect_input_files() -> List[Path]:
    if INPUT_FILES:
        return sorted(resolve_path(file_path) for file_path in INPUT_FILES)

    input_dir = resolve_path(INPUT_DIR)
    return sorted(input_dir.glob(FILE_PATTERN))


def read_spectrum(file_path: Path) -> Tuple[List[float], List[float]]:
    wavelengths: List[float] = []
    counts: List[float] = []
    in_data_section = False

    with file_path.open("r", encoding=FILE_ENCODING) as file:
        for line in file:
            text = line.strip()
            if not in_data_section:
                if text == DATA_START_MARKER:
                    in_data_section = True
                continue

            if not text:
                continue

            parts = text.split()
            if len(parts) < 2:
                continue

            wavelengths.append(float(Decimal(parts[0]).quantize(WAVELENGTH_QUANTUM, rounding=ROUND_HALF_UP)))
            counts.append(float(parts[1]))

    if not wavelengths:
        raise ValueError(f"No spectral data found in {file_path.name}")

    return wavelengths, counts


def average_spectra(file_paths: List[Path]) -> Tuple[List[float], List[float]]:
    wavelengths_ref: List[float] = []
    sum_counts: List[float] = []

    for index, file_path in enumerate(sorted(file_paths)):
        wavelengths, counts = read_spectrum(file_path)

        if index == 0:
            wavelengths_ref = wavelengths
            sum_counts = counts[:]
            continue

        if len(wavelengths) != len(wavelengths_ref):
            raise ValueError(f"Point count mismatch: {file_path.name}")

        for point_index, wavelength in enumerate(wavelengths):
            if abs(wavelength - wavelengths_ref[point_index]) > WAVELENGTH_TOLERANCE:
                raise ValueError(f"Wavelength axis mismatch: {file_path.name}")
            sum_counts[point_index] += counts[point_index]

    averaged_counts = [value / len(file_paths) for value in sum_counts]
    return wavelengths_ref, averaged_counts


def spectrum_group_key(file_path: Path) -> str:
    stem = file_path.stem
    if stem == BACKGROUND_PREFIX or stem.startswith(f"{BACKGROUND_PREFIX}_"):
        return BACKGROUND_PREFIX

    main_window_match = MAIN_WINDOW_SPECTRUM_PREFIX_RE.match(stem)
    if main_window_match:
        return main_window_match.group("temperature")

    return stem.split("_", 1)[0]


def collect_groups(txt_files: List[Path]) -> Dict[str, List[Path]]:
    groups: Dict[str, List[Path]] = {}
    for file_path in txt_files:
        prefix = spectrum_group_key(file_path)
        groups.setdefault(prefix, []).append(file_path)
    return groups


def is_allowed_sample_prefix(prefix: str) -> bool:
    if prefix == BACKGROUND_PREFIX:
        return True
    if ALLOWED_SAMPLE_PREFIXES:
        return prefix in ALLOWED_SAMPLE_PREFIXES
    if REQUIRE_NUMERIC_TEMPERATURE_PREFIX:
        try:
            float(prefix)
        except ValueError:
            return False
    return True


def filter_groups(groups: Dict[str, List[Path]]) -> Dict[str, List[Path]]:
    filtered: Dict[str, List[Path]] = {}
    skipped: Dict[str, List[Path]] = {}

    for prefix, file_paths in groups.items():
        if is_allowed_sample_prefix(prefix):
            filtered[prefix] = file_paths
        else:
            skipped[prefix] = file_paths

    for prefix, file_paths in sorted(skipped.items()):
        names = ", ".join(file_path.name for file_path in file_paths)
        print(f"Skipped prefix '{prefix}': {names}")

    return filtered


def subtract_background(sample_counts: List[float], background_counts: List[float]) -> List[float]:
    if len(sample_counts) != len(background_counts):
        raise ValueError("Sample spectrum and background spectrum have different point counts.")
    return [sample - background for sample, background in zip(sample_counts, background_counts)]


def configure_plot_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "mathtext.fontset": "dejavuserif",
            "axes.linewidth": 1.0,
            "axes.unicode_minus": False,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.major.size": 4,
            "ytick.major.size": 4,
            "xtick.major.width": 0.9,
            "ytick.major.width": 0.9,
            "savefig.facecolor": "white",
            "figure.facecolor": "white",
        }
    )


def plot_temperature_spectra(
    wavelengths: List[float],
    temperature_groups: Dict[str, List[float]],
    output_path: Path,
) -> None:
    configure_plot_style()
    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH, FIGURE_HEIGHT), dpi=300)

    ordered_temperatures = sorted(temperature_groups.keys(), key=lambda item: float(item))
    for index, temperature in enumerate(ordered_temperatures):
        ax.plot(
            wavelengths,
            temperature_groups[temperature],
            color=COLORS[index % len(COLORS)],
            linestyle="-",
            linewidth=LINE_WIDTH,
            label=LEGEND_LABEL_TEMPLATE.format(temperature=temperature),
        )

    ax.set_xlabel(X_LABEL, fontsize=AXIS_LABEL_FONT_SIZE)
    ax.set_ylabel(Y_LABEL, fontsize=AXIS_LABEL_FONT_SIZE)
    ax.set_xlim(X_MIN, X_MAX)
    ax.tick_params(top=True, right=True, labelsize=TICK_LABEL_SIZE)
    ax.text(
        MATERIAL_LABEL_X,
        MATERIAL_LABEL_Y,
        MATERIAL_LABEL,
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=MATERIAL_LABEL_FONT_SIZE,
    )
    if SHOW_LEGEND:
        ax.legend(
            loc=LEGEND_LOCATION,
            frameon=LEGEND_FRAME_ON,
            fontsize=LEGEND_FONT_SIZE,
            handlelength=LEGEND_HANDLE_LENGTH,
            handletextpad=LEGEND_HANDLE_TEXT_PAD,
            borderpad=LEGEND_BORDER_PAD,
        )

    fig.tight_layout()
    fig.savefig(output_path, dpi=SAVE_DPI, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    txt_files = collect_input_files()
    if not txt_files:
        raise FileNotFoundError("No spectral data files found. Check INPUT_DIR, INPUT_FILES, and FILE_PATTERN.")

    groups = filter_groups(collect_groups(txt_files))
    if BACKGROUND_PREFIX not in groups:
        raise FileNotFoundError("No background files with prefix 'back' were found.")

    background_wavelengths, background_counts = average_spectra(groups[BACKGROUND_PREFIX])
    temperature_groups: Dict[str, List[float]] = {}

    for prefix, file_paths in groups.items():
        if prefix == BACKGROUND_PREFIX:
            continue

        wavelengths, counts = average_spectra(file_paths)
        if len(wavelengths) != len(background_wavelengths):
            raise ValueError(f"Point count mismatch between background and {prefix}.")

        for point_index, wavelength in enumerate(wavelengths):
            if abs(wavelength - background_wavelengths[point_index]) > WAVELENGTH_TOLERANCE:
                raise ValueError(f"Wavelength axis mismatch between background and {prefix}.")

        temperature_groups[prefix] = subtract_background(counts, background_counts)

    if not temperature_groups:
        raise ValueError("No temperature spectra available for plotting.")

    output_dir = resolve_path(OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / OUTPUT_NAME
    plot_temperature_spectra(background_wavelengths, temperature_groups, output_path)
    print(f"Processed {len(txt_files)} files from: {txt_files[0].parent}")
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
