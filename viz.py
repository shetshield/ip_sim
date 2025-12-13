from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Iterable, List

from PIL import Image

ITER_PATTERN = re.compile(r"best_iter(\d+)_score.*\\.png$", re.IGNORECASE)


def _find_layout0(output_dir: Path) -> Path:
    layout_candidates = sorted(output_dir.glob("layout0*.png"))
    if not layout_candidates:
        raise FileNotFoundError(f"No 'layout0*.png' files found in {output_dir}.")
    return layout_candidates[0]


def _find_iteration_frames(output_dir: Path) -> List[Path]:
    matches = []
    for path in output_dir.glob("*.png"):
        if path.name.lower().startswith("layout0"):
            continue
        match = ITER_PATTERN.fullmatch(path.name)
        if match:
            matches.append((int(match.group(1)), path))
    if not matches:
        raise FileNotFoundError(
            "No iteration frames found matching 'best_iter*' pattern in the output directory."
        )
    matches.sort(key=lambda item: (item[0], item[1].name))
    return [path for _, path in matches]


def collect_frame_paths(output_dir: Path) -> List[Path]:
    """Collect PNGs in the order required for GIF creation.

    The sequence always begins with a file named ``layout0*.png`` followed by PNG
    files that match ``best_iter<digits>_score*.png`` ordered by the iteration
    number extracted from the filename.
    """

    if not output_dir.is_dir():
        raise FileNotFoundError(f"Output directory does not exist: {output_dir}")

    layout0 = _find_layout0(output_dir)
    iteration_frames = _find_iteration_frames(output_dir)

    return [layout0, *iteration_frames]


def create_gif(png_paths: Iterable[Path], output_path: Path, duration: float = 0.5) -> Path:
    """Create a GIF from the given PNG paths.

    Args:
        png_paths: Ordered collection of PNG paths.
        output_path: Destination GIF path.
        duration: Frame duration in seconds.
    """

    frames = []
    for path in png_paths:
        with Image.open(path) as image:
            frames.append(image.convert("RGB"))

    if not frames:
        raise ValueError("No PNG paths provided to create GIF.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(duration * 1000),
        loop=0,
    )
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a GIF from PNGs in /output.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output"),
        help="Directory containing PNG files. Defaults to /output.",
    )
    parser.add_argument(
        "--gif-name",
        dest="gif_name",
        default="sequence.gif",
        help="Name of the resulting GIF file (default: sequence.gif).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.5,
        help="Frame duration in seconds (default: 0.5).",
    )

    args = parser.parse_args()

    frame_paths = collect_frame_paths(args.output_dir)
    gif_path = args.output_dir / args.gif_name
    create_gif(frame_paths, gif_path, duration=args.duration)
    print(f"GIF saved to {gif_path}")


if __name__ == "__main__":
    main()
