#!/usr/bin/env python3
"""Plot desired/measured base-frame x position and z force from FP CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


def load_csv(path: Path) -> dict[str, list[float]]:
    columns = ("time_s", "pdes_x", "pcur_x", "fdes_z", "fcur_z")
    data: dict[str, list[float]] = {name: [] for name in columns}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = set(columns) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing FP columns: {', '.join(sorted(missing))}")
        if "ffiltered_z" in reader.fieldnames:
            columns += ("ffiltered_z",)
            data["ffiltered_z"] = []
        for line, row in enumerate(reader, start=2):
            try:
                values = [float(row[name]) for name in columns]
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid or missing number on CSV line {line}") from exc
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"Nonfinite value on CSV line {line}")
            if data["time_s"] and values[0] < data["time_s"][-1]:
                raise ValueError(f"Timestamp decreases on CSV line {line}")
            for name, value in zip(columns, values):
                data[name].append(value)
    if not data["time_s"]:
        raise ValueError("CSV has no data rows")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Path to hybrid_fp_ft_datalog_NNNN.csv")
    parser.add_argument("--save", type=Path, help="Save figure (defaults to PNG if no extension)")
    parser.add_argument("--no-show", action="store_true", help="Save without opening a window; requires --save")
    args = parser.parse_args()
    if args.no_show and args.save is None:
        parser.error("--no-show requires --save")
    try:
        data = load_csv(args.csv)
    except (ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("error: matplotlib is required (python3 -m pip install matplotlib)", file=sys.stderr)
        return 1

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    for ax, desired, measured, label, title in (
        (axes[0], "pdes_x", "pcur_x", "Position (m)", "Base-frame position — X"),
        (axes[1], "fdes_z", "fcur_z", "Force (N)", "Base-frame force — Z (robot on environment)"),
    ):
        ax.plot(data["time_s"], data[desired], "--", label="Desired", linewidth=1.8)
        ax.plot(data["time_s"], data[measured], label="Measured", linewidth=1.0)
        if measured == "fcur_z" and "ffiltered_z" in data:
            ax.plot(data["time_s"], data["ffiltered_z"], label="Filtered (before clamp)", linewidth=1.2)
        ax.set_ylabel(label)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()
    axes[1].set_xlabel("Elapsed time (s)")
    fig.suptitle(args.csv.name)
    if args.save:
        output = args.save if args.save.suffix else args.save.with_suffix(".png")
        try:
            fig.savefig(output, dpi=150, bbox_inches="tight")
        except (OSError, ValueError) as exc:
            plt.close(fig)
            print(f"error: {exc}", file=sys.stderr)
            return 1
        print(f"Wrote {output}")
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
