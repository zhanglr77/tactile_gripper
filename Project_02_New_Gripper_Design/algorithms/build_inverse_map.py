#!/usr/bin/env python3
"""Build a loading inverse map ``(force, dforce_dt) -> torque`` from ramp CSVs.

The output is a self-contained ``inverse_map.npz`` with ``force_grid``,
``rate_grid`` and ``torque_grid`` (plus provenance metadata).  No hardware is
required and SciPy is optional.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def _read(path, window, loading_only=False):
    rows = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if loading_only and not row.get("ramp_name", "").lower().startswith("loading"):
                continue
            try:
                t, force, tau = float(row["timestamp"]), float(row["force"]), float(row["tau_command"])
            except (KeyError, TypeError, ValueError):
                continue
            if np.isfinite(t) and np.isfinite(force) and np.isfinite(tau):
                rows.append((t, force, tau))
    if len(rows) < 3:
        return np.empty((0, 3))
    a = np.asarray(rows); order = np.argsort(a[:, 0]); a = a[order]
    keep = np.r_[True, np.diff(a[:, 0]) > 0]; a = a[keep]
    rate = np.gradient(a[:, 1], a[:, 0])
    dt = np.median(np.diff(a[:, 0])); n = max(3, int(round(window / max(dt, 1e-6))))
    if n % 2 == 0: n += 1
    rate = np.convolve(rate, np.ones(n) / n, mode="same") if n < len(rate) else rate
    return np.column_stack((a[:, 1], rate, a[:, 2]))


def build(files, force_min, force_max, rate_min, rate_max, force_bins, rate_bins, window, loading_only=False):
    chunks = [_read(p, window, loading_only) for p in files]
    data = np.vstack([x for x in chunks if len(x)]) if any(len(x) for x in chunks) else np.empty((0, 3))
    mask = ((data[:, 0] >= force_min) & (data[:, 0] <= force_max) &
            (data[:, 1] >= rate_min) & (data[:, 1] <= rate_max))
    data = data[mask]
    if len(data) < 3: raise ValueError("not enough valid calibration samples after filtering")
    fg = np.linspace(force_min, force_max, force_bins); rg = np.linspace(rate_min, rate_max, rate_bins)
    grid = np.full((force_bins, rate_bins), np.nan)
    fi = np.clip(np.searchsorted(fg, data[:, 0]), 1, force_bins - 1); fi -= (data[:, 0] - fg[fi-1] < fg[fi] - data[:, 0])
    ri = np.clip(np.searchsorted(rg, data[:, 1]), 1, rate_bins - 1); ri -= (data[:, 1] - rg[ri-1] < rg[ri] - data[:, 1])
    for i in range(force_bins):
        for j in range(rate_bins):
            v = data[(fi == i) & (ri == j), 2]
            if len(v): grid[i, j] = np.median(v)
    known = np.argwhere(np.isfinite(grid))
    for i, j in np.argwhere(~np.isfinite(grid)):
        k = np.argmin((known[:, 0] - i) ** 2 + (known[:, 1] - j) ** 2); grid[i, j] = grid[tuple(known[k])]
    return fg, rg, grid, len(data)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input-dir", type=Path, default=None)
    p.add_argument("--input", type=Path, action="append", dest="inputs")
    p.add_argument("--output", type=Path, default=None)
    p.add_argument("--pattern", default="*.csv")
    p.add_argument("--direction", choices=("loading", "all"), default="loading")
    p.add_argument("--force-range", nargs=2, type=float, default=(2.0, 20.0))
    p.add_argument("--rate-range", nargs=2, type=float, default=(-20.0, 20.0))
    p.add_argument("--force-bins", type=int, default=37); p.add_argument("--rate-bins", type=int, default=41)
    p.add_argument("--window", type=float, default=0.1, help="Derivative smoothing window in seconds")
    a = p.parse_args(argv); root = Path(__file__).resolve().parents[1]
    if a.force_range[0] >= a.force_range[1]: p.error("--force-range requires min < max")
    if a.rate_range[0] >= a.rate_range[1]: p.error("--rate-range requires min < max")
    if a.force_bins < 2 or a.rate_bins < 2: p.error("grid bin counts must be at least 2")
    if a.window <= 0: p.error("--window must be positive")
    indir = a.input_dir or root / "data"
    files = a.inputs or sorted(indir.glob(a.pattern))
    if not files: p.error("no input CSV files found")
    out = a.output or Path(__file__).resolve().parent / "inverse_map.npz"; out.parent.mkdir(parents=True, exist_ok=True)
    fg, rg, grid, count = build(files, a.force_range[0], a.force_range[1],
                                 a.rate_range[0], a.rate_range[1],
                                 a.force_bins, a.rate_bins, a.window,
                                 loading_only=(a.direction == "loading"))
    np.savez(out, force_grid=fg, rate_grid=rg, torque_grid=grid,
             metadata=json.dumps({"samples": count, "files": [str(x) for x in files], "window_s": a.window}))
    print(f"wrote {out} ({count} samples, {len(files)} files)")


if __name__ == "__main__": main()
