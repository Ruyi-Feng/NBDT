"""
SSM Heatmap Plot
Overlay event-density heatmap onto a background image.

Reads one or more event CSVs produced by ssm_event_extractor.py,
deduplicates per event_id, takes (conflict_x_m, conflict_y_m) as the
event location, converts to pixel coordinates of the background image,
and renders a 2D-histogram heatmap on top.
"""

import os
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
from scipy.ndimage import gaussian_filter


def collect_event_positions(csv_paths):
    """One row per event_id, taking conflict_x_m/y_m + extreme column if present."""
    frames = []
    for path in csv_paths:
        df = pd.read_csv(path, usecols=lambda c: c in {
            "event_id", "conflict_x_m", "conflict_y_m",
            "TTC_min", "DRAC_max", "PET_min",
        })
        if {"conflict_x_m", "conflict_y_m"}.difference(df.columns):
            print(f"  WARNING: {path} missing conflict_x_m/conflict_y_m, skipping.")
            continue
        per_event = df.groupby("event_id").first().reset_index()
        per_event = per_event.dropna(subset=["conflict_x_m", "conflict_y_m"])
        frames.append(per_event)
        print(f"  {os.path.basename(path)}: {len(per_event)} events")
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def main():
    parser = argparse.ArgumentParser(
        description="Plot 2D heatmap of risk-event locations on a background image."
    )
    parser.add_argument("--csv_paths", type=str, nargs="+", required=True)
    parser.add_argument("--background", type=str, required=True,
                        help="Path to background image (e.g. background.png).")
    parser.add_argument("--pix2meter", type=float, required=True,
                        help="Pixel-to-meter scale (meters per pixel).")
    parser.add_argument("--bins", type=int, default=120,
                        help="Number of bins per axis for the 2D histogram.")
    parser.add_argument("--sigma", type=float, default=1.5,
                        help="Gaussian smoothing sigma (in bin units).")
    parser.add_argument("--alpha", type=float, default=0.55)
    parser.add_argument("--cmap", type=str, default="hot")
    parser.add_argument("--region_polygon", type=str, default=None,
                        help="Optional region polygon JSON (meters) to overlay on the plot.")
    parser.add_argument("--title", type=str, default=None)
    parser.add_argument("--output_path", type=str, required=True)
    args = parser.parse_args()

    print(f"Reading event positions from {len(args.csv_paths)} file(s) ...")
    events = collect_event_positions(args.csv_paths)
    if events.empty:
        print("No event positions found.")
        return
    print(f"Total: {len(events)} events")

    bg = Image.open(args.background)
    w, h = bg.size
    scale = args.pix2meter

    px = events["conflict_x_m"].to_numpy() / scale
    py = events["conflict_y_m"].to_numpy() / scale

    H, xe, ye = np.histogram2d(
        px, py,
        bins=[args.bins, args.bins],
        range=[[0, w], [0, h]],
    )
    if args.sigma > 0:
        H = gaussian_filter(H, sigma=args.sigma)

    fig, ax = plt.subplots(figsize=(20, 11))
    ax.imshow(bg)
    Hm = np.ma.masked_where(H.T <= 0, H.T)
    im = ax.imshow(
        Hm, origin="lower",
        extent=[xe[0], xe[-1], ye[0], ye[-1]],
        cmap=args.cmap, alpha=args.alpha,
    )
    plt.colorbar(im, ax=ax, fraction=0.025, pad=0.01, label="event density (smoothed)")

    if args.region_polygon:
        with open(args.region_polygon) as f:
            rd = json.load(f)
        verts_m = rd.get("vertices_m") or rd.get("vertices") or rd
        verts_px = [(x / scale, y / scale) for x, y in verts_m]
        ax.add_patch(Polygon(verts_px, closed=True, fill=False,
                             edgecolor="lime", linewidth=2.5, label="region polygon"))
        ax.legend(loc="upper right", fontsize=12)

    ax.set_xlim(0, w); ax.set_ylim(h, 0)
    ax.set_title(args.title or f"Event density heatmap (n={len(events)})", fontsize=16)
    plt.tight_layout()
    plt.savefig(args.output_path, dpi=120)
    plt.close()
    print(f"Saved plot to {args.output_path}")


if __name__ == "__main__":
    main()
