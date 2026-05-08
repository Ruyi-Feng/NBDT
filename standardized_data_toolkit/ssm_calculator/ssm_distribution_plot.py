"""
SSM Distribution Plot
Read the event CSV(s) produced by ssm_event_extractor.py,
extract the per-event extreme column, and plot its distribution as a histogram.
"""

import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt


def collect_extremes_from_csvs(csv_paths, extreme_col):
    """
    Read one or more event CSVs, deduplicate by event_id,
    and return the list of extreme values.
    """
    all_dfs = []
    for path in csv_paths:
        df = pd.read_csv(path)
        if extreme_col not in df.columns:
            print(f"  WARNING: column '{extreme_col}' not found in {path}, skipping.")
            continue
        # each event_id has the same extreme value on every row, take one per event
        event_extremes = df.groupby("event_id")[extreme_col].first()
        all_dfs.append(event_extremes)
        print(f"  {os.path.basename(path)}: {len(event_extremes)} events")

    if not all_dfs:
        return []

    combined = pd.concat(all_dfs)
    return combined.tolist()


def plot_distribution(values, extreme_col, bins, threshold_line, title, output_path):
    """
    Plot a histogram of event-level extreme values.
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.hist(values, bins=bins, edgecolor="black", alpha=0.7)

    if threshold_line is not None:
        ax.axvline(x=threshold_line, color="red", linestyle="--",
                   linewidth=1.5, label=f"Threshold = {threshold_line}")
        ax.legend()

    if title:
        ax.set_title(title, fontsize=14)
    else:
        ax.set_title(f"Distribution of {extreme_col} (n={len(values)} events)", fontsize=14)

    ax.set_xlabel(extreme_col, fontsize=12)
    ax.set_ylabel("Frequency", fontsize=12)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot distribution of event-level SSM extremes from extractor CSV(s)."
    )
    parser.add_argument(
        "--csv_paths", type=str, nargs="+", required=True,
        help="One or more event CSV files produced by ssm_event_extractor.py."
    )
    parser.add_argument(
        "--extreme_col", type=str, default="TTC_min",
        help="Name of the extreme value column in the CSV (default: TTC_min)."
    )
    parser.add_argument(
        "--bins", type=int, default=50,
        help="Number of histogram bins (default: 50)."
    )
    parser.add_argument(
        "--threshold_line", type=float, default=None,
        help="Optional: draw a vertical threshold reference line."
    )
    parser.add_argument(
        "--output_path", type=str, default=None,
        help="Output image path. Default: {first_csv_dir}/{extreme_col}_distribution.png"
    )
    parser.add_argument(
        "--title", type=str, default=None,
        help="Optional custom chart title."
    )
    args = parser.parse_args()

    print(f"Reading extreme column '{args.extreme_col}' from {len(args.csv_paths)} file(s) ...")
    values = collect_extremes_from_csvs(args.csv_paths, args.extreme_col)

    if not values:
        print("No event data found. No plot generated.")
        return

    print(f"Total: {len(values)} events")

    output_path = args.output_path
    if output_path is None:
        first_dir = os.path.dirname(args.csv_paths[0]) or "."
        output_path = os.path.join(first_dir, f"{args.extreme_col}_distribution.png")

    plot_distribution(values, args.extreme_col, args.bins, args.threshold_line,
                      args.title, output_path)


if __name__ == "__main__":
    main()
