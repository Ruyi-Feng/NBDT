"""
Waymo Ozone Trajectory Visualization
-------------------------------------
Reads `data/waymo_ozone.csv` (Waymo Open Motion-style trajectories) and
renders one scenario as an animated GIF: every agent is drawn as a
heading-aligned bounding box with a fading motion trail, color-coded by
object type. Designed to look clean on a dark background.

Run:
    python visualize_waymo_trajectory.py                 # default scenario
    python visualize_waymo_trajectory.py --scenario <id> # specific scenario
    python visualize_waymo_trajectory.py --list          # list scenario ids
"""

from __future__ import annotations

import argparse
import os
from collections import defaultdict, deque

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection, PolyCollection

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
# Default paths resolve relative to the repo root. This file lives at
# standardized_data_toolkit/utils/visualize_waymo_trajectory.py — two levels
# below the repo root, which is where the data/ folder lives.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CSV_PATH = os.path.join(_REPO_ROOT, "data", "waymo_ozone.csv")
OUT_PATH = os.path.join(_REPO_ROOT, "waymo_trajectory.gif")

# obj_type encoding in the dataset: 1 = vehicle, 2 = pedestrian, 3 = cyclist
# The ego / SDC (Waymo self-driving car) is encoded as obj_type=1 AND vtype=2 —
# there is exactly one such row-group per scenario. All other vehicles have
# vtype=1. We label that special agent with kind="ego" internally.
TYPE_NAMES = {1: "Vehicle", 2: "Pedestrian", 3: "Cyclist", "ego": "Ego (SDC)"}
TYPE_COLORS = {
    1: "#5DE2E7",     # cyan-teal for cars
    2: "#FFD166",     # warm yellow for pedestrians
    3: "#FF6B9D",     # pink for cyclists
    "ego": "#FF4D3D", # vivid red-orange for the Waymo ego vehicle
}
TRAIL_LEN = 25       # how many past frames to keep in the trail for normal agents
EGO_TRAIL_LEN = 80   # longer trail for the ego so its path stands out
FPS = 10             # output animation frame rate
DPI = 110            # output resolution
PAD_M = 8.0          # extra meters of padding around the scene bounds


def _kind(obj_type: int, vtype: int):
    """Map (obj_type, vtype) → drawing kind. The ego is the only car whose
    vtype=2; everything else falls back to obj_type."""
    if obj_type == 1 and vtype == 2:
        return "ego"
    return int(obj_type)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_scenario(csv_path: str, scenario_id: str | None) -> tuple[pd.DataFrame, str]:
    """Load a single scenario from the CSV. If scenario_id is None, pick a
    medium-density scenario automatically so the animation is visually rich
    but not overcrowded."""
    cols = [
        "scenarioId", "Frame", "carId", "obj_type", "vtype",
        "carCenterXm", "carCenterYm",
        "boundingBox1Xm", "boundingBox1Ym",
        "boundingBox2Xm", "boundingBox2Ym",
        "boundingBox3Xm", "boundingBox3Ym",
        "boundingBox4Xm", "boundingBox4Ym",
        "heading", "speed",
    ]
    # Stream-read only what we need, filter to the target scenario.
    if scenario_id is None:
        # First pass: count agents per scenario to pick a nice one.
        sample = pd.read_csv(csv_path, usecols=["scenarioId", "carId"])
        counts = sample.groupby("scenarioId")["carId"].nunique()
        # Pick the scenario whose agent count is closest to 80
        scenario_id = (counts - 80).abs().idxmin()
        print(f"[auto] selected scenario {scenario_id} with {counts[scenario_id]} agents")

    chunks = []
    for chunk in pd.read_csv(csv_path, usecols=cols, chunksize=500_000):
        chunks.append(chunk[chunk["scenarioId"] == scenario_id])
    df = pd.concat(chunks, ignore_index=True)
    if df.empty:
        raise SystemExit(f"No rows found for scenario {scenario_id!r}")
    df = df.sort_values(["Frame", "carId"]).reset_index(drop=True)
    return df, scenario_id


def list_scenarios(csv_path: str, n: int = 25) -> None:
    sample = pd.read_csv(csv_path, usecols=["scenarioId", "carId", "obj_type"])
    summary = (
        sample.groupby("scenarioId")
        .agg(agents=("carId", "nunique"),
             vehicles=("obj_type", lambda s: (s == 1).sum()),
             peds=("obj_type", lambda s: (s == 2).sum()),
             cyclists=("obj_type", lambda s: (s == 3).sum()))
        .sort_values("agents", ascending=False)
        .head(n)
    )
    print(summary.to_string())


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------
def build_animation(df: pd.DataFrame, scenario_id: str, out_path: str) -> None:
    frames = sorted(df["Frame"].unique())
    n_frames = len(frames)

    # Pre-group rows by frame for O(1) lookup during animation
    by_frame: dict[int, pd.DataFrame] = {f: g for f, g in df.groupby("Frame")}

    # Stable per-agent kind ("ego" or 1/2/3) — derived from (obj_type, vtype)
    first_seen = df.drop_duplicates("carId").set_index("carId")
    agent_kind: dict[int, object] = {
        cid: _kind(row.obj_type, row.vtype) for cid, row in first_seen.iterrows()
    }
    ego_ids = [cid for cid, k in agent_kind.items() if k == "ego"]
    ego_id = ego_ids[0] if ego_ids else None
    if ego_id is not None:
        print(f"[ego] scenario {scenario_id[:8]}…  ego carId={ego_id}")
    else:
        print(f"[ego] scenario {scenario_id[:8]}…  no ego flagged (unexpected)")

    # Per-agent rolling trail. Ego gets a longer trail so its path is clear.
    def _make_deque(cid):
        return deque(maxlen=EGO_TRAIL_LEN if cid == ego_id else TRAIL_LEN)
    trails: dict[int, deque] = {}

    # Scene bounds (with a little padding)
    xmin = df["carCenterXm"].min() - PAD_M
    xmax = df["carCenterXm"].max() + PAD_M
    ymin = df["carCenterYm"].min() - PAD_M
    ymax = df["carCenterYm"].max() + PAD_M

    # --- Figure setup ---------------------------------------------------------
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "axes.edgecolor": "#2a2f3a",
        "axes.labelcolor": "#cdd3df",
        "xtick.color": "#7a8290",
        "ytick.color": "#7a8290",
    })
    fig, ax = plt.subplots(figsize=(11, 9), dpi=DPI, facecolor="#0b0f17")
    ax.set_facecolor("#0b0f17")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # Subtle grid for spatial reference
    ax.grid(True, color="#1b2230", linewidth=0.6, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_linewidth(0.8)

    title = ax.set_title(
        f"Waymo Scenario  {scenario_id[:8]}…   frame 0/{n_frames-1}",
        color="#e6e9ef", fontsize=14, pad=14, loc="left", weight="bold",
    )

    # Legend (built once). Put ego first so it reads as the headliner.
    kinds_present = list(set(agent_kind.values()))
    kinds_present.sort(key=lambda k: (0 if k == "ego" else 1, str(k)))
    legend_handles = [
        mpatches.Patch(color=TYPE_COLORS[k], label=TYPE_NAMES[k])
        for k in kinds_present
    ]
    leg = ax.legend(
        handles=legend_handles, loc="upper right",
        facecolor="#0b0f17", edgecolor="#2a2f3a", labelcolor="#cdd3df",
        framealpha=0.9, fontsize=10,
    )
    for txt in leg.get_texts():
        txt.set_color("#cdd3df")

    # Mutable artists, replaced each frame
    box_collection = PolyCollection([], closed=True)
    ax.add_collection(box_collection)
    trail_collection = LineCollection([], linewidths=1.5)
    ax.add_collection(trail_collection)

    # Ego-specific artists: a glowing halo (scatter) + the ego's own bbox drawn
    # on top so it's always above NPCs, plus a text label that follows it.
    ego_halo = ax.scatter([], [], s=900, facecolor=_with_alpha(TYPE_COLORS["ego"], 0.18),
                          edgecolor="none", zorder=4)
    ego_box = PolyCollection([], closed=True, zorder=5)
    ax.add_collection(ego_box)
    ego_label = ax.text(0, 0, "", color=TYPE_COLORS["ego"], fontsize=9,
                        weight="bold", zorder=6, ha="left", va="bottom")

    # ------------------------------------------------------------------ update
    def update(frame_idx: int):
        f = frames[frame_idx]
        rows = by_frame.get(f)
        if rows is None:
            return box_collection, trail_collection, ego_halo, ego_box, ego_label, title

        # Update trails for every agent active in this frame and collect
        # bbox polygons. The ego is collected separately so it can be drawn
        # on top with a halo and a label.
        polys, face_colors, edge_colors = [], [], []
        ego_poly = None
        ego_xy = None
        ego_speed = None
        for r in rows.itertuples(index=False):
            cid = r.carId
            if cid not in trails:
                trails[cid] = _make_deque(cid)
            trails[cid].append((r.carCenterXm, r.carCenterYm))
            corners = [
                (r.boundingBox1Xm, r.boundingBox1Ym),
                (r.boundingBox2Xm, r.boundingBox2Ym),
                (r.boundingBox3Xm, r.boundingBox3Ym),
                (r.boundingBox4Xm, r.boundingBox4Ym),
            ]
            if cid == ego_id:
                ego_poly = corners
                ego_xy = (r.carCenterXm, r.carCenterYm)
                ego_speed = float(r.speed)
                continue
            polys.append(corners)
            c = TYPE_COLORS[agent_kind.get(cid, int(r.obj_type))]
            face_colors.append(_with_alpha(c, 0.55))
            edge_colors.append(c)

        box_collection.set_verts(polys)
        box_collection.set_facecolor(face_colors)
        box_collection.set_edgecolor(edge_colors)
        box_collection.set_linewidth(1.1)

        # Ego rendering (halo + filled box on top + speed label)
        if ego_poly is not None:
            ego_c = TYPE_COLORS["ego"]
            ego_box.set_verts([ego_poly])
            ego_box.set_facecolor([_with_alpha(ego_c, 0.85)])
            ego_box.set_edgecolor(["#ffffff"])
            ego_box.set_linewidth(1.6)
            ego_halo.set_offsets([ego_xy])
            ego_label.set_position((ego_xy[0] + 2.5, ego_xy[1] + 2.5))
            ego_label.set_text(f"EGO  {ego_speed:.1f} m/s")
        else:
            ego_box.set_verts([])
            ego_halo.set_offsets(np.empty((0, 2)))
            ego_label.set_text("")

        # Build fading trail segments (alpha grows toward the head)
        segments, colors, widths = [], [], []
        for cid, pts in trails.items():
            if len(pts) < 2:
                continue
            base = TYPE_COLORS[agent_kind.get(cid, 1)]
            is_ego = cid == ego_id
            pts_arr = np.asarray(pts)
            for i in range(len(pts_arr) - 1):
                segments.append([pts_arr[i], pts_arr[i + 1]])
                # Ego trail is brighter and slightly thicker
                if is_ego:
                    a = 0.15 + 0.85 * (i / max(1, len(pts_arr) - 1))
                    widths.append(2.6)
                else:
                    a = 0.08 + 0.65 * (i / max(1, len(pts_arr) - 1))
                    widths.append(1.4)
                colors.append(_with_alpha(base, a))
        trail_collection.set_segments(segments)
        trail_collection.set_color(colors)
        trail_collection.set_linewidth(widths)

        title.set_text(
            f"Waymo Scenario  {scenario_id[:8]}…   frame {frame_idx}/{n_frames-1}   "
            f"agents: {len(rows)}"
        )
        return box_collection, trail_collection, ego_halo, ego_box, ego_label, title

    anim = FuncAnimation(
        fig, update, frames=n_frames, interval=1000 / FPS, blit=False,
    )

    print(f"Rendering {n_frames} frames → {out_path}")
    anim.save(out_path, writer=PillowWriter(fps=FPS), dpi=DPI)
    plt.close(fig)
    print("Done.")


def _with_alpha(hex_color: str, alpha: float) -> tuple[float, float, float, float]:
    """Convert '#RRGGBB' + alpha to an (r,g,b,a) tuple in 0..1."""
    h = hex_color.lstrip("#")
    r, g, b = int(h[0:2], 16) / 255, int(h[2:4], 16) / 255, int(h[4:6], 16) / 255
    return (r, g, b, float(np.clip(alpha, 0, 1)))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--csv", default=CSV_PATH, help="path to waymo_ozone.csv")
    p.add_argument("--scenario", default=None, help="scenarioId to render")
    p.add_argument("--out", default=OUT_PATH, help="output GIF path")
    p.add_argument("--list", action="store_true",
                   help="list top scenarios by agent count and exit")
    args = p.parse_args()

    if args.list:
        list_scenarios(args.csv)
        return

    df, sid = load_scenario(args.csv, args.scenario)
    print(f"Loaded {len(df):,} rows  |  {df['carId'].nunique()} agents  |  "
          f"{df['Frame'].nunique()} frames")
    build_animation(df, sid, args.out)


if __name__ == "__main__":
    main()
