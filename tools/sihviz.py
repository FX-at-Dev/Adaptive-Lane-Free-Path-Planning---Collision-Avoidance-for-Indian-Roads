"""Shared drawing primitives for every SIH 26037 visual output.

Both the per-scenario preview (render_run.py) and the submission video
(make_video.py) draw from here, so a change to how a rickshaw or a planned
trajectory is depicted lands in both. The alternative — a copy of the drawing
code per output — guarantees the preview and the submitted video eventually
disagree about what they are showing.

Everything reads the JSON written by sim/sih_export_run.m, so the picture is
always of the run the metrics were computed from.
"""

import json
import math
import os
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

# ---------------------------------------------------------------- palette
BG      = "#0c0f14"
PANEL   = "#141922"
LINE    = "#2a3446"
INK     = "#e8edf5"
DIM     = "#93a1b5"
ROAD    = "#39465c"
CENTRE  = "#41506b"

EGO     = "#f5f7fa"
PLAN    = "#22d3aa"
TRACK   = "#60a5fa"

STATE_COLOR = {
    "CRUISE": "#34d399", "FOLLOW": "#60a5fa", "NUDGE": "#fbbf24",
    "YIELD":  "#fb923c", "CREEP":  "#f87171", "STOP":  "#ef4444",
}

# Road-user classes named in the problem statement, each visually distinct.
CLASS_STYLE = {
    "car":         "#5b8fd6",
    "bus":         "#3b63a8",
    "truck":       "#4a6fa0",
    "auto":        "#f2c744",
    "two_wheeler": "#e4813b",
    "bicycle":     "#e8a33d",
    "pedestrian":  "#ef5d75",
    "cattle":      "#b07a4a",
    "pushcart":    "#9a72d0",
    "static":      "#7b8698",
}

CLASS_SIZE = {
    "car": (4.2, 1.8), "bus": (11.0, 2.6), "truck": (8.5, 2.5),
    "auto": (2.6, 1.4), "two_wheeler": (1.9, 0.7), "bicycle": (1.7, 0.6),
    "pedestrian": (0.6, 0.6), "cattle": (2.2, 0.9), "pushcart": (2.0, 1.2),
    "static": (3.0, 1.6),
}

CLASS_LABEL = {
    "car": "car", "bus": "bus", "truck": "truck", "auto": "auto-rickshaw",
    "two_wheeler": "two-wheeler", "bicycle": "bicycle",
    "pedestrian": "pedestrian", "cattle": "cattle", "pushcart": "pushcart",
    "static": "obstacle",
}


# ---------------------------------------------------------------- loading
def load(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def rows(v, width):
    """Normalise an exported matrix to a list of fixed-width rows.

    Octave's jsonencode collapses a 1 x N matrix to a flat array and an empty
    one to [], so a frame holding exactly one tracked object arrives as
    [x, y, psi, v] rather than [[x, y, psi, v]]. Reshaping here keeps that
    quirk out of every drawing routine.
    """
    if not v:
        return []
    if isinstance(v[0], (int, float)):
        return [list(v[i:i + width]) for i in range(0, len(v), width)]
    return v


def as_list(v):
    """Normalise a cell array that may have collapsed to a bare string."""
    if v is None:
        return []
    if isinstance(v, str):
        return [v]
    return list(v)


def road_edges(rx, ry, hw):
    """Offset the corridor centreline by +/- its half-width."""
    left, right = [], []
    n = len(rx)
    for i in range(n):
        i0, i1 = max(0, i - 1), min(n - 1, i + 1)
        dx, dy = rx[i1] - rx[i0], ry[i1] - ry[i0]
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm, dx / norm
        left.append((rx[i] + nx * hw[i], ry[i] + ny * hw[i]))
        right.append((rx[i] - nx * hw[i], ry[i] - ny * hw[i]))
    return left, right


# ---------------------------------------------------------------- drawing
def vehicle(ax, x, y, psi, length, width, color, z=4, edge=None, lw=0.0, alpha=1.0):
    """A footprint as a rotated rectangle centred on (x, y)."""
    c, s = math.cos(psi), math.sin(psi)
    hl, hw = length / 2.0, width / 2.0
    pts = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    xy = [(x + px * c - py * s, y + px * s + py * c) for px, py in pts]
    ax.add_patch(Polygon(xy, closed=True, facecolor=color,
                         edgecolor=edge or "none", linewidth=lw,
                         alpha=alpha, zorder=z))


def draw_scene(ax, data, frame, span=30.0, show_tracks=True):
    """Ego-centred bird's-eye view of one frame.

    span sets how many metres are visible ahead of and behind the vehicle. It
    is deliberately tighter than the road is long: the point of this view is to
    see the vehicle's relationship to the traffic immediately around it, and a
    wide view makes a rickshaw three pixels across.
    """
    meta, road = data["meta"], data["road"]
    rx, ry, hw = road["x"], road["y"], road["hw"]
    left, right = road_edges(rx, ry, hw)

    ex, ey, epsi, ev = frame["ego"]

    # Carriageway as a filled band, so the drivable corridor reads at a glance.
    band = left + right[::-1]
    ax.add_patch(Polygon(band, closed=True, facecolor=ROAD, alpha=0.48,
                         edgecolor="none", zorder=0))
    ax.plot([p[0] for p in left], [p[1] for p in left], color=LINE, lw=1.6, zorder=1)
    ax.plot([p[0] for p in right], [p[1] for p in right], color=LINE, lw=1.6, zorder=1)

    # The corridor reference. Dashed and faint on purpose: it is guidance, not
    # a lane the vehicle is obliged to hold.
    ax.plot(rx, ry, color=CENTRE, lw=1.0, ls=(0, (6, 6)),
            alpha=0.55, zorder=1)

    gx, gy = meta["goal"]
    ax.plot(gx, gy, marker="*", ms=22, color="#34d399",
            markeredgecolor="#0c0f14", markeredgewidth=1.0, zorder=2)

    # Ground-truth road users.
    for i, (ax_, ay_, apsi) in enumerate(rows(frame["agents"], 3)):
        cls = _cls(as_list(frame["agent_class"]), i)
        L, W = CLASS_SIZE.get(cls, (2.0, 1.0))
        vehicle(ax, ax_, ay_, apsi, L, W, CLASS_STYLE.get(cls, "#888"), z=5)

    # Tracked estimates as hollow rings. The offset between a ring and the
    # solid shape under it IS the perception error, shown rather than described.
    if show_tracks:
        for i, (tx, ty, tpsi, tv) in enumerate(rows(frame["tracks"], 4)):
            ax.add_patch(Circle((tx, ty), 1.25, fill=False, edgecolor=TRACK,
                                lw=1.6, ls=(0, (2, 2)), alpha=0.9, zorder=6))

    # The current plan.
    tj = frame["traj"]
    if tj and len(tj) == 2 and isinstance(tj[0], list) and tj[0]:
        ax.plot(tj[0], tj[1], color=PLAN, lw=3.4, alpha=0.95, zorder=7,
                solid_capstyle="round")

    # Ego. Its pose is the rear axle, so shift forward to centre the body.
    vehicle(ax, ex + math.cos(epsi) * 1.35, ey + math.sin(epsi) * 1.35, epsi,
            meta["ego_length"], meta["ego_width"], EGO, z=8,
            edge="#0c0f14", lw=1.2)

    ax.set_xlim(ex - span, ex + span)
    ax.set_ylim(ey - span * 9 / 32, ey + span * 9 / 32)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_facecolor(BG)
    for sp in ax.spines.values():
        sp.set_visible(False)


def draw_strip(ax, t_all, series, t_now, color, label, ylim=None, hline=None):
    """One telemetry strip with a playhead at the current time."""
    ax.plot(t_all, series, color=color, lw=1.8)
    if hline is not None:
        ax.axhline(hline[0], color=hline[1], lw=1.2, ls="--", alpha=0.85)
    ax.axvline(t_now, color="#ef4444", lw=1.6)
    ax.set_xlim(t_all[0], t_all[-1])
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_ylabel(label, fontsize=11, color=DIM)
    ax.tick_params(labelsize=9, colors=DIM)
    ax.set_facecolor(PANEL)
    ax.grid(alpha=0.16, color=DIM)
    for sp in ax.spines.values():
        sp.set_color(LINE)


def _cls(lst, i):
    return lst[i] if i < len(lst) else "car"


def legend_handles(classes):
    """Legend entries for the classes actually present in a scenario."""
    hs = [
        plt.Line2D([0], [0], marker="s", color="none", markerfacecolor=EGO,
                   markersize=11, label="ego vehicle"),
        plt.Line2D([0], [0], color=PLAN, lw=3, label="planned trajectory (10 Hz)"),
        plt.Line2D([0], [0], marker="o", color="none", markeredgecolor=TRACK,
                   markerfacecolor="none", markersize=11, label="tracked estimate"),
    ]
    for c in classes:
        hs.append(plt.Line2D([0], [0], marker="s", color="none",
                             markerfacecolor=CLASS_STYLE.get(c, "#888"),
                             markersize=11, label=CLASS_LABEL.get(c, c)))
    return hs


def classes_in(data):
    """Every road-user class that appears anywhere in a run, in a stable order."""
    seen = []
    for f in data["frames"]:
        for c in as_list(f["agent_class"]):
            if c not in seen:
                seen.append(c)
    order = list(CLASS_STYLE.keys())
    seen.sort(key=lambda c: order.index(c) if c in order else 99)
    return seen


# ---------------------------------------------------------------- encoding
def encode(frame_dir, out_path, fps=25, width=1920, gif=False):
    """Assemble numbered PNG frames into an MP4, or a palette-optimised GIF."""
    if gif:
        vf = (f"fps=12,scale={width}:-2:flags=lanczos,split[a][b];"
              "[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer")
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
               "-i", os.path.join(frame_dir, "f%06d.png"), "-vf", vf, out_path]
    else:
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
               "-i", os.path.join(frame_dir, "f%06d.png"),
               "-vf", f"scale={width}:-2,format=yuv420p",
               "-c:v", "libx264", "-preset", "medium", "-crf", "19",
               "-movflags", "+faststart", out_path]
    subprocess.run(cmd, check=True)
    return out_path
