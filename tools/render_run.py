#!/usr/bin/env python3
"""Per-scenario preview: telemetry figure and a short animated replay.

This is the quick-look tool used while tuning a scenario. The submission video
is built separately by make_video.py; both draw through sihviz.py, so a change
to how the scene is depicted lands in both rather than drifting apart.

Produces, next to the input JSON:
    <scenario>_metrics.png   speed, clearance, replan latency, behaviour band
    <scenario>.mp4           animated replay
    <scenario>.gif           the same, smaller, for embedding

Usage:
    python tools/render_run.py results/village_road.json [...]
    python tools/render_run.py --no-anim results/*.json     # figures only
"""

import os
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sihviz as V

DPI = 120


def metrics_figure(data, out_path):
    """Four stacked panels summarising the whole run."""
    meta, ser = data["meta"], data["series"]
    t = ser["t"]

    fig, ax = plt.subplots(4, 1, figsize=(13, 9.5), sharex=True, dpi=DPI)
    fig.patch.set_facecolor(V.BG)
    fig.suptitle(f"{meta['name']}  —  {meta['desc']}",
                 color=V.INK, fontsize=14, y=0.975)

    ax[0].plot(t, ser["v"], color="#60a5fa", lw=1.8, label="speed")
    ax[0].plot(t, ser["v_cap"], color=V.DIM, lw=1.1, ls="--",
               label="behaviour speed cap")
    ax[0].set_ylabel("speed [m/s]")
    ax[0].legend(loc="upper right", fontsize=9, labelcolor=V.DIM,
                 facecolor=V.PANEL, edgecolor=V.LINE)

    clr = ser["min_clear"]
    ax[1].plot(t, clr, color="#34d399", lw=1.8)
    ax[1].axhline(0.0, color="#ef4444", lw=1.3, ls="--", label="contact")
    ax[1].axhline(0.3, color="#fb923c", lw=1.0, ls=":", label="near miss")
    ax[1].set_ylabel("clearance [m]")
    ax[1].set_ylim(min(-0.5, min(clr) - 0.2), min(12, max(clr)))
    ax[1].legend(loc="upper right", fontsize=9, labelcolor=V.DIM,
                 facecolor=V.PANEL, edgecolor=V.LINE)

    ax[2].plot(ser["latency_t"], ser["latency_ms"], color="#9a72d0", lw=1.0)
    ax[2].axhline(meta["latency_budget"], color="#ef4444", lw=1.1, ls="--",
                  label=f"budget {meta['latency_budget']:.0f} ms")
    ax[2].set_ylabel("replan [ms]")
    ax[2].legend(loc="upper right", fontsize=9, labelcolor=V.DIM,
                 facecolor=V.PANEL, edgecolor=V.LINE)

    states = ser["state"]
    for i in range(len(t) - 1):
        ax[3].axvspan(t[i], t[i + 1],
                      color=V.STATE_COLOR.get(states[i], "#555"), lw=0)
    ax[3].set_ylabel("behaviour")
    ax[3].set_yticks([])
    ax[3].set_xlabel("time [s]")
    ax[3].legend(handles=[plt.Line2D([0], [0], color=c, lw=7, label=s)
                          for s, c in V.STATE_COLOR.items()],
                 loc="upper center", ncol=6, fontsize=9, frameon=False,
                 labelcolor=V.DIM, bbox_to_anchor=(0.5, -0.32))

    for a in ax:
        a.set_facecolor(V.PANEL)
        a.grid(alpha=0.16, color=V.DIM)
        a.tick_params(colors=V.DIM, labelsize=10)
        a.yaxis.label.set_color(V.DIM)
        a.xaxis.label.set_color(V.DIM)
        for sp in a.spines.values():
            sp.set_color(V.LINE)

    fig.tight_layout(rect=[0, 0.02, 1, 0.96])
    fig.savefig(out_path, dpi=DPI, facecolor=V.BG)
    plt.close(fig)
    return out_path


def animate(data, out_mp4, out_gif, max_seconds=26.0, fps=25):
    """Ego-centred replay with two telemetry strips underneath."""
    frames = data["frames"]
    ser = data["series"]
    t_all, v_all = ser["t"], ser["v"]
    clr = ser["min_clear"]
    clr_hi = min(12, max(clr))
    classes = V.classes_in(data)

    stride = max(1, len(frames) // int(max_seconds * fps))
    sel = frames[::stride]

    tmp = tempfile.mkdtemp(prefix="sih_prev_")
    for k, f in enumerate(sel):
        fig = plt.figure(figsize=(14, 8), dpi=100)
        fig.patch.set_facecolor(V.BG)
        gs = fig.add_gridspec(3, 1, height_ratios=[5.6, 1.2, 1.2],
                              left=0.045, right=0.985, top=0.93, bottom=0.08,
                              hspace=0.38)

        ax = fig.add_subplot(gs[0])
        V.draw_scene(ax, data, f, span=19.0)

        st = f["state"]
        _, _, _, ev = f["ego"]
        fig.text(0.045, 0.955, data["meta"]["name"], color=V.INK,
                 fontsize=17, weight="semibold")
        fig.text(0.985, 0.955, f"t = {f['t']:5.1f} s     {ev:4.1f} m/s     {st}",
                 color=V.STATE_COLOR.get(st, V.DIM), fontsize=15, ha="right")

        V.draw_strip(fig.add_subplot(gs[1]), t_all, v_all, f["t"],
                     "#60a5fa", "speed\n[m/s]")
        axc = fig.add_subplot(gs[2])
        V.draw_strip(axc, t_all, clr, f["t"], "#34d399", "clearance\n[m]",
                     ylim=(-0.6, clr_hi), hline=(0.0, "#ef4444"))
        axc.set_xlabel("time [s]", fontsize=11, color=V.DIM)

        fig.savefig(os.path.join(tmp, f"f{k:06d}.png"), dpi=100, facecolor=V.BG)
        plt.close(fig)
        if k % 100 == 0:
            print(f"    frame {k}/{len(sel)}")

    V.encode(tmp, out_mp4, fps=fps, width=1400)
    V.encode(tmp, out_gif, fps=fps, width=820, gif=True)

    for fn in os.listdir(tmp):
        os.remove(os.path.join(tmp, fn))
    os.rmdir(tmp)


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    do_anim = "--no-anim" not in argv
    if not args:
        print(__doc__)
        return 1

    for path in args:
        if not os.path.exists(path):
            print(f"  missing: {path}")
            continue
        data = V.load(path)
        base = os.path.splitext(path)[0]
        print(f"rendering {os.path.basename(path)}")
        print("  " + metrics_figure(data, base + "_metrics.png"))
        if do_anim:
            animate(data, base + ".mp4", base + ".gif")
            print(f"  {base}.mp4 / .gif")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
