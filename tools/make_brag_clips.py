#!/usr/bin/env python3
"""Render clean scene-only clips for the /brag Hyperframes composition.

The composition plays real footage and puts DOM typography over it, so these
clips carry no text, no telemetry strips and no HUD -- just the bird's-eye
world drawn through the project's own sihviz primitives. Same code as the
submission video, so the two match.

    python tools/make_brag_clips.py <output-dir>
"""

import os
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sihviz as V

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")
W, H, FPS, DPI = 1920, 1080, 30, 120

# name, source run, t_from, t_to, visible span in metres
CLIPS = [
    ("cattle",  "cattle_crossing", 63.6, 73.4, 16.0),
    ("village", "village_road",    17.0, 22.45, 14.0),
]


def frame_at(data, t):
    fr = data["frames"]
    lo, hi = fr[0]["t"], fr[-1]["t"]
    t = max(lo, min(hi, t))
    return fr[int(round((t - lo) / max(hi - lo, 1e-6) * (len(fr) - 1)))]


def render_clip(name, src, t0, t1, span, outdir):
    data = V.load(os.path.join(RESULTS, f"{src}.json"))
    n = int(round((t1 - t0) * FPS))
    tmp = tempfile.mkdtemp(prefix=f"hf_{name}_")

    for k in range(n):
        u = k / max(n - 1, 1)
        f = frame_at(data, t0 + (t1 - t0) * u)

        fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI)
        fig.patch.set_facecolor(V.BG)
        ax = fig.add_axes([0, 0, 1, 1])
        V.draw_scene(ax, data, f, span=span)
        # Fill the frame rather than letting the equal aspect shrink the axes
        # to the data's shape, which leaves the road as a letterboxed strip.
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlim(f["ego"][0] - span, f["ego"][0] + span)
        fig.savefig(os.path.join(tmp, f"f{k:05d}.png"), dpi=DPI, facecolor=V.BG)
        plt.close(fig)
        if k % 60 == 0:
            print(f"    {name} {k}/{n}")

    out = os.path.join(outdir, f"{name}.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", os.path.join(tmp, "f%05d.png"),
                    "-vf", f"scale={W}:{H},format=yuv420p",
                    "-c:v", "libx264", "-preset", "medium", "-crf", "20",
                    "-movflags", "+faststart", out], check=True)
    for fn in os.listdir(tmp):
        os.remove(os.path.join(tmp, fn))
    os.rmdir(tmp)
    print(f"  {out}  ({n / FPS:.2f}s, {os.path.getsize(out)/1e6:.1f} MB)")
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    outdir = os.path.join(sys.argv[1], "composition", "assets", "video")
    os.makedirs(outdir, exist_ok=True)
    for name, src, t0, t1, span in CLIPS:
        render_clip(name, src, t0, t1, span, outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
