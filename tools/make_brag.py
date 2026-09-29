#!/usr/bin/env python3
"""Build the /brag-slim launch video for SIH 26037.

Follows brag-output/brag-plan.md. Draws the real exported runs through the same
sihviz primitives the submission video uses, so this looks like the same piece
of work rather than a separate re-creation.

    python tools/make_brag.py            # stills for review
    python tools/make_brag.py --render   # full render, audio, poster, copy
"""

import math
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sihviz as V

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")
OUT = os.path.join(ROOT, "brag-output")
WORK = os.path.join(OUT, "work")
FRAMES = os.path.join(WORK, "frames")

W, H, FPS, DPI = 1920, 1080, 30, 120

# scene: (source, t_from, t_to, kind, line, sub, span)
# Windows come from measured closest-approach per road-user class, not from
# when the event was authored to happen -- the ego reaches the cattle at t = 67,
# not at the t = 10 they were released at. Span is per scene: tight for a close
# pass, wide when the point is traffic streaming across ahead.
SCENES = [
    ("cattle_crossing",   63.6, 68.6, "hook",  "Cattle step into the road.", None, 16.0),
    ("village_road",      27.0, 32.0, "title", "Adaptive Path Planning",
     "for unstructured Indian roads", 26.0),
    ("village_road",      17.0, 22.0, "beat",
     "An auto-rickshaw, head-on,\non your side of the road.", None, 14.0),
    ("market",            84.0, 89.0, "beat",
     "A market street with\nthe market still in it.", None, 14.0),
    ("urban_intersection", 26.0, 31.0, "beat",
     "No signals. No stop line.\nIt reads the gap and goes.", None, 19.0),
    (None,                 0.0,  0.0, "outro", "5 scenarios. 0 collisions.",
     "closed-loop simulation  ·  runs in MATLAB or Octave", 0.0),
]
DURS = [3.2, 3.4, 3.6, 3.6, 3.0, 3.2]          # sums to 20.0 s

BG   = "#0c0f14"
INK  = "#e8edf5"
DIM  = "#93a1b5"
OK   = "#34d399"


def ease(x):
    """Smooth 0..1 ramp."""
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def frame_at(data, t):
    """Nearest exported frame to a wall time."""
    fr = data["frames"]
    lo, hi = fr[0]["t"], fr[-1]["t"]
    t = max(lo, min(hi, t))
    idx = int(round((t - lo) / max(hi - lo, 1e-6) * (len(fr) - 1)))
    return fr[idx]


def draw(scene_i, u, data_cache):
    """Render one frame. u is 0..1 progress through the scene."""
    src, t0, t1, kind, line, sub, span = SCENES[scene_i]

    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI)
    fig.patch.set_facecolor(BG)

    # Type gets its own band rather than sitting over the road. Overlaid text
    # collided with whatever happened to be driving past -- a line landing on a
    # pushcart is unreadable, and moving the line per scene just moves the
    # problem. A fixed band is also what a launch video looks like.
    BAND = 0.30

    if src is not None:
        data = data_cache[src]
        ax = fig.add_axes([0, BAND, 1, 1 - BAND])
        f = frame_at(data, t0 + (t1 - t0) * u)
        V.draw_scene(ax, data, f, span=span, show_tracks=(kind != "title"))
        # draw_scene sets an equal aspect with the default box adjustment, so
        # matplotlib shrinks the AXES to the data's own shape. In a 16:9 frame
        # that leaves the road as a thin strip with dead space above and below.
        # Adjusting the data limits instead keeps the scale honest and fills
        # the frame.
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlim(f["ego"][0] - span, f["ego"][0] + span)
        # Vignette so overlaid type always has something to sit on.
        ax.add_patch(plt.Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                                   facecolor=BG, alpha=0.86 if kind == "title" else 0.14,
                                   zorder=20))

    # Dip to background at the seams rather than crossfading two busy road
    # scenes, which would double-expose into mush.
    dip = min(ease(u / 0.10), ease((1 - u) / 0.10))
    if dip < 1:
        fig.patches.append(plt.Rectangle((0, 0), 1, 1, transform=fig.transFigure,
                                         facecolor=BG, alpha=1 - dip, zorder=40))

    # Text: fast in, then hold. Never fast-in then gone.
    app = ease((u - 0.10) / 0.16)
    if kind == "title":
        fig.text(0.5, 0.56, line, ha="center", color=INK, fontsize=74,
                 weight="semibold", alpha=app, zorder=50)
        fig.text(0.5, 0.455, sub, ha="center", color=DIM, fontsize=30,
                 alpha=app, zorder=50)
    elif kind == "outro":
        fig.text(0.5, 0.545, line, ha="center", color=OK, fontsize=78,
                 weight="semibold", alpha=app, zorder=50)
        fig.text(0.5, 0.435, sub, ha="center", color=DIM, fontsize=26,
                 alpha=app, zorder=50)
        fig.text(0.5, 0.315, "Smart India Hackathon  ·  Problem Statement 26037",
                 ha="center", color="#55627a", fontsize=20, alpha=app, zorder=50)
    else:
        # Solid band under the type, with a hairline where it meets the scene.
        fig.patches.append(plt.Rectangle((0, 0), 1, BAND, transform=fig.transFigure,
                                         facecolor=BG, zorder=45))
        fig.patches.append(plt.Rectangle((0, BAND - 0.0016), 1, 0.0016,
                                         transform=fig.transFigure,
                                         facecolor="#2a3446", zorder=46))
        y = 0.085 + 0.010 * (1 - app)
        fig.text(0.055, y, line, ha="left", va="bottom", color=INK, fontsize=46,
                 weight="semibold", linespacing=1.32, alpha=app, zorder=50)
        if src is not None:
            nice = {"cattle_crossing": "sudden cattle crossing",
                    "village_road": "unmarked village road",
                    "market": "dense market street",
                    "urban_intersection": "unsignalled crossroads"}[src]
            # Label lives at the top of the SCENE, not in the band: a two-line
            # caption fills the band and the label landed on top of its first
            # line.
            fig.text(0.055, 0.925, nice.upper(), ha="left", color=OK,
                     fontsize=19, alpha=app * 0.95, zorder=50)
    return fig


def build_frames(data_cache, stills_only):
    os.makedirs(FRAMES, exist_ok=True)
    k = 0
    for i, dur in enumerate(DURS):
        n = int(round(dur * FPS))
        picks = [0.5] if stills_only else range(n)
        for j in picks:
            u = (j / max(n - 1, 1)) if not stills_only else 0.5
            fig = draw(i, u, data_cache)
            name = (os.path.join(WORK, f"still_{i}.png") if stills_only
                    else os.path.join(FRAMES, f"f{k:05d}.png"))
            fig.savefig(name, dpi=DPI, facecolor=BG)
            plt.close(fig)
            k += 1
        # Mid-transition stills too: that is where collisions and mush show up.
        if stills_only:
            fig = draw(i, 0.04, data_cache)
            fig.savefig(os.path.join(WORK, f"still_{i}_in.png"), dpi=DPI, facecolor=BG)
            plt.close(fig)
        print(f"  scene {i+1}/{len(DURS)}")
    return k


def audio(total_s, path):
    """One piece: a slow minor pad, a pulse on the cuts, whooshes at the seams.

    Written together rather than music with effects dropped on top -- the
    whooshes are filtered noise in the same register as the pad, and everything
    sits well under the footage.
    """
    sr = 48000
    n = int(total_s * sr)
    t = np.arange(n) / sr
    mix = np.zeros(n)

    # Pad: A minor triad plus a fifth, detuned slightly so it breathes.
    for f, g in ((110.0, 0.20), (130.81, 0.15), (164.81, 0.13), (220.0, 0.08)):
        for det in (-0.6, 0.6):
            mix += g * 0.5 * np.sin(2 * np.pi * (f + det) * t)
    # Slow swell.
    mix *= 0.35 + 0.22 * np.sin(2 * np.pi * t / 9.0 - np.pi / 2)

    # Pulse on the scene boundaries.
    edges = np.cumsum([0] + DURS)[:-1]
    for e in edges:
        i0 = int(e * sr)
        if i0 >= n:
            continue
        env_n = min(int(0.55 * sr), n - i0)
        env = np.exp(-np.arange(env_n) / (0.11 * sr))
        sweep = np.linspace(58, 40, env_n)
        mix[i0:i0 + env_n] += 0.34 * env * np.sin(
            2 * np.pi * np.cumsum(sweep) / sr)
        # Whoosh: filtered noise, same seam, quieter.
        nz = np.random.default_rng(int(e * 100)).normal(0, 1, env_n)
        nz = np.convolve(nz, np.ones(180) / 180, mode="same")
        mix[i0:i0 + env_n] += 0.10 * env * nz

    mix /= (np.max(np.abs(mix)) + 1e-9)
    mix *= 0.52
    fade = int(0.9 * sr)
    mix[:fade] *= np.linspace(0, 1, fade)
    mix[-fade:] *= np.linspace(1, 0, fade)

    st = np.stack([mix, mix], axis=1)
    raw = os.path.join(WORK, "audio.raw")
    (st * 32767).astype("<i2").tofile(raw)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "s16le",
                    "-ar", str(sr), "-ac", "2", "-i", raw, path], check=True)
    return path


def main():
    render = "--render" in sys.argv
    os.makedirs(WORK, exist_ok=True)

    cache = {}
    for s, *_ in SCENES:
        if s and s not in cache:
            cache[s] = V.load(os.path.join(RESULTS, f"{s}.json"))

    if not render:
        build_frames(cache, stills_only=True)
        print(f"stills in {WORK}")
        return 0

    total = build_frames(cache, stills_only=False)
    dur = total / FPS
    print(f"{total} frames, {dur:.1f} s")

    wav = audio(dur, os.path.join(WORK, "track.wav"))
    silent = os.path.join(WORK, "silent.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", os.path.join(FRAMES, "f%05d.png"),
                    "-vf", f"scale={W}:{H},format=yuv420p",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", silent],
                   check=True)

    out = os.path.join(OUT, "brag.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent, "-i", wav,
                    "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                    "-shortest", "-movflags", "+faststart", out], check=True)

    # Poster: a settled frame from the outro, where the claim is fully on screen.
    poster_src = os.path.join(FRAMES, f"f{total - int(0.9 * FPS):05d}.png")
    poster = os.path.join(OUT, "brag.jpg")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", poster_src,
                    "-q:v", "2", poster], check=True)

    # Bake the poster as frame 0 by REPLACING it, so duration and sync hold.
    import shutil
    shutil.copy(poster_src, os.path.join(FRAMES, "f00000.png"))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", os.path.join(FRAMES, "f%05d.png"),
                    "-vf", f"scale={W}:{H},format=yuv420p",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", silent],
                   check=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent, "-i", wav,
                    "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                    "-shortest", "-movflags", "+faststart", out], check=True)

    with open(os.path.join(OUT, "share-copy.txt"), "w", encoding="utf-8") as fh:
        fh.write(
            "Taught a car to drive on roads with no lane markings — where an "
            "auto-rickshaw comes head-on down the middle and the hardest obstacle "
            "in the system is a cow.\n\n"
            "Five Indian road scenarios, zero collisions, measured against ground "
            "truth rather than the car's own perception. Closed-loop simulation "
            "for Smart India Hackathon PS 26037.\n")

    sz = os.path.getsize(out) / 1e6
    print(f"\nwrote {out}  ({dur:.1f} s, {sz:.1f} MB)")
    print(f"wrote {poster}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
