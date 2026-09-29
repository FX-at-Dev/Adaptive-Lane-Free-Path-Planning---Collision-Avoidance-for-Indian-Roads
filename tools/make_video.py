#!/usr/bin/env python3
"""Build the SIH 26037 submission video from exported scenario runs.

Assembles a single 1080p MP4: title, the problem, the architecture, each
scenario replay with live telemetry, the aggregate results, and an honest
statement of what is and is not built yet.

Static cards are rendered once and held by ffmpeg for their duration rather
than being written out as hundreds of identical frames. Only the scenario
replays are rendered frame by frame.

Usage:
    python tools/make_video.py                 # every scenario found
    python tools/make_video.py village_road    # a subset, in the given order
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sihviz as V

W, H = 1920, 1080
DPI = 120
FPS = 25
FIGSIZE = (W / DPI, H / DPI)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")

# Order matters: the scenarios build from the simplest situation to the one
# that most tests the prediction layer.
ORDER = ["village_road", "urban_intersection", "highway_merge",
         "market", "cattle_crossing"]

# Cold open. The /brag video is a 20 s hook — the cattle, the loop, the claim —
# and putting it in front of the technical walkthrough gives a judge working
# through many entries a reason to keep watching. When it is present the
# submission's own title card is dropped: brag already opens on the title, and
# two title cards back to back is the one thing that would make the join
# obvious. Falls back to the plain title card if the file is missing.
def find_cold_open():
    # Newest by modification time, not by name. Sorting lexically puts the
    # un-timestamped `brag-output/` after `brag-output-2026-...`, so it would
    # quietly pick an older run over the latest one.
    import glob
    hits = glob.glob(os.path.join(ROOT, "brag-output*", "brag.mp4"))
    return max(hits, key=os.path.getmtime) if hits else None


# Music bed for the body. The open carries its own mixed audio; without this the
# remaining three minutes would be silent, which is a worse join than no music
# at all.
MUSIC = os.path.join(
    os.path.expanduser("~"), ".claude", "skills", "brag", "assets", "music",
    "happy-beats-business-moves-vol-10-by-ende-dot-app.mp3")

TITLES = {
    "village_road":       ("Unmarked village road",
                           "No centre line. An auto-rickshaw comes head-on down the middle,\n"
                           "a cyclist blocks the nearside, a pedestrian crosses, a pushcart narrows the road."),
    "urban_intersection": ("Unsignalled urban crossroads",
                           "No signals, no stop line, no give-way. Traffic crosses from both sides\n"
                           "and concedes nothing — right of way is taken, not granted."),
    "highway_merge":      ("Highway merge into slow traffic",
                           "Joining from a slip road behind a truck and a bus, with two-wheelers\n"
                           "filtering past on both sides and nobody opening a gap."),
    "market":             ("Dense market street",
                           "Stalls and parked carts narrow the road to barely more than the vehicle.\n"
                           "Pedestrians step out from behind them with almost no warning."),
    "cattle_crossing":    ("Sudden cattle crossing",
                           "Cattle emerge from behind a parked truck at 11 m/s closing speed.\n"
                           "They do not hold a line and they do not react to the vehicle."),
}


# ------------------------------------------------------------------ cards
def new_fig():
    fig = plt.figure(figsize=FIGSIZE, dpi=DPI)
    fig.patch.set_facecolor(V.BG)
    return fig


def save_card(fig, path):
    fig.savefig(path, dpi=DPI, facecolor=V.BG)
    plt.close(fig)
    return path


def card_title(path):
    fig = new_fig()
    fig.text(0.5, 0.735, " ".join("SMART INDIA HACKATHON  ·  PROBLEM STATEMENT 26037"),
             ha="center", color="#34d399", fontsize=17)
    fig.text(0.5, 0.60, "Adaptive Path Planning &", ha="center",
             color=V.INK, fontsize=58, weight="semibold")
    fig.text(0.5, 0.505, "Collision Avoidance", ha="center",
             color=V.INK, fontsize=58, weight="semibold")
    fig.text(0.5, 0.395, "for autonomous vehicles on unstructured Indian roads",
             ha="center", color=V.DIM, fontsize=25)
    fig.text(0.5, 0.235,
             "Closed-loop simulation  ·  perception → fusion → prediction → planning → control",
             ha="center", color="#6b7a91", fontsize=18)
    fig.text(0.5, 0.135, "MathWorks  ·  Smart Vehicles", ha="center",
             color="#55627a", fontsize=16)
    return save_card(fig, path)


def card_problem(path):
    fig = new_fig()
    fig.text(0.07, 0.86, "The problem", color=V.INK, fontsize=40, weight="semibold")
    fig.text(0.07, 0.785,
             "Standard planners assume the road tells the vehicle where to go.",
             color="#34d399", fontsize=22)

    left = [
        ("What they assume", "#60a5fa", [
            "lane markings define the path",
            "traffic keeps to its lane",
            "motion is predictable from the lane",
            "intersections are controlled",
            "road users share one behaviour model",
        ]),
        ("What is actually there", "#f87171", [
            "no markings, soft and shifting road edges",
            "cars, buses, autos, two-wheelers, carts, cattle",
            "lateral movement whenever a gap appears",
            "crossings negotiated by who commits first",
            "a cow and a bus do not move alike at all",
        ]),
    ]
    for col, (heading, colour, items) in enumerate(left):
        x = 0.09 + col * 0.46
        fig.text(x, 0.66, heading, color=colour, fontsize=24, weight="semibold")
        for i, it in enumerate(items):
            fig.text(x, 0.575 - i * 0.072, "—  " + it,
                     color=V.DIM, fontsize=19)

    fig.text(0.07, 0.115,
             "So the planner cannot be told where the lane is. It has to work out where there is room.",
             color=V.INK, fontsize=22, style="italic")
    return save_card(fig, path)


def card_architecture(path):
    fig = new_fig()
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)

    fig.text(0.07, 0.90, "Pipeline", color=V.INK, fontsize=40, weight="semibold")
    fig.text(0.07, 0.845, "Every block runs in closed loop at 20 Hz; planning and decision at 10 Hz.",
             color=V.DIM, fontsize=20)

    blocks = [
        ("Camera\nRadar\nLiDAR",        "#3b63a8", "range, bearing,\nocclusion, clutter"),
        ("Fusion\nEKF + GNN",           "#4a6fa0", "CTRV tracks,\nM-of-N management"),
        ("Prediction\nmulti-hypothesis","#9a72d0", "class-conditioned,\ntime-discounted risk"),
        ("Planner\nFrenet lattice",     "#22d3aa", "66 candidates,\ncorridor not lane"),
        ("Behaviour\nstate machine",    "#fbbf24", "speed cap, lateral\nfreedom, risk tolerance"),
        ("Vehicle\nbicycle model",      "#e4813b", "pure pursuit +\nfeed-forward speed"),
    ]
    n = len(blocks)
    bw, bh = 0.138, 0.26
    gap = (0.885 - n * bw) / (n - 1)
    y = 0.43

    for i, (name, colour, sub) in enumerate(blocks):
        x = 0.06 + i * (bw + gap)
        ax.add_patch(FancyBboxPatch((x, y), bw, bh,
                                    boxstyle="round,pad=0.008,rounding_size=0.012",
                                    facecolor=V.PANEL, edgecolor=colour, linewidth=2.2))
        ax.text(x + bw / 2, y + bh * 0.70, name, ha="center", va="center",
                color=colour, fontsize=16, weight="semibold", linespacing=1.35)
        ax.text(x + bw / 2, y + bh * 0.24, sub, ha="center", va="center",
                color=V.DIM, fontsize=11, linespacing=1.45)
        if i < n - 1:
            ax.add_patch(FancyArrowPatch((x + bw + 0.004, y + bh / 2),
                                         (x + bw + gap - 0.004, y + bh / 2),
                                         arrowstyle="-|>", mutation_scale=17,
                                         color="#55627a", linewidth=1.8))

    # The feedback path is the point of the diagram: it is a closed loop.
    ax.add_patch(FancyArrowPatch((0.94, y - 0.055), (0.06, y - 0.055),
                                 connectionstyle="arc3,rad=0.14",
                                 arrowstyle="-|>", mutation_scale=17,
                                 color="#55627a", linewidth=1.6, linestyle="--"))
    ax.text(0.5, 0.235, "world state — the loop is closed, not replayed",
            ha="center", color="#6b7a91", fontsize=16, style="italic")

    fig.text(0.07, 0.085,
             "Toolbox-free core: it runs identically under MATLAB and GNU Octave.",
             color="#55627a", fontsize=17)
    return save_card(fig, path)


def card_scenario_intro(path, name, index, total):
    title, blurb = TITLES.get(name, (name, ""))
    fig = new_fig()
    fig.text(0.07, 0.70, " ".join(f"SCENARIO {index} OF {total}"), color="#34d399",
             fontsize=17)
    fig.text(0.07, 0.585, title, color=V.INK, fontsize=50, weight="semibold")
    fig.text(0.07, 0.40, blurb, color=V.DIM, fontsize=23, linespacing=1.7,
             va="top")
    return save_card(fig, path)


def card_results(path, runs):
    fig = new_fig()
    fig.text(0.06, 0.90, "Results", color=V.INK, fontsize=40, weight="semibold")
    fig.text(0.06, 0.845,
             "Clearance and collisions are measured against ground truth, not the vehicle's own tracks.",
             color=V.DIM, fontsize=18)

    cols = ["scenario", "goal", "collisions", "min clearance",
            "time to goal", "replan p95", "peak curvature"]
    xs = [0.06, 0.40, 0.49, 0.61, 0.735, 0.85, 0.945]
    y0, dy = 0.735, 0.082

    for x, c in zip(xs, cols):
        fig.text(x, y0, " ".join(c.upper()), color="#6b7a91", fontsize=14, ha="left" if x < 0.2 else "center")

    n_done = n_coll = 0
    for i, r in enumerate(runs):
        y = y0 - (i + 1) * dy
        m, res = r["metrics"], r["result"]
        reached = bool(res["reached"]); collided = bool(res["collided"])
        n_done += reached; n_coll += collided

        fig.text(xs[0], y, TITLES.get(r["name"], (r["name"], ""))[0],
                 color=V.INK, fontsize=19)
        fig.text(xs[1], y, "reached" if reached else "not reached",
                 color="#34d399" if reached else "#f87171", fontsize=18, ha="center")
        fig.text(xs[2], y, "0" if not collided else "1",
                 color="#34d399" if not collided else "#f87171",
                 fontsize=19, ha="center", weight="semibold")
        fig.text(xs[3], y, f"{m['min_clear']:.2f} m", color=V.INK, fontsize=18, ha="center")
        t = m.get("t_to_goal")
        fig.text(xs[4], y, f"{t:.1f} s" if isinstance(t, (int, float)) else "—",
                 color=V.INK, fontsize=18, ha="center")
        fig.text(xs[5], y, f"{m['latency_p95_ms']:.0f} ms", color=V.INK, fontsize=18, ha="center")
        fig.text(xs[6], y, f"{m['curv_max']:.3f}", color=V.INK, fontsize=18, ha="center")

    fig.text(0.06, 0.20,
             f"{n_done} of {len(runs)} scenarios completed   ·   {n_coll} collisions",
             color="#34d399" if n_coll == 0 else "#fbbf24",
             fontsize=30, weight="semibold")
    fig.text(0.06, 0.12,
             "Replanning latency is wall-clock under an interpreter on a laptop — a measure of "
             "relative planner cost, not an embedded real-time claim.",
             color="#55627a", fontsize=15)
    return save_card(fig, path)


def card_honest(path):
    fig = new_fig()
    fig.text(0.07, 0.88, "What is built, and what is not",
             color=V.INK, fontsize=40, weight="semibold")

    done = [
        "Closed-loop pipeline: sensing, fusion, prediction, planning, decision, dynamics",
        "All five required scenarios, running collision-free",
        "Unit tests for the mathematics, plus a full closed-loop regression test",
        "Metrics: latency, smoothness, clearance, completion",
    ]
    todo = [
        "Simulink model and Stateflow chart — the behaviour logic is written to port cleanly",
        "RoadRunner scenes — roads are authored as OpenDRIVE, importable when licensed",
        "Detection on the Indian Driving Dataset — perception is currently a sensor model",
        "Hybrid A* fallback — written and verified in isolation, but disabled: enabling it cost latency and a scenario",
    ]

    fig.text(0.07, 0.775, "Working and verified", color="#34d399",
             fontsize=24, weight="semibold")
    for i, d in enumerate(done):
        fig.text(0.075, 0.705 - i * 0.062, "✓   " + d, color=V.DIM, fontsize=18)

    fig.text(0.07, 0.40, "Not yet built", color="#fbbf24",
             fontsize=24, weight="semibold")
    for i, d in enumerate(todo):
        fig.text(0.075, 0.330 - i * 0.062, "∘   " + d, color=V.DIM, fontsize=18)
    return save_card(fig, path)


def card_close(path):
    fig = new_fig()
    fig.text(0.5, 0.56, "Adaptive Path Planning & Collision Avoidance",
             ha="center", color=V.INK, fontsize=38, weight="semibold")
    fig.text(0.5, 0.465, "Problem Statement 26037  ·  MathWorks  ·  Smart Vehicles",
             ha="center", color=V.DIM, fontsize=21)
    fig.text(0.5, 0.33, "Simulation, scenarios, metrics and tests are reproducible from source.",
             ha="center", color="#55627a", fontsize=17)
    return save_card(fig, path)


# -------------------------------------------------------------- replay
def render_replay(data, frame_dir, max_seconds=30.0):
    """Write the per-frame PNGs for one scenario replay."""
    frames = data["frames"]
    ser = data["series"]
    meta = data["meta"]

    # Keep every scenario to a similar on-screen length regardless of how long
    # the run took, so a 90 s market crawl does not dominate the video.
    budget = int(max_seconds * FPS)
    stride = max(1, len(frames) // budget)
    sel = frames[::stride]

    t_all = ser["t"]
    v_all = ser["v"]
    clr = ser["min_clear"]
    classes = V.classes_in(data)
    title, _ = TITLES.get(meta["name"], (meta["name"], ""))
    clr_hi = min(12, max(clr))

    for k, f in enumerate(sel):
        fig = plt.figure(figsize=FIGSIZE, dpi=DPI)
        fig.patch.set_facecolor(V.BG)

        gs = fig.add_gridspec(3, 2, height_ratios=[6.2, 1.25, 1.25],
                              width_ratios=[4.25, 1.0],
                              left=0.035, right=0.975, top=0.90, bottom=0.075,
                              hspace=0.42, wspace=0.045)

        ax = fig.add_subplot(gs[0, :])
        V.draw_scene(ax, data, f, span=19.0)

        st = f["state"]
        ex, ey, epsi, ev = f["ego"]
        fig.text(0.035, 0.945, title, color=V.INK, fontsize=26, weight="semibold")
        fig.text(0.975, 0.952, f"t = {f['t']:5.1f} s", color=V.DIM,
                 fontsize=19, ha="right")
        fig.text(0.975, 0.917, f"{ev:4.1f} m/s", color=V.INK,
                 fontsize=19, ha="right")
        fig.text(0.802, 0.9345, st, color=V.BG, fontsize=17, ha="center",
                 va="center", weight="semibold",
                 bbox=dict(boxstyle="round,pad=0.45",
                           facecolor=V.STATE_COLOR.get(st, "#888"),
                           edgecolor="none"))

        axv = fig.add_subplot(gs[1, 0])
        V.draw_strip(axv, t_all, v_all, f["t"], "#60a5fa", "speed\n[m/s]")
        axc = fig.add_subplot(gs[2, 0])
        V.draw_strip(axc, t_all, clr, f["t"], "#34d399", "clearance\n[m]",
                     ylim=(-0.6, clr_hi), hline=(0.0, "#ef4444"))
        axc.set_xlabel("time [s]", fontsize=11, color=V.DIM)

        axl = fig.add_subplot(gs[1:, 1]); axl.axis("off")
        axl.legend(handles=V.legend_handles(classes), loc="center left",
                   frameon=False, fontsize=12.5, labelcolor=V.DIM,
                   handletextpad=1.0, labelspacing=0.85)

        fig.savefig(os.path.join(frame_dir, f"f{k:06d}.png"),
                    dpi=DPI, facecolor=V.BG)
        plt.close(fig)

        if k % 100 == 0:
            print(f"      frame {k}/{len(sel)}")

    return len(sel)


# -------------------------------------------------------------- assembly
def normalise_open(src, out):
    """Re-encode the cold open to the submission's frame rate and codec params.

    The concat demuxer copies streams, so every segment has to agree. brag
    renders at 30 fps and the submission runs at 25, so this is not optional.
    """
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", src,
         "-an", "-r", str(FPS), "-vf", f"scale={W}:{H},format=yuv420p",
         "-c:v", "libx264", "-preset", "medium", "-crf", "19", out],
        check=True)
    return out


def build_audio(open_src, open_dur, total_dur, work, out):
    """One continuous audio track: the open's own mix, then a quieter bed.

    The open is already mixed and beat-locked, so it is taken verbatim. Its bed
    fades out at its own tail, and the body's bed fades in from there, so the
    handover lands in a natural gap rather than as a cut.
    """
    open_wav = os.path.join(work, "a_open.wav")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", open_src,
                    "-vn", "-ac", "2", "-ar", "48000", open_wav], check=True)

    body_dur = max(total_dur - open_dur, 0.1)
    body_wav = os.path.join(work, "a_body.wav")
    # Sits well under the body: there is no narration to duck against, and the
    # content is dense, so the bed only has to keep time.
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-stream_loop", "-1",
         "-i", MUSIC, "-t", f"{body_dur:.3f}",
         "-af", (f"afade=t=in:st=0:d=2.0,"
                 f"afade=t=out:st={max(body_dur - 3.0, 0):.3f}:d=3.0,"
                 f"volume=0.16"),
         "-ac", "2", "-ar", "48000", body_wav], check=True)

    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", open_wav, "-i", body_wav,
         "-filter_complex", "[0:a][1:a]concat=n=2:v=0:a=1[a]",
         "-map", "[a]", "-ac", "2", "-ar", "48000", out], check=True)
    return out


def still_segment(png, seconds, out):
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-t", str(seconds),
         "-i", png, "-r", str(FPS),
         "-vf", f"scale={W}:{H},format=yuv420p",
         "-c:v", "libx264", "-preset", "medium", "-crf", "19", out],
        check=True)
    return out


def main(argv):
    wanted = argv[1:] if len(argv) > 1 else ORDER
    runs = []
    for name in wanted:
        p = os.path.join(RESULTS, f"{name}.json")
        if not os.path.exists(p):
            print(f"  skip {name}: no exported run at {p}")
            continue
        d = V.load(p)
        d["name"] = name
        runs.append(d)

    if not runs:
        print("no scenario runs found; run sih_run_all in Octave first")
        return 1

    work = tempfile.mkdtemp(prefix="sih_video_")
    segs = []

    print("cards...")
    cold = find_cold_open()
    open_dur = 0.0
    if cold:
        print(f"  cold open: {os.path.relpath(cold, ROOT)}")
        segs.append(normalise_open(cold, os.path.join(work, "s_open.mp4")))
        open_dur = float(subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "default=nw=1:nk=1", cold],
            capture_output=True, text=True).stdout.strip())
    else:
        print("  no cold open found; using the plain title card")
        segs.append(still_segment(card_title(os.path.join(work, "c_title.png")),
                                  6.0, os.path.join(work, "s00.mp4")))

    segs.append(still_segment(card_problem(os.path.join(work, "c_prob.png")), 11.0,
                              os.path.join(work, "s01.mp4")))
    segs.append(still_segment(card_architecture(os.path.join(work, "c_arch.png")), 12.0,
                              os.path.join(work, "s02.mp4")))

    for i, d in enumerate(runs):
        name = d["name"]
        print(f"scenario {i+1}/{len(runs)}: {name}")
        intro = card_scenario_intro(os.path.join(work, f"c_in{i}.png"),
                                    name, i + 1, len(runs))
        segs.append(still_segment(intro, 5.0, os.path.join(work, f"s1{i}a.mp4")))

        fd = os.path.join(work, f"fr{i}")
        os.makedirs(fd, exist_ok=True)
        n = render_replay(d, fd)
        seg = os.path.join(work, f"s1{i}b.mp4")
        V.encode(fd, seg, fps=FPS, width=W)
        segs.append(seg)
        print(f"      {n} frames -> {os.path.basename(seg)}")

    print("cards...")
    segs.append(still_segment(card_results(os.path.join(work, "c_res.png"), runs),
                              16.0, os.path.join(work, "s90.mp4")))
    segs.append(still_segment(card_honest(os.path.join(work, "c_hon.png")), 14.0,
                              os.path.join(work, "s91.mp4")))
    segs.append(still_segment(card_close(os.path.join(work, "c_end.png")), 5.0,
                              os.path.join(work, "s92.mp4")))

    lst = os.path.join(work, "segments.txt")
    with open(lst, "w", encoding="utf-8") as fh:
        for s in segs:
            fh.write(f"file '{s.replace(os.sep, '/')}'\n")

    out = os.path.join(RESULTS, "SIH26037_submission.mp4")
    silent = os.path.join(work, "video_only.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat",
                    "-safe", "0", "-i", lst, "-c", "copy", silent], check=True)

    total = float(subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=nw=1:nk=1", silent],
        capture_output=True, text=True).stdout.strip())

    if cold and os.path.exists(MUSIC):
        track = build_audio(cold, open_dur, total, work,
                            os.path.join(work, "audio.wav"))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error",
                        "-i", silent, "-i", track,
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                        "-shortest", "-movflags", "+faststart", out], check=True)
    else:
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                        "-c", "copy", "-movflags", "+faststart", out], check=True)

    dur = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=noprint_wrappers=1:nokey=1", out],
        capture_output=True, text=True).stdout.strip()
    size = os.path.getsize(out) / 1e6
    print(f"\nwrote {out}\n  {float(dur):.1f} s, {size:.1f} MB, {W}x{H} @ {FPS} fps")

    shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
