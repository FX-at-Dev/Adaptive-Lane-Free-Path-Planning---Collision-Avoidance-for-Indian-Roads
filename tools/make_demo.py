#!/usr/bin/env python3
"""Generate the self-running live demo page from exported scenario runs.

Writes results/demo.html with the real metrics baked in, so the page cannot
quietly disagree with the runs it is showing. Intended to be opened fullscreen
on a laptop at an evaluation:

    chrome --kiosk --autoplay-policy=no-user-gesture-required \
        --user-data-dir=%TEMP%\\sih_demo results/demo.html

It loops indefinitely and needs no interaction. Each scenario plays its replay
full-bleed, which is the point of the page -- the simulation is the content,
and everything else is a caption over it.

Run tools/render_run.py first so the per-scenario .mp4 files exist.
"""

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")

ORDER = ["village_road", "urban_intersection", "highway_merge",
         "market", "cattle_crossing"]

BLURB = {
    "village_road":       "No centre line. An auto-rickshaw comes head-on down the middle, a cyclist blocks the nearside, a pedestrian crosses, a pushcart narrows the road.",
    "urban_intersection": "No signals, no stop line, no give-way. Traffic crosses from both sides and concedes nothing — right of way is taken, not granted.",
    "highway_merge":      "Joining from a slip road behind a truck and a bus, with two-wheelers filtering past on both sides and nobody opening a gap.",
    "market":             "Stalls and parked carts narrow the road to barely more than the vehicle. Pedestrians step out from behind them with almost no warning.",
    "cattle_crossing":    "Cattle emerge from behind a parked truck at 11 m/s closing speed. They do not hold a line and they do not react to the vehicle.",
}

NICE = {
    "village_road": "Unmarked village road",
    "urban_intersection": "Unsignalled urban crossroads",
    "highway_merge": "Highway merge into slow traffic",
    "market": "Dense market street",
    "cattle_crossing": "Sudden cattle crossing",
}

CSS = """
:root{--bg:#0c0f14;--panel:#141922;--line:#2a3446;--ink:#e8edf5;
      --dim:#93a1b5;--ok:#34d399;--blue:#60a5fa;--amber:#fbbf24;--red:#f87171}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;background:var(--bg);color:var(--ink);overflow:hidden;
  font-family:"Segoe UI",system-ui,sans-serif}
.slide{position:fixed;inset:0;opacity:0;transition:opacity .6s ease;pointer-events:none}
.slide.on{opacity:1}

/* full-bleed scenario video: the simulation IS the page */
.scn video{position:absolute;inset:0;width:100%;height:100%;object-fit:contain;background:#0c0f14}
.topbar{position:absolute;top:0;left:0;right:0;padding:26px 46px;
  background:linear-gradient(180deg,rgba(12,15,20,.94) 0%,rgba(12,15,20,0) 100%);
  display:flex;align-items:baseline;gap:22px;z-index:2}
.num{font-size:17px;letter-spacing:.34em;color:var(--ok)}
.ttl{font-size:38px;font-weight:640;letter-spacing:-.015em}
.botbar{position:absolute;left:0;right:0;bottom:0;padding:34px 46px 30px;
  background:linear-gradient(0deg,rgba(12,15,20,.95) 0%,rgba(12,15,20,0) 100%);z-index:2}
.blurb{font-size:22px;color:var(--dim);line-height:1.5;max-width:1500px}
.chips{display:flex;gap:12px;margin-top:16px;flex-wrap:wrap}
.chip{border:1px solid var(--line);background:rgba(20,25,34,.9);border-radius:999px;
  padding:9px 20px;font-size:17px;color:var(--dim)}
.chip b{color:var(--ink);font-weight:620}
.chip.ok b{color:var(--ok)}

/* title + results */
.mid{display:flex;flex-direction:column;align-items:center;justify-content:center;
  height:100%;gap:26px;padding:60px}
.eyebrow{letter-spacing:.42em;font-size:18px;color:var(--ok);text-transform:uppercase}
h1{font-size:74px;font-weight:660;letter-spacing:-.025em;text-align:center;line-height:1.08}
.sub{font-size:26px;color:var(--dim);text-align:center;max-width:1180px;line-height:1.5}
.tags{display:flex;gap:13px;flex-wrap:wrap;justify-content:center;margin-top:8px}
.tag{border:1px solid var(--line);background:var(--panel);border-radius:999px;
  padding:10px 22px;font-size:17px;color:var(--dim)}
table{border-collapse:collapse;font-size:21px}
th{font-size:14px;letter-spacing:.14em;color:#6b7a91;text-transform:uppercase;
  text-align:center;padding:0 24px 16px}
th:first-child,td:first-child{text-align:left}
td{padding:13px 24px;text-align:center;border-top:1px solid var(--line)}
.big{font-size:34px;font-weight:660;color:var(--ok);margin-top:26px}
.foot{position:fixed;right:34px;bottom:22px;font-size:15px;color:#55627a;letter-spacing:.05em;z-index:3}
.prog{position:fixed;left:0;bottom:0;height:3px;background:var(--ok);width:0;z-index:4}
"""


def load_runs():
    runs = []
    for name in ORDER:
        p = os.path.join(RESULTS, f"{name}.json")
        mp4 = os.path.join(RESULTS, f"{name}.mp4")
        if not os.path.exists(p):
            print(f"  skip {name}: no {name}.json")
            continue
        with open(p, encoding="utf-8") as fh:
            d = json.load(fh)
        d["name"] = name
        d["has_video"] = os.path.exists(mp4)
        if not d["has_video"]:
            print(f"  note {name}: no {name}.mp4 (run render_run.py)")
        runs.append(d)
    return runs


def chips(d):
    m, r = d["metrics"], d["result"]
    out = []
    out.append(f'<div class="chip ok">collisions <b>{0 if not r["collided"] else 1}</b></div>')
    out.append(f'<div class="chip">min clearance <b>{m["min_clear"]:.2f} m</b></div>')
    t = m.get("t_to_goal")
    if isinstance(t, (int, float)):
        out.append(f'<div class="chip">goal reached <b>{t:.1f} s</b></div>')
    out.append(f'<div class="chip">replan p95 <b>{m["latency_p95_ms"]:.0f} ms</b></div>')
    out.append(f'<div class="chip">peak curvature <b>{m["curv_max"]:.3f} 1/m</b></div>')
    return "".join(out)


def build(runs):
    slides, seq = [], []

    slides.append(f"""<div class="slide" id="s0"><div class="mid">
  <div class="eyebrow">Smart India Hackathon &middot; Problem Statement 26037 &middot; MathWorks</div>
  <h1>Adaptive Path Planning &amp;<br>Collision Avoidance</h1>
  <div class="sub">A closed-loop autonomous-driving stack for <b>unstructured Indian roads</b> &mdash;
    no lane markings, mixed traffic, and road users that do not follow rules.</div>
  <div class="tags">
    <div class="tag">Multi-sensor fusion</div><div class="tag">Multi-hypothesis prediction</div>
    <div class="tag">Frenet lattice planner</div><div class="tag">Behaviour state machine</div>
    <div class="tag">Kinematic bicycle model</div>
  </div></div></div>""")
    seq.append(("s0", 7000, None))

    for i, d in enumerate(runs):
        name = d["name"]
        vid = (f'<video id="v{i}" src="{name}.mp4" muted playsinline></video>'
               if d["has_video"] else
               '<div class="mid"><div class="sub">replay not rendered</div></div>')
        slides.append(f"""<div class="slide scn" id="s{i+1}">
  {vid}
  <div class="topbar"><div class="num">SCENARIO {i+1} OF {len(runs)}</div>
    <div class="ttl">{NICE.get(name, name)}</div></div>
  <div class="botbar"><div class="blurb">{BLURB.get(name,'')}</div>
    <div class="chips">{chips(d)}</div></div></div>""")
        # Long enough for the clip to play out; the replays run about 26 s.
        seq.append((f"s{i+1}", 28000, f"v{i}" if d["has_video"] else None))

    n_done = sum(1 for d in runs if d["result"]["reached"])
    n_coll = sum(1 for d in runs if d["result"]["collided"])
    rows = "".join(
        f"<tr><td>{NICE.get(d['name'], d['name'])}</td>"
        f"<td style=\"color:{'#34d399' if d['result']['reached'] else '#f87171'}\">"
        f"{'reached' if d['result']['reached'] else 'not reached'}</td>"
        f"<td style=\"color:{'#34d399' if not d['result']['collided'] else '#f87171'}\">"
        f"{0 if not d['result']['collided'] else 1}</td>"
        f"<td>{d['metrics']['min_clear']:.2f} m</td>"
        f"<td>{d['metrics']['latency_p95_ms']:.0f} ms</td>"
        f"<td>{d['metrics']['curv_max']:.3f}</td></tr>"
        for d in runs)

    slides.append(f"""<div class="slide" id="sr"><div class="mid">
  <h1 style="font-size:50px">Results</h1>
  <div class="sub" style="font-size:20px">Clearance and collisions are measured against ground truth,
    not the vehicle&rsquo;s own tracks.</div>
  <table><tr><th>scenario</th><th>goal</th><th>collisions</th>
    <th>min clearance</th><th>replan p95</th><th>peak curvature</th></tr>{rows}</table>
  <div class="big">{n_done} of {len(runs)} scenarios completed &middot; {n_coll} collisions</div>
</div></div>""")
    seq.append(("sr", 15000, None))

    js_seq = ",".join(
        f'{{id:"{sid}",ms:{ms},vid:{("null" if v is None else chr(34)+v+chr(34))}}}'
        for sid, ms, v in seq)

    return f"""<meta charset="utf-8">
<title>SIH 26037 &mdash; Adaptive Path Planning</title>
<style>{CSS}</style>
{''.join(slides)}
<div class="foot">SIH 26037 &middot; {len(runs)} scenarios &middot; closed-loop simulation</div>
<div class="prog" id="prog"></div>
<script>
// Generated by tools/make_demo.py -- do not edit by hand; the metrics above
// are baked in from the exported runs so the page cannot disagree with them.
const seq=[{js_seq}];
const total=seq.reduce((a,s)=>a+s.ms,0);
const prog=document.getElementById('prog');
let cycle=Date.now();
function show(i){{
  document.querySelectorAll('.slide').forEach(e=>e.classList.remove('on'));
  const s=seq[i];
  document.getElementById(s.id).classList.add('on');
  if(s.vid){{ const v=document.getElementById(s.vid);
    if(v){{ try{{ v.currentTime=0; v.play(); }}catch(e){{}} }} }}
  setTimeout(()=>{{ if(i+1<seq.length){{ show(i+1); }}
                   else {{ cycle=Date.now(); show(0); }} }}, s.ms);
}}
setInterval(()=>{{ prog.style.width=Math.min(100,(Date.now()-cycle)/total*100)+'%'; }},100);
setTimeout(()=>show(0),1000);
</script>
"""


def main():
    runs = load_runs()
    if not runs:
        print("no exported runs found; run sih_run_all in Octave first")
        return 1
    out = os.path.join(RESULTS, "demo.html")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(build(runs))
    print(f"wrote {out}  ({len(runs)} scenarios)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
