# SIH 26037 — Adaptive Path Planning for Unstructured Indian Roads

Closed-loop simulation of an autonomous vehicle on unmarked, mixed-traffic Indian roads:
perception → multi-sensor fusion → multi-hypothesis prediction → lattice planning →
behaviour arbitration → vehicle dynamics.

**Status: all five required scenarios run collision-free.** The Simulink model, Stateflow
chart, RoadRunner scenes and the IDD detection demo are not built yet — see *Roadmap*.

## Results

| scenario | goal | collisions | min clearance | time | mean speed | replan p95 |
|---|---|---|---|---|---|---|
| unmarked village road | reached | 0 | 0.46 m | 48.1 s | 3.91 m/s | 95 ms |
| unsignalled crossroads | reached | 0 | 3.33 m | 43.5 s | 3.59 m/s | 125 ms |
| highway merge | reached | 0 | 3.89 m | 38.0 s | 7.43 m/s | 71 ms |
| dense market street | reached | 0 | 0.13 m | 97.1 s | 1.54 m/s | 80 ms |
| sudden cattle crossing | reached | 0 | 1.47 m | 84.2 s | 2.69 m/s | 85 ms |

**5 of 5 completed, 0 collisions.** Clearance and collisions are measured against **ground
truth**, not the vehicle's own tracks — measuring safety from perception would be circular,
because a tracker that lost an agent would report comfortable clearance right up to impact.

Replanning latency is wall-clock under an interpreter on a laptop: a measure of relative
planner cost, not an embedded real-time claim. It sits near the 100 ms budget and crosses it
on the busiest cycles.

## Running it

The algorithm core is toolbox-free and runs under GNU Octave, so none of this needs a MATLAB
licence yet.

```bash
octave-cli --eval "startup; sih_test_all"
```

```bash
octave-cli --eval "startup; sih_run_all"
```

```bash
python tools/make_video.py
```

```bash
python tools/render_run.py results/village_road.json && python tools/make_demo.py
```

`sih_run_all` runs every scenario with its own configuration and writes `results/*.json`.
`make_video.py` builds the 1080p submission video; `render_run.py` makes per-scenario
previews and `make_demo.py` builds a self-running fullscreen page for a laptop.

Rendering lives in Python because the only Octave graphics toolkit available here requires a
display. Simulation and rendering communicate through the exported JSON, so the pictures
always match the run the metrics came from.

## Design

**The reference path is a drivable corridor, not a lane.** There are no markings to hold, so
the planner samples lateral offsets across the whole carriageway and only mildly prefers the
centre. Passing an oncoming rickshaw on its own side of the road is ordinary driving here,
not a violation to be penalised.

**Lateral motion is parameterised by arc length, not time.** This is the single most
important detail in the planner. A time-parameterised lateral polynomial asks the vehicle to
be a certain distance across the road after a certain *time*, regardless of how far it has
travelled — which from low speed means moving sideways before moving forward, needing a 0.9 m
turning radius where the steering gives 4.4 m. Tying lateral displacement to longitudinal
progress keeps curvature inside what the steering can actually do.

**Prediction is multi-hypothesis and class-conditioned.** Each tracked agent emits weighted
"carry on / brake / drift left / drift right" hypotheses, weighted by class: a bus puts
almost all its probability on carrying on, cattle spread it nearly evenly. Conflict risk is
discounted by how imminent it is, because a conflict at the far end of a 4 s horizon will be
replanned against forty more times before it can happen.

**The behaviour layer emits parameters, not controls** — a speed cap, lateral freedom and a
risk tolerance — so the planner stays one optimiser rather than a pile of special cases, and
the state machine stays small enough to transcribe into a legible Stateflow chart.

## Layout

```
core/       toolbox-free algorithms (world, sensors, fusion, predict, plan, decide, vehicle)
scenarios/  the five scenarios; opendrive/ for the .xodr scenes (Phase 3)
sim/        closed-loop runner, ground-truth safety check, batch runner, JSON export
tools/      Python rendering: sihviz (shared), render_run, make_video, make_demo
tests/      numeric unit tests plus a full closed-loop regression test
results/    generated JSON, videos, figures
```

## Portability

Written for both MATLAB and Octave: functions and structs only, no `classdef`, no
`arguments` blocks, no string arrays, no `+package` directories. Toolbox calls are guarded
and have hand-rolled fallbacks. `startup.m` reports which capability tier is available, so a
run that silently lost a toolbox is obvious before it produces misleading results.

`sih_gradient` and `sih_gradient_cols` exist purely for interpreter speed and are numerically
identical to the built-ins they replace — Octave's `gradient` was 77% of a planning cycle.
Batching candidate generation and collision checking took the planner from 919 ms to 61 ms
per cycle.

## Known limitations

- **Hybrid A\* fallback is implemented but disabled** (`cfg.plan.hybrid.enable = false`). It
  works in isolation — given a boxed-in state it finds escape paths of around five metres —
  but enabling it end to end raised p95 planner latency from about 80 ms to 250–400 ms, cost
  the urban intersection its completion, and, planning to a thinner margin than the lattice,
  produced a contact on a scenario that had been clean. Turning it on is a one-line change
  once a search cycle is cheaper and its margin handling is reconciled with the lattice's.
- **The scenarios are tuned to be demanding but passable.** Earlier versions were physically
  impassable in places — a pedestrian finishing a crossing mid-carriageway, stalls leaving a
  gap narrower than the vehicle — which reads as a planner failure while measuring nothing.
  Each scenario file documents what was eased and why.
- **The market clears by 0.13 m at its tightest.** Real, but slim; it is the scenario most
  sensitive to margin tuning.
- **Surrounding traffic is a reactive model, not a planner.** Agents brake and steer around
  the ego but do not plan. Adequate for exercising the ego; not a traffic model in itself.
- **Latency crosses the 100 ms budget on the busiest cycles** (p95 71–125 ms by scenario).

## Roadmap

| Phase | Scope | Tier |
|---|---|---|
| 1 ✅ | Core loop + village road | Octave, free |
| 2 ✅ | All five scenarios, batch metrics, video pipeline | Octave, free |
| 3 | Simulink model (`basic` variant) | MATLAB Online Basic, free |
| 4 | Technical report, HTML dashboard | free |
| 5 | Stateflow, ADT/Navigation blocks, RoadRunner scenes, YOLO on IDD | 30-day trial |

MATLAB Online Basic includes Simulink and nine toolboxes, so the Simulink model costs no
trial days. The trial is the only non-renewable resource and is spent last, on what actually
needs it.
