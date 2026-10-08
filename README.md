# SIH 26037 — Adaptive Path Planning for Unstructured Indian Roads

SIH 26037 is a reproducible, closed-loop autonomous-driving simulation for unmarked,
mixed-traffic Indian roads. The stack combines perception, multi-sensor fusion,
multi-hypothesis prediction, lattice planning, behaviour arbitration, and vehicle
dynamics to keep an ego vehicle safe while it negotiates everyday road situations.

The core implementation is written in portable MATLAB/GNU Octave code. Python tools
turn exported simulation runs into figures, previews, demos, and the submission video.
An optional Unity 6 project provides a live 3D world with simulated LiDAR, radar, and
camera sensors.

**Status: all five required scenarios run collision-free.** The Simulink model, Stateflow
chart, RoadRunner scenes and the IDD detection demo are not built yet — see *Roadmap*.

## Results

| scenario | goal | collisions | min clearance | time | mean speed | replan p95 |
|---|---|---|---|---|---|---|
| unmarked village road | reached | 0 | 1.05 m | 43.1 s | 4.41 m/s | 30 ms |
| unsignalled crossroads | reached | 0 | 2.69 m | 26.1 s | 6.06 m/s | 37 ms |
| highway merge | reached | 0 | 3.89 m | 31.2 s | 9.16 m/s | 30 ms |
| dense market street | reached | 0 | 0.40 m | 90.5 s | 1.68 m/s | 33 ms |
| sudden cattle crossing | reached | 0 | 0.28 m | 55.9 s | 4.11 m/s | 34 ms |

**5 of 5 completed, 0 collisions,** keeping to the left lane and coming to rest at the target
(time includes the stop). Clearance and collisions are measured against **ground
truth**, not the vehicle's own tracks — measuring safety from perception would be circular,
because a tracker that lost an agent would report comfortable clearance right up to impact.

Replanning latency is wall-clock under an interpreter on a laptop: a measure of relative
planner cost, not an embedded real-time claim. It was measured with Windows power throttling
off for the Octave process: left on, Windows can run a windowless process on the slow
efficiency cores of a hybrid CPU, which made every step about three times slower.

**Every tracked object is a footprint** -- real centre, long-axis orientation and size, learnt
from the LiDAR's boxes (`sih_track_shape`) -- not a class-sized box turned by the filter's
heading, which for a parked object is noise. The LiDAR box is fitted to the outline of the
returns (L-shape fitting), not their principal axis, which for the L a LiDAR sees of a car is
its diagonal and swung as the car drove past. A box gives the orientation only up to a quarter
turn, so the established axis, the direction of travel or the road decides which side is long.
Several LiDAR pieces lying in one tracked object (a bus side at a grazing angle, a stall's
posts) are merged into one box before association. A track counts only once a LiDAR, or both
the camera and the radar, has seen it. "In the path" is measured along the road
(Frenet), not along the car's straight-ahead axis. **Random worlds** (each scenario's story plus random everyday traffic, `sih_scn_*(cfg, seed)`;
seeds 11, 23 and 101 for each scenario): 0 collisions in 15; 14 reach the target -- all but
village road world 11, where the car waits beside a parked bus whose tracked heading is 12°
out (see *Known limitations*). Four further worlds (market 376837, village road 13901 and
152526, cattle crossing 849169) all reach it, with 0 collisions. The market's time swings with
timing alone: whether a crossing pedestrian stops in front of the car or behind it.

## Setup and quick start

### Prerequisites

Required for the algorithm and tests:

- GNU Octave 7 or newer, or MATLAB R2020b or newer
- Git

Required only for the Python rendering tools:

- Python 3.8 or newer
- `matplotlib` and `numpy`
- `ffmpeg` on `PATH` for MP4 output

The core simulation and tests do not require a MATLAB toolbox or Python. Unity live
simulation additionally requires Unity 6 (6000.6) and the project under `unity/SIH3D`.

Clone the repository, then run commands from its root directory so `startup.m` can
resolve all source folders:

```powershell
git clone <repository-url>
cd Adaptive-Lane-Free-Path-Planning---Collision-Avoidance-for-Indian-Roads
```

If Python rendering is needed, install its two Python packages in a virtual
environment or user environment:

```powershell
py -m pip install matplotlib numpy
```

### Run the tests

```bash
octave-cli --eval "startup; sih_test_all"
```

The test command runs the numeric unit tests and the closed-loop regression checks.
In MATLAB, open the repository root and run `startup; sih_test_all` in the Command
Window instead.

### Run all scenarios

```bash
octave-cli --eval "startup; sih_run_all"
```

The runner executes all five scenarios with their configured seeds and writes the
exported metrics to `results/*.json`. To run one scenario interactively, call
`sih_run_scenario` after `startup` from the Octave or MATLAB Command Window.

### Render results (optional)

```bash
python tools/make_video.py
```

```bash
python tools/render_run.py results/village_road.json && python tools/make_demo.py
```

`make_video.py` builds the 1080p submission video; `render_run.py` makes per-scenario
previews and `make_demo.py` builds a self-running fullscreen page for a laptop. Render
tools consume the JSON files produced by `sih_run_all`, so regenerate the results before
rendering if the algorithm or scenario configuration has changed.

Rendering lives in Python because the only Octave graphics toolkit available here requires a
display. Simulation and rendering communicate through the exported JSON, so the pictures
always match the run the metrics came from.

## 3D live demo (Unity)

`unity/SIH3D` is a Unity 6 (URP) project in which the stack drives a car live. Octave stays
the brain; Unity is the world, the sensors and the vehicle.

```
seed ─► Octave traffic (scenario roles + road rules) ─┐     Unity scenery (prefab catalog + rules)
                                                       ▼                    │ static map
Unity world ─► LiDAR / radar / camera (SihSensorRig) ─► core/perception ─► tracker (motion + shape)
            ─► sih_footprint ─► risk (in-path along the road) ─► predict ─► plan ─► control
            ─► Unity plant (SihEgoPlant, a port of sih_bicycle_step) ─► the car moves
```

**Running it.** Open `unity/SIH3D` in Unity 6000.6, run *SIH › Create Scenes* (or *SIH › Add
Live Link to Scenes* to upgrade scenes you have edited, then save them), open a scene from
`Assets/Scenes` and press Play. Without the Live Link a scene only replays the recorded run in
`results/3d`, which is the same every time. Move the scene's *Goal* flag to choose where the
car stops. The Live Link starts `octave-cli` itself (`cosim/sih_cosim_serve.m`) and keeps it
running between Plays. Every setting is an Inspector field; untick *Use Live* on the Live Link to
replay a recorded run instead.

**A new world every Play.** The Live Link's *Seed* 0 draws a fresh world seed each Play; any
other number gives that world again exactly. The seed drives the stack's random traffic
(`scenarios/sih_scn_*.m` take a seed and keep the scenario's story, and `sih_scn_traffic` adds
the everyday traffic of an Indian road around it -- oncoming vehicles in their lane, cycles and
carts ahead, vehicles at the kerb, people on the verge, cattle at the roadside;
`sih_scn_validate` rejects layouts that break the rules of the road). Every road user is there
from the first step, most of it well ahead, for the car to find with its sensors; nothing
appears in front of it. Unity's scenery generator (houses, walls, trees,
poles, fields, cones, barriers, signs, milestones from `Assets/SIH/Catalog.asset`). The
generator has *Preview layout / Keep this layout / Clear* in edit mode.

**Driving through your own script.** The Live Link's *Launcher* is *Python Script* by default:
Play starts `main.py` (with the `python` on the PATH, or *Python Exe*), not Octave; set it to
*Octave* to skip the script, and if Python cannot be started Play falls back to Octave with a
warning in the Console. `main.py` starts the stack itself and sits between
Unity and it: every sensor message and every command passes through it, and each step the
command goes through its `drive()` function before the car gets it. By default `drive()` passes
the stack's command on unchanged and prints what the car is doing; set `SPEED_LIMIT`, or
change `drive()`, and the car in Unity does what your script says (the stack plans from where
the car really is). The HUD then reads *the stack is driving, via main.py*. **R** starts a new
world; a run in which the car has not moved for 20 s starts a new one by itself. If the stack goes away mid-Play (it crashed, or was closed), Unity says why in the Console and
starts it again after 5 s (*Relaunch After* on the Live Link).

**Seeing what the car sees.** *Car View* (picture in picture, **G** full screen) is a camera at
the car's sensor mount. Every tracked object is drawn in the world as the footprint the stack
decided with -- its centre, its long axis, its size -- so the boxes sit on the objects in both
views. **I** inspects the object under the pointer; **B/P/K** drop a cow, a pedestrian or a cart
into the live run; **1–9** toggle the overlays.

**Results with Unity's sensors** (scripted traffic, `LivePlay` in a copy of the project):

| scenario | goal | collisions | time | LiDAR: road users found / ghosts |
|---|---|---|---|---|
| village road | reached | 0 | 46.2 s | 92% / 0% |
| crossroads | reached | 0 | 44.7 s | 72% / 41%* |
| highway merge | reached | 0 | 31.1 s | 100% / 0% |
| market street | reached | 0 | 75.3 s | 97% / 0% |
| cattle crossing | **timed out** at 100 s | 0 | — | 95% / 15%* |

Driven through `main.py` with Unity's sensors and random traffic: village road world 13901
reached in 55.8 s and crossroads world 23 in 44.3 s, 0 collisions, step latency p95 42-47 ms.

\* "Ghosts" counts detections more than 2.5 m from any road user's centre, which includes the
ends of an 11 m bus or a lorry. A live run with an exact plant reproduces the offline run to
1e-13 m (*SIH › Check Live Link*).

## Design

**The vehicle keeps to its lane, and leaves it when it has to.** India drives on the left:
on a road at least 5 m wide the planner holds the centre of the left lane
(`sih_lane_centre`), so oncoming traffic has its side and a stop -- for a cow, in a queue --
does not block the road. A single-lane road is driven down the middle. The lattice still
samples the whole carriageway, and while easing past something in the way (NUDGE) the lane
preference is relaxed to a sixth, so the car pulls out early round a car parked at the kerb
instead of stopping behind it with no room to steer out.

**It slows for what it is not sure of, and stops only for what is there.** A new object in
the path may be a ghost -- a reflection, a camera box with a range metres out, a stray LiDAR
cluster -- so until it has been tracked for 0.6 s and (within the LiDAR's reach) the LiDAR has
seen it, it only caps the speed at what a gentle 1.5 m/s² brake stops from short of it; the
planner and the emergency stop ignore it. A ghost is gone by then and the car is back at cruise; a real
object is confirmed while there is still room to stop. In the 3D view such an object is amber,
tagged *(checking)*.

**It keeps room to pull out, and backs out of a box.** Behind something standing in its way
-- a cart, a stall, a parked lorry -- no planned path may end within 5 m of it while still in
line with it, so the car stops where it can still steer round it, not a metre behind it. If it
does end up boxed in (stopped 3 s with no way forward and something standing close in front),
it backs up 3 m at walking pace, steering towards the road's heading angled 11° to the open
side (so a second back-up straightens it rather than turning it further across the road), if
the road behind is clear, and plans again (`sih_reverse`; the bicycle model and its Unity port
have a reverse gear). A short time to collision with something that is not coming towards it
is no emergency while the car can still brake short of it, so pulling out at walking pace from
2 m behind a parked lorry does not slam the brakes on.

**It stops where it is told to, and nowhere else.** The target (the *Goal* flag in a Unity
scene, or the scenario's goal) caps the speed at what a gentle 0.8 m/s² stops from at the
target (`sih_arrive`); the run counts as arrived once the car is stopped within 3 m of it.
Every other stop is the behaviour layer's, for something actually in the way.

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
core/       toolbox-free algorithms (world, sensors, perception, fusion, predict, plan, decide, vehicle)
scenarios/  the five scenarios, their random variants (seed argument), validation; opendrive/ (Phase 3)
sim/        closed-loop runner and shared stack step, ground-truth safety check, batch runner, JSON export
cosim/      the Unity co-simulation server (sih_cosim_serve) and its live-vs-offline check
unity/      SIH3D, the Unity 6 project: live 3D demo, sensors, Car View, generated worlds
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
- **Dense random worlds can still hold the car up.** It backs out of a box (`sih_reverse`) and
  traffic now goes round a stopped car, which cleared nearly all of the long waits. What is left
  comes from perception: in village road world 11 a parked bus is tracked with its heading 12°
  out, so its near corner reads as in front of the nose and the car, rightly, will not drive into
  it; it waits about 90 s, without contact, before it gets past. A tracker that refines a long
  vehicle's heading as the car draws alongside would remove it.
- **Surrounding traffic is a reactive model, not a planner.** Agents brake for the ego, steer
  round it and never step into it, but do not plan. Adequate for exercising the ego; not a
  traffic model in itself.
- **Latency** is p95 22–36 ms per replan at full speed, but roughly three times that if
  Windows throttles the Octave process (the Unity Live Link turns throttling off itself).

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
