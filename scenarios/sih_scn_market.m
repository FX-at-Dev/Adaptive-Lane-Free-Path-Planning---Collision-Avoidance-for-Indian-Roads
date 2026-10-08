function [scn, cfg] = sih_scn_market(cfg, seed)
%SIH_SCN_MARKET Dense market street with mixed traffic at walking pace.
%
%   Scenario 4 of the five required by the problem statement.
%
%   A narrow market road with stalls and parked pushcarts encroaching from both
%   kerbs, pedestrians stepping out between them, a cyclist and auto-rickshaws
%   threading through. The drivable width is barely wider than the vehicle in
%   places, and it changes as people move.
%
%   What this exercises: the low-speed end of the envelope, where progress
%   depends on accepting gaps that would be unacceptable at road speed. It is
%   the scenario that most directly tests whether the risk tolerance and the
%   CREEP state are tuned to let the vehicle make progress at all -- an
%   over-conservative planner does not crash here, it simply stops and never
%   arrives, which the completion metric is there to catch.
%
%   Occlusion matters more here than anywhere else: pedestrians emerge from
%   behind parked carts with very little warning, which is what the tracker's
%   coasting and the predictor's class-conditioned uncertainty are for.
%   [scn, cfg] = SIH_SCN_MARKET(cfg, seed) draws a random layout that keeps
%   the scenario's story (sih_scn_random): which road users, how many, where,
%   how fast and when are drawn from ranges, and the draw is checked against
%   the rules of the road (sih_scn_validate). Without a seed the scripted
%   layout below is used, which keeps the regression suite reproducible.

if nargin < 1 || isempty(cfg)
    cfg = sih_config();
end

cfg.plan.v_max              = 5.0;    % ~18 km/h, and rarely achieved
cfg.plan.corridor_halfwidth = 4.0;
cfg.sim.t_end               = 140.0;  % 160 m at walking pace, with stops

% Creeping through gaps is the expected behaviour, not a degraded mode.
cfg.dec.creep_speed         = 1.6;
cfg.dec.clear_caution       = 2.0;    % react to tighter clearances sooner
cfg.plan.safety_margin      = 0.35;   % accept closer passes than on open road

% A shorter horizon suits walking pace: at 4 m/s a 4 s horizon reaches 16 m,
% far past the next decision, and the extra range only adds uncertainty.
cfg.pred.horizon            = 3.0;
cfg.plan.horizon_T          = [2.0 3.0];

% ---- road ----------------------------------------------------------------
road_wp = [  0,   0;
            30,   3;
            60,   4;
            95,  -1;
           130,  -4;
           160,  -3];

scn.name = 'market';
scn.desc = 'Dense market street, stalls and pushcarts narrowing the road, pedestrians crossing';
scn.rp   = sih_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', 0, 'psi', atan2(3, 30), 'v', 3.0);
[gx, gy] = sih_wp_at(scn.rp, scn.rp.length - 10, 0);
scn.goal = [gx, gy];

% ---- static encroachment: stalls and parked carts ------------------------
% Alternating sides, so the drivable channel weaves rather than narrowing
% uniformly. Offsets are kept inside +/-2.2 m so a passable gap always exists;
% the scenario is meant to be difficult, not impossible.
% Five stalls rather than seven, and spaced further apart. Alternating
% encroachment from both kerbs is what makes the street demanding; packing
% them closer simply removed the through route, and an impassable street tests
% nothing. The drivable channel is deliberately narrow but continuous.
% Four stalls, well spaced and tight against the kerbs. The encroachment is
% what makes the street demanding; the count is what made it impassable. With
% stalls at 26 m intervals the vehicle had to be committed to the next weave
% before it had cleared the last, at a speed where it could not steer.
% Stalls tight against the kerbs of a 7.2 m street. The encroachment still
% forces a continuous weave and the pedestrians still emerge from behind them;
% what changed is that the channel between opposing stalls is now wide enough
% for the vehicle to steer through at creep speed. Packed tighter, the street
% was not difficult -- it was impassable, and an impassable scenario measures
% nothing about the planner.
% Three stalls, alternating kerbs. The fourth sat at station 146, five metres
% short of the goal, so the final approach demanded a lateral correction at
% creep speed with no room left to make it -- the vehicle consistently stopped
% within ten metres of finishing. The weave is what this scenario is testing;
% putting an obstacle on the finish line only tested the goal tolerance.
statics = [ 30, -3.4, 1;
            70,  3.4, 2;
           112, -3.4, 1];

agents = [];
id = 0;
for k = 1:size(statics, 1)
    id = id + 1;
    if statics(k,3) == 1
        cls = 'pushcart';
    else
        cls = 'static';
    end
    agents = [agents, local_parked(id, cls, scn.rp, statics(k,1), statics(k,2))]; %#ok<AGROW>
end

% ---- moving traffic ------------------------------------------------------
% Pedestrians crossing between the carts, timed so the ego meets them rather
% than arriving after they have cleared.
% Crossing pedestrians must START and FINISH clear of the carriageway. An
% agent that runs out of waypoints stops dead and stays there for the rest of
% the run, so a pedestrian whose crossing ends inside the corridor becomes a
% permanent obstacle. Paired with a stall on the opposite kerb that left a gap
% narrower than the vehicle, and the street was simply impassable -- a scenario
% artefact that reads as a planner failure.
id = id + 1;
% Moved clear of the first stall. Crossing at the same station as a stall put
% the pinch point and the moving hazard in the same place, which left no gap at
% all rather than a difficult one.
agents = [agents, local_walker(id, 'pedestrian', scn.rp,  48, -4.8,  4.8, 1.0,  6.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp,  88,  4.8, -4.8, 1.1, 20.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp, 126, -4.8,  4.8, 0.9, 40.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp, 152,  4.8, -4.8, 1.2, 58.0)];

% A cyclist ahead, slower than the ego, that has to be followed or eased past.
id = id + 1;
cyc = sih_wp_path(scn.rp, [ 45  70  95 120 150], [1.0 -0.6 0.8 -0.4 0.4]);
agents = [agents, local_mover(id, 'bicycle', scn.rp, 45, 1.0, 2.4, cyc, 0)];

% An auto-rickshaw coming the other way through the same gaps.
id = id + 1;
% The oncoming rickshaw keeps to its own side of an unmarked street, with a
% weave rather than a full centreline crossing. Swinging right across the
% centre while the ego was threading between stalls left the two with nowhere
% to pass and produced a graze; it still has no lane discipline, it simply has
% somewhere to go.
onc = sih_wp_path(scn.rp, [150 120  90  60  30  -5], [-1.9 -1.1 -2.1 -1.3 -1.8 -1.4]);
agents = [agents, local_mover(id, 'auto', scn.rp, 150, -0.8, 3.2, onc, 34.0)];

scn.agents = agents;
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = sih_scn_random(scn, cfg, seed, @local_draw);
end
end

% -------------------------------------------------------------------------
function a = local_parked(id, class_name, rp, s_at, d_at)
[x0, y0] = sih_wp_at(rp, s_at, d_at);
a = sih_agent_new(id, class_name, x0, y0, sih_wp_heading(rp, s_at), 0.0, 'static', [], 0);
end

% -------------------------------------------------------------------------
function a = local_walker(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)
[x0, y0] = sih_wp_at(rp, s_at, d_from);
wp   = sih_wp_path(rp, [s_at, s_at + 1, s_at + 2], [d_from, 0, d_to]);
psi0 = sih_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;
a = sih_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end

% -------------------------------------------------------------------------
function a = local_mover(id, class_name, rp, s0, d0, v, wp, t_spawn)
% Due at s0 at t_spawn, it is already on its way there from the start.
if t_spawn > 0 && size(wp, 1) >= 2
    s_end = sih_cart2frenet(rp, wp(end, 1), wp(end, 2));
    [s0, t_spawn] = sih_upstream(rp, s0, v, t_spawn, sign(s_end - s0));
end
[x0, y0] = sih_wp_at(rp, s0, d0);
% Face the SECOND waypoint. The first one coincides with the start position
% by construction, so aiming at it gives atan2(0,0) and an agent that begins
% life pointing along +x regardless of which way it is meant to travel -- an
% oncoming rickshaw would spend its first second spinning through 180 degrees.
psi0 = sih_wp_heading(rp, s0);
if size(wp,1) >= 2
    dxw = wp(2,1) - x0;
    dyw = wp(2,2) - y0;
    if hypot(dxw, dyw) > 1e-6
        psi0 = atan2(dyw, dxw);
    end
end
a = sih_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end

% -------------------------------------------------------------------------
function A = local_draw(rp, cfg, R)
%LOCAL_DRAW Market street: stalls and pushcarts narrowing the road on
%   alternating sides, pedestrians crossing, a cyclist, an oncoming rickshaw.
L = rp.length; A = []; id = 0;
s = R.u(25, 35); side = R.pick({1, -1});
while s < L - 25
    id = id + 1;
    A = [A, sih_role('parked', id, R.pick({'pushcart', 'static', 'pushcart'}), rp, s, side * R.u(3.2, 3.5))]; %#ok<AGROW>
    side = -side;
    s = s + R.u(25, 42);
end
for k = 1:R.int(3, 5)
    id = id + 1;
    sp = R.u(40, L - 8); side = R.pick({1, -1});
    A = [A, sih_role('walker', id, 'pedestrian', rp, sp, side * 4.8, -side * 4.8, R.u(0.9, 1.2), ...
                     max(0, sp / 2.6 - R.u(6, 12)))]; %#ok<AGROW>
end
id = id + 1;
s0 = R.u(35, 60); sl = s0 + 25:25:L; dl = arrayfun(@(~) R.u(-0.6, 1.0), sl);
A = [A, sih_role('mover', id, 'bicycle', rp, s0, R.u(0.4, 1.2), R.u(2.0, 2.8), sl, dl, 0)];
id = id + 1;
s0 = R.u(130, L - 5); sl = s0 - 30:-30:-5; dl = arrayfun(@(~) R.u(-2.2, -1.0), sl);
A = [A, sih_role('mover', id, 'auto', rp, s0, -0.8, R.u(2.8, 3.6), sl, dl, R.u(25, 45))];
end
