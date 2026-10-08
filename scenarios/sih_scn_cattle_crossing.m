function [scn, cfg] = sih_scn_cattle_crossing(cfg, seed)
%SIH_SCN_CATTLE_CROSSING Cattle stepping into the road at speed, with no warning.
%
%   Scenario 5 of the five required by the problem statement.
%
%   The ego is running at close to its cruising speed on an open rural road
%   when cattle emerge from the left verge, partly screened by a parked truck,
%   and walk across. They do not hurry, they do not hold a line, and they do
%   not react to the vehicle.
%
%   What this exercises, and why it is the sharpest test of the prediction
%   layer: cattle carry the highest 'erratic' weighting of any class, so
%   sih_predict_intent spreads their probability mass almost evenly over
%   carry-on, stop, and drift either way. The planner therefore sees a wide,
%   diffuse occupancy band rather than a trajectory, and has to shed speed
%   early rather than commit to threading a gap that may close. A
%   constant-velocity predictor would produce a crisp, confident forecast here
%   and would be confidently wrong.
%
%   The event is deliberately time-triggered rather than present from the
%   start, so the run measures reaction and replanning rather than route
%   planning around a known obstacle.
%   [scn, cfg] = SIH_SCN_CATTLE_CROSSING(cfg, seed) draws a random layout that keeps
%   the scenario's story (sih_scn_random): which road users, how many, where,
%   how fast and when are drawn from ranges, and the draw is checked against
%   the rules of the road (sih_scn_validate). Without a seed the scripted
%   layout below is used, which keeps the regression suite reproducible.

if nargin < 1 || isempty(cfg)
    cfg = sih_config();
end

cfg.plan.v_max              = 12.0;   % ~43 km/h on open rural road
cfg.plan.corridor_halfwidth = 3.7;
cfg.sim.t_end               = 140.0;   % room for a herd crossing and random traffic

% Cattle are slow and unpredictable; look further ahead than on the village
% road so the vehicle has room to shed speed smoothly rather than emergency
% braking.
cfg.pred.horizon            = 5.0;
cfg.plan.horizon_T          = [3.0 5.0];

% ---- road ----------------------------------------------------------------
road_wp = [   0,   0;
             50,  -2;
            100,  -3;
            150,   0;
            200,   4;
            240,   5];

scn.name = 'cattle_crossing';
scn.desc = 'Open rural road, cattle emerging from behind a parked truck at speed';
scn.rp   = sih_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', 0, 'psi', atan2(-2, 50), 'v', 11.0);
[gx, gy] = sih_wp_at(scn.rp, scn.rp.length - 12, 0);
scn.goal = [gx, gy];

% ---- the crossing --------------------------------------------------------
% The ego covers roughly 11 m/s, so it reaches station 100 at about t = 9 s.
% The herd is released at t = 7.5 s from the left verge around station 105,
% which puts them in the carriageway exactly as the vehicle arrives.
S_CROSS = 105;

% A parked truck on the left verge, just before the crossing point. It screens
% the cattle from the camera and LiDAR until they step out, so the detection is
% genuinely late rather than merely noisy.
truck = local_parked(1, 'truck', scn.rp, S_CROSS - 13, 3.1);

% Three animals, spread slightly in station and released a beat apart, so the
% road does not clear in one move.
% Each animal finishes well clear of the 3.3 m corridor. An animal that stops
% on the carriageway when its path runs out would block the road permanently.
cow1 = local_crosser(2, 'cattle', scn.rp, S_CROSS - 2, 5.0, -5.4, 1.0,  7.5);
cow2 = local_crosser(3, 'cattle', scn.rp, S_CROSS + 3, 5.6, -5.2, 0.8,  8.6);
cow3 = local_crosser(4, 'cattle', scn.rp, S_CROSS + 9, 6.0, -5.0, 0.9, 10.4);

% Ordinary traffic so the road is not otherwise empty: a slower auto-rickshaw
% ahead in the same direction, and an oncoming two-wheeler that removes the
% option of simply swinging across to the far side of the road.
auto_wp = sih_wp_path(scn.rp, [ 60 110 160 210 250], [ 1.2  0.6  1.0  0.4  0.8]);
tw_wp   = sih_wp_path(scn.rp, [230 180 130  80  30  -10], [-1.0 -0.6 -1.2 -0.8 -1.0 -0.6]);

auto = local_mover(5, 'auto',        scn.rp,  60,  1.2, 7.0, auto_wp, 0);
% The oncoming two-wheeler is timed to arrive AFTER the herd has cleared.
% Released at t = 0 it reached the crossing point at the same moment as the
% cattle, closing the offside at precisely the instant the nearside was
% blocked, which left no route through at all. Staggering it keeps the
% oncoming traffic meaningful without making the road momentarily solid.
% The oncoming two-wheeler is held back until the herd is well clear. Any
% earlier and it closes the offside at the moment the cattle block the
% nearside, which leaves no route through the road at all -- the vehicle then
% stops, correctly, and the scenario measures nothing.
tw   = local_mover(6, 'two_wheeler', scn.rp, 238, -1.8, 7.0, tw_wp,  38.0);

scn.agents = [truck, cow1, cow2, cow3, auto, tw];
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
function a = local_crosser(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)
%LOCAL_CROSSER An animal ambling across the carriageway.
%   The waypoints drift slightly in station as well as across, so the animal
%   wanders rather than tracking a straight line -- which is both realistic and
%   the behaviour the class-conditioned predictor is meant to cope with.
[x0, y0] = sih_wp_at(rp, s_at, d_from);
wp   = sih_wp_path(rp, [s_at, s_at + 2, s_at + 1, s_at + 3], ...
                       [d_from, 0.5 * d_from, 0.4 * d_to, d_to]);
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
psi0 = sih_wp_heading(rp, s0);
if size(wp, 1) >= 2
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
%LOCAL_DRAW Cattle crossing: a parked lorry hides cattle that step out from
%   behind it across the road, with traffic both ways.
L = rp.length; A = []; id = 0;
S = R.u(90, 125); side = R.pick({1, -1});
id = id + 1;
A = [A, sih_role('parked', id, R.pick({'truck', 'bus'}), rp, S - R.u(10, 16), side * 3.1)];
off = 0;
for k = 1:R.int(2, 4)
    id = id + 1;
    off = off + R.u(2, 6);
    A = [A, sih_role('crosser', id, 'cattle', rp, S - 2 + off, side * R.u(5.0, 6.2), -side * R.u(5.0, 5.6), ...
                     R.u(0.7, 1.1), max(0, S / 9.5 - R.u(1, 3.5) + 0.3 * off))]; %#ok<AGROW>
end
id = id + 1;
s0 = R.u(45, 75); sl = s0 + 50:50:L + 10; dl = arrayfun(@(~) R.u(0.4, 1.2), sl);
A = [A, sih_role('mover', id, 'auto', rp, s0, R.u(0.6, 1.4), R.u(6, 8), sl, dl, 0)];
id = id + 1;
s0 = L - R.u(0, 10); sl = s0 - 50:-50:-10; dl = arrayfun(@(~) R.u(-1.2, -0.6), sl);
A = [A, sih_role('mover', id, 'two_wheeler', rp, s0, R.u(-1.9, -1.2), R.u(6.5, 8), sl, dl, R.u(30, 45))];
end
