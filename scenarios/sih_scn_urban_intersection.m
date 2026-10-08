function [scn, cfg] = sih_scn_urban_intersection(cfg, seed)
%SIH_SCN_URBAN_INTERSECTION Unsignalled four-way crossing with unyielding traffic.
%
%   Scenario 2 of the five required by the problem statement.
%
%   The ego travels straight through a crossroads with no signals, no stop
%   line and no marked give-way. Traffic crosses its path from both sides and
%   does not concede right of way -- which is the defining feature of the
%   situation, because right of way here is settled by who commits first, not
%   by rules.
%
%   What this exercises: the crossing-threat branch of sih_risk_assess, which
%   looks for agents approaching the ego's path laterally rather than sitting
%   in it. A pure in-path lead check sees nothing until the moment of impact,
%   because a crossing vehicle is not in the path until it is. The behaviour
%   layer then has to accept or reject gaps, which is what YIELD is for.
%   [scn, cfg] = SIH_SCN_URBAN_INTERSECTION(cfg, seed) draws a random layout that keeps
%   the scenario's story (sih_scn_random): which road users, how many, where,
%   how fast and when are drawn from ranges, and the draw is checked against
%   the rules of the road (sih_scn_validate). Without a seed the scripted
%   layout below is used, which keeps the regression suite reproducible.

if nargin < 1 || isempty(cfg)
    cfg = sih_config();
end

cfg.plan.v_max              = 10.0;   % ~36 km/h through an urban junction
cfg.plan.corridor_halfwidth = 4.0;    % wide urban carriageway
cfg.sim.t_end               = 75.0;
cfg.dec.yield_gap           = 4.5;    % demand a larger gap than on open road

% ---- road: straight through the junction --------------------------------
scn.name = 'urban_intersection';
scn.desc = 'Unsignalled urban crossroads, cross traffic that does not yield';
scn.rp   = sih_ref_path([0 0; 60 0; 120 0; 170 0], 0.25, cfg.plan.corridor_halfwidth);

XJ = 85;    % corridor station of the junction centre
% The cross street: [station half-width], kept clear of scenery and allowed
% for vehicles starting off the main road.
scn.cross_streets = [XJ, 7.0];

scn.ego  = struct('x', 0, 'y', 0, 'psi', 0, 'v', 8.0);
[gx, gy] = sih_wp_at(scn.rp, scn.rp.length - 12, 0);
scn.goal = [gx, gy];

% ---- cross traffic -------------------------------------------------------
% Timings are chosen so the ego, which reaches the junction at roughly t = 10 s,
% meets a genuine stream rather than an empty crossing. Offsets are corridor
% lateral coordinates: positive is to the ego's left.
scn.agents = [ ...
    local_crosser(1, 'auto',        scn.rp, XJ -  6, -52,  70,  6.2), ...
    local_crosser(2, 'car',         scn.rp, XJ +  7,  46, -70,  7.5), ...
    local_crosser(3, 'two_wheeler', scn.rp, XJ +  1, -64,  70,  9.0), ...
    local_crosser(4, 'auto',        scn.rp, XJ + 13,  58, -70,  5.5), ...
    ...
    % A pedestrian crossing the ego's own carriageway just past the junction,
    % so the vehicle cannot simply accelerate away once the cars have cleared.
    local_walker(5, 'pedestrian', scn.rp, XJ + 26, 5.5, -5.5, 1.2, 9.0), ...
    ...
    % A bus parked beyond the junction on the far side. It occludes agent 4
    % until late, which is what makes the tracker's coasting behaviour matter
    % here, and it narrows the exit without closing it: pulled tight to the
    % kerb, there is room to pass on the offside. Placed further into the lane
    % it made the exit genuinely impassable, and a scenario the vehicle cannot
    % physically complete measures nothing about the planner.
    local_parked(6, 'bus', scn.rp, XJ + 46, -3.4)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = sih_scn_random(scn, cfg, seed, @local_draw);
end
end

% -------------------------------------------------------------------------
function a = local_crosser(id, class_name, rp, s_at, d_from, d_to, v)
%LOCAL_CROSSER An agent driving straight across the corridor.
%
%   d_from and d_to are corridor lateral offsets, so the direction of travel
%   follows from their order rather than from the sign of the speed. Deriving
%   it from the speed instead was an earlier mistake that sent agents starting
%   south of the road further south, away from the junction entirely.
[x0, y0] = sih_wp_at(rp, s_at, d_from);

% Waypoints straight across, continuing well past the far kerb so the agent
% clears the junction rather than stopping in it.
wp = sih_wp_path(rp, [s_at, s_at, s_at], [d_from, 0, d_to]);

% Heading points from d_from towards d_to. Increasing offset is to the ego's
% left, which is the corridor heading rotated by +pi/2.
psi0 = sih_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;

a = sih_agent_new(id, class_name, x0, y0, psi0, abs(v), 'path', wp, 0);
end

% -------------------------------------------------------------------------
function a = local_walker(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)
%LOCAL_WALKER A pedestrian crossing the carriageway at a given station.
[x0, y0] = sih_wp_at(rp, s_at, d_from);
wp   = sih_wp_path(rp, [s_at, s_at + 1, s_at + 2], [d_from, 0, d_to]);
psi0 = sih_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;
a = sih_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end

% -------------------------------------------------------------------------
function a = local_parked(id, class_name, rp, s_at, d_at)
%LOCAL_PARKED A stationary vehicle at the roadside.
[x0, y0] = sih_wp_at(rp, s_at, d_at);
a = sih_agent_new(id, class_name, x0, y0, sih_wp_heading(rp, s_at), 0.0, 'static', [], 0);
end

% -------------------------------------------------------------------------
function A = local_draw(rp, cfg, R)
%LOCAL_DRAW Urban crossroads: cross traffic that does not yield, a
%   pedestrian beyond the junction, a vehicle parked further on.
XJ = 85; A = []; id = 0;
speed = struct('auto', [5.0 6.5], 'car', [6.5 8.5], 'two_wheeler', [8.0 10.0], 'bus', [5.0 6.5]);
for k = 1:R.int(3, 5)
    id = id + 1;
    cls = R.pick({'auto', 'car', 'two_wheeler', 'auto', 'car', 'bus'});
    dir = R.pick({1, -1});
    A = [A, sih_role('cross_traffic', id, cls, rp, XJ + R.u(-7, 14), -dir * R.u(45, 65), dir * 70, ...
                     R.u(speed.(cls)(1), speed.(cls)(2)))]; %#ok<AGROW>
end
id = id + 1;
side = R.pick({1, -1});
A = [A, sih_role('walker', id, 'pedestrian', rp, XJ + R.u(20, 32), side * 5.5, -side * 5.5, R.u(1.0, 1.3), R.u(6, 11))];
id = id + 1;
A = [A, sih_role('parked', id, R.pick({'bus', 'truck', 'car'}), rp, XJ + R.u(38, 55), R.pick({1, -1}) * 3.4)];
end
