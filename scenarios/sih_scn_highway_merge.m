function [scn, cfg] = sih_scn_highway_merge(cfg, seed)
%SIH_SCN_HIGHWAY_MERGE Informal merge into a stream of slow heavy vehicles.
%
%   Scenario 3 of the five required by the problem statement.
%
%   The ego joins a highway from a slip road and has to merge into a lane
%   already occupied by a slow truck and a bus, with faster two-wheelers
%   filtering past on both sides. There is no zip-merge convention and nobody
%   opens a gap, so the merge has to be taken rather than given.
%
%   What this exercises: the largest speed range of the five scenarios, where
%   the planner's lateral freedom is most constrained by lateral acceleration
%   rather than by geometry -- at 18 m/s a 3 m lateral shift that is trivial at
%   walking pace becomes a comfort-limited manoeuvre. It also exercises FOLLOW,
%   which the village road never triggered, because matching a slow lead and
%   waiting for room is the correct behaviour here rather than a failure.
%   [scn, cfg] = SIH_SCN_HIGHWAY_MERGE(cfg, seed) draws a random layout that keeps
%   the scenario's story (sih_scn_random): which road users, how many, where,
%   how fast and when are drawn from ranges, and the draw is checked against
%   the rules of the road (sih_scn_validate). Without a seed the scripted
%   layout below is used, which keeps the regression suite reproducible.

if nargin < 1 || isempty(cfg)
    cfg = sih_config();
end

cfg.plan.v_max              = 18.0;   % ~65 km/h
cfg.plan.corridor_halfwidth = 3.6;
cfg.sim.t_end               = 75.0;   % 300 m at merge speeds, with room to spare

% Comfort limits matter more at speed; give the planner a slightly longer view.
cfg.pred.horizon            = 5.0;
cfg.plan.horizon_T          = [3.0 5.0];

% ---- road: slip road curving into the main carriageway -------------------
% The corridor itself performs the merge. The ego starts on the slip road side
% and the reference path carries it across; the planner's job is to do that
% while the lane it is joining is occupied.
road_wp = [   0,  -7.0;
             50,  -6.6;
            100,  -3.4;
            150,  -0.6;
            220,   0.0;
            300,   0.0];

scn.name = 'highway_merge';
scn.desc = 'Slip-road merge into slow heavy traffic, no gap conceded';
scn.rp   = sih_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', -7.0, 'psi', atan2(0.4, 50), 'v', 14.0);
[gx, gy] = sih_wp_at(scn.rp, scn.rp.length - 15, 0);
scn.goal = [gx, gy];

% ---- traffic already on the carriageway ---------------------------------
% The truck and bus sit in the lane the ego is merging into, travelling well
% below the ego's cruising speed. The two-wheelers filter past, which is what
% stops the ego from simply swinging wide around the heavy vehicles.
lane   = @(s0, d) sih_wp_path(scn.rp, s0, d);

truck_wp = lane([ 95 150 210 270 320], [ 0.2  0.0 -0.2  0.0  0.0]);
bus_wp   = lane([140 200 260 320],     [-0.4 -0.2  0.0  0.0]);
tw1_wp   = lane([ 60 120 180 240 320], [ 2.4  2.0  2.6  2.2  2.0]);
tw2_wp   = lane([ 30  90 150 220 320], [-2.6 -2.2 -2.8 -2.4 -2.0]);
car_wp   = lane([ 10  70 130 200 320], [ 1.0  0.6  1.2  0.8  0.6]);

scn.agents = [ ...
    local_mover(1, 'truck',       scn.rp,  95,  0.2, 10.0, truck_wp), ...
    local_mover(2, 'bus',         scn.rp, 140, -0.4,  8.5, bus_wp), ...
    local_mover(3, 'two_wheeler', scn.rp,  60,  2.4, 16.0, tw1_wp), ...
    local_mover(4, 'two_wheeler', scn.rp,  30, -2.6, 17.0, tw2_wp), ...
    ...
    % A car closing from behind in the target lane, so the ego cannot simply
    % hang back indefinitely and merge into empty road.
    local_mover(5, 'car',         scn.rp,  10,  1.0, 15.5, car_wp)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = sih_scn_random(scn, cfg, seed, @local_draw);
end
end

% -------------------------------------------------------------------------
function a = local_mover(id, class_name, rp, s0, d0, v, wp)
%LOCAL_MOVER An agent travelling along the corridor from a given station.
[x0, y0] = sih_wp_at(rp, s0, d0);
a = sih_agent_new(id, class_name, x0, y0, sih_wp_heading(rp, s0), v, 'path', wp, 0);
end

% -------------------------------------------------------------------------
function A = local_draw(rp, cfg, R)
%LOCAL_DRAW Highway merge: slow heavy vehicles ahead, fast two-wheelers
%   filtering past, a car alongside -- no gap conceded.
L = rp.length; A = []; id = 0;
s_heavy = R.u(85, 150);
for k = 1:1 + R.coin(0.6)
    id = id + 1;
    s0 = s_heavy + (k - 1) * R.u(40, 60); d0 = R.u(-0.5, 0.3);
    sl = s0 + 60:60:L + 20; dl = arrayfun(@(~) R.u(-0.4, 0.2), sl);
    A = [A, sih_role('mover', id, R.pick({'truck', 'bus'}), rp, s0, d0, R.u(8, 10.5), sl, dl, 0)]; %#ok<AGROW>
end
for k = 1:R.int(1, 3)
    id = id + 1;
    side = R.pick({1, -1});
    s0 = R.u(20, 80); d0 = side * R.u(1.8, 2.8);
    sl = s0 + 60:60:L + 20; dl = side * arrayfun(@(~) R.u(1.8, 2.8), sl);
    A = [A, sih_role('mover', id, 'two_wheeler', rp, s0, d0, R.u(14, 17.5), sl, dl, 0)]; %#ok<AGROW>
end
id = id + 1;
s0 = R.u(10, 25); d0 = R.u(0.6, 1.4);
sl = s0 + 60:60:L + 20; dl = arrayfun(@(~) R.u(0.6, 1.2), sl);
A = [A, sih_role('mover', id, 'car', rp, s0, d0, R.u(14, 16), sl, dl, 0)];
end
