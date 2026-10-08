function [scn, cfg] = sih_scn_village_road(cfg, seed)
%SIH_SCN_VILLAGE_ROAD Unmarked rural road with mixed traffic and no lane discipline.
%
%   [scn, cfg] = SIH_SCN_VILLAGE_ROAD(cfg) returns the scenario struct and a
%   config with the scenario's overrides applied.
%
%   Scenario 1 of the five required by the problem statement.
%
%   The road is a single unmarked carriageway about 7 m wide, curving gently,
%   with no centre line and soft edges. Traffic:
%     - an auto-rickshaw approaching head-on, on the ego's side of the road,
%       which is ordinary behaviour here rather than a violation
%     - a cyclist ahead in the same direction, slower than the ego
%     - a pedestrian walking along the verge who steps further into the road
%     - a parked pushcart narrowing the carriageway
%
%   What this exercises: the corridor reference has no lane to hold, so the
%   planner must use its full lateral freedom to pass the oncoming rickshaw and
%   the pushcart, and the behaviour layer must distinguish "ease around"
%   (NUDGE) from "slow right down" (CREEP) as the free space closes.
%
%   The scenario struct is the shared contract described in the technical
%   report: the same fields feed the Octave core, the Simulink model and the
%   MATLAB drivingScenario builder.
%   [scn, cfg] = SIH_SCN_VILLAGE_ROAD(cfg, seed) draws a random layout that keeps
%   the scenario's story (sih_scn_random): which road users, how many, where,
%   how fast and when are drawn from ranges, and the draw is checked against
%   the rules of the road (sih_scn_validate). Without a seed the scripted
%   layout below is used, which keeps the regression suite reproducible.

if nargin < 1 || isempty(cfg)
    cfg = sih_config();
end

% ---- scenario-specific configuration ------------------------------------
cfg.plan.v_max              = 8.0;    % ~29 km/h, realistic for a village road
cfg.plan.corridor_halfwidth = 3.2;
% 190 m of road with four interacting road users. Measured completion is
% around 50 s, so the budget allows for a slower run without truncating one
% that would otherwise succeed.
cfg.sim.t_end               = 120.0;   % room for random traffic and the stop at the target

% ---- road ----------------------------------------------------------------
% A gently curving unmarked road. The waypoints define the drivable corridor
% centreline, not a lane centre: the vehicle is expected to depart from it.
road_wp = [   0,   0;
             40,   2;
             80,  10;
            120,  16;
            160,  14;
            200,   6];

scn.name = 'village_road';
scn.rp   = sih_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);
scn.desc = 'Unmarked village road, oncoming rickshaw, cyclist, verge pedestrian, parked pushcart';

% ---- ego -----------------------------------------------------------------
scn.ego = struct('x', 0, 'y', 0, 'psi', atan2(2, 40), 'v', 6.0);

% Goal near the end of the corridor, short of the final waypoint so the run
% finishes cleanly rather than at a path endpoint singularity.
gx = interp1(scn.rp.s, scn.rp.x, scn.rp.length - 12);
gy = interp1(scn.rp.s, scn.rp.y, scn.rp.length - 12);
scn.goal = [gx, gy];

% ---- traffic -------------------------------------------------------------
% Agent waypoints are laid out along the corridor with deliberate lateral
% offsets, which is how the "no lane discipline" character is expressed.
% Oncoming traffic keeps to its own (left) side, our right: India drives on
% the left. It still wanders within its lane.
onc = local_offset_path(scn.rp, [190 150 110 70 30 -10], [-1.4 -1.1 -1.7 -1.2 -1.5 -1.3]);
cyc = local_offset_path(scn.rp, [ 55  95 135 175 210],   [ 1.6  1.9 1.4 1.0 1.0]);
% The pedestrian crosses fully to the far verge. An earlier version stopped
% them at d = 1.2, mid-carriageway, where they stood indefinitely and became a
% permanent obstacle -- a scenario artefact rather than behaviour worth testing.
ped = local_offset_path(scn.rp, [ 70  76  84  92],      [ 3.4  1.2 -1.8 -4.6]);

[px, py] = local_point(scn.rp, 118, -2.4);      % parked pushcart
[ax, ay] = local_point(scn.rp, 190, -1.4);      % oncoming rickshaw start
[bx, by] = local_point(scn.rp,  55,  1.6);      % cyclist start
[qx, qy] = local_point(scn.rp,  70,  3.0);      % pedestrian start

scn.agents = [ ...
    sih_agent_new(1, 'auto',       ax, ay, local_heading(scn.rp, 190) + pi, ...
                  6.5, 'path', onc, 0), ...
    sih_agent_new(2, 'bicycle',    bx, by, local_heading(scn.rp,  55), ...
                  3.6, 'path', cyc, 0), ...
    sih_agent_new(3, 'pedestrian', qx, qy, local_heading(scn.rp,  70) - 0.9, ...
                  1.1, 'path', ped, 6.0), ...
    sih_agent_new(4, 'pushcart',   px, py, local_heading(scn.rp, 118), ...
                  0.0, 'static', [], 0)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = sih_scn_random(scn, cfg, seed, @local_draw);
end
end

% -------------------------------------------------------------------------
function wp = local_offset_path(rp, s_list, d_list)
%LOCAL_OFFSET_PATH Waypoints at given stations and lateral offsets.
wp = zeros(numel(s_list), 2);
for k = 1:numel(s_list)
    [wp(k,1), wp(k,2)] = local_point(rp, s_list(k), d_list(k));
end
end

% -------------------------------------------------------------------------
function [x, y] = local_point(rp, s, d)
s = min(max(s, 0), rp.length);
[x, y] = sih_frenet2cart(rp, s, d);
end

% -------------------------------------------------------------------------
function psi = local_heading(rp, s)
s   = min(max(s, 0), rp.length);
psi = sih_wrap_pi(interp1(rp.s, rp.psi, s, 'linear'));
end

% -------------------------------------------------------------------------
function A = local_draw(rp, cfg, R)
%LOCAL_DRAW Village road: oncoming traffic, a slower user ahead in the same
%   direction, a pedestrian crossing from the verge, parked obstacles.
L = rp.length; A = []; id = 0;
speed = struct('auto', [5.5 7.0], 'car', [6.0 7.5], 'two_wheeler', [6.5 8.0], 'bicycle', [3.0 4.0]);
for k = 1:R.int(1, 2)                                     % oncoming
    id = id + 1;
    cls = R.pick({'auto', 'car', 'two_wheeler', 'auto', 'bicycle'});
    s0 = R.u(130, L - 5); d0 = R.u(-2.0, -0.9);         % in its own lane
    sl = s0 - 40:-40:-20; dl = d0 + arrayfun(@(~) R.u(-0.6, 0.6), sl);
    A = [A, sih_role('mover', id, cls, rp, s0, d0, R.u(speed.(cls)(1), speed.(cls)(2)), ...
                     sl, max(min(dl, -0.8), -2.1), (k - 1) * R.u(8, 15))]; %#ok<AGROW>
end
id = id + 1;                                              % slower, same way
cls = R.pick({'bicycle', 'bicycle', 'two_wheeler'});
s0 = R.u(40, 80); d0 = R.u(1.0, 2.0);
sl = s0 + 40:40:L + 20; dl = arrayfun(@(~) R.u(0.8, 2.0), sl);
A = [A, sih_role('mover', id, cls, rp, s0, d0, R.u(speed.(cls)(1), speed.(cls)(2)) * 0.8, sl, dl, 0)];
id = id + 1;                                              % verge pedestrian
s = R.u(60, 120); side = R.pick({1, -1});
A = [A, sih_role('walker', id, 'pedestrian', rp, s, side * R.u(3.4, 4.2), -side * R.u(3.6, 4.8), ...
                 R.u(0.9, 1.3), max(0, s / 6 - R.u(5, 8)))];
for k = 1:R.int(1, 2)                                     % parked
    id = id + 1;
    cls = R.pick({'pushcart', 'static', 'car'});
    A = [A, sih_role('parked', id, cls, rp, R.u(95, 165), R.pick({1, -1}) * R.u(2.2, 2.6))]; %#ok<AGROW>
end
end
