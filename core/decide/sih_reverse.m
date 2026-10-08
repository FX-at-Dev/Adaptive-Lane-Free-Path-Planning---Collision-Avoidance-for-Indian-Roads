function [rv, a_cmd, delta_cmd, active, why] = sih_reverse(rv, ego, st, cfg, dt, rp)
%SIH_REVERSE Back out of a box, as a driver would.
%
%   [rv, a_cmd, delta_cmd, active, why] = SIH_REVERSE(rv, ego, st, cfg, dt, rp)
%
%   The lattice planner plans forward only. Pulled up close behind a parked
%   cart, or with a stall's corner beside the bumper, every forward path
%   clips something and the vehicle would wait for ever. A driver backs up a
%   few metres, turning the nose towards the open side, and drives round.
%   This is that manoeuvre, outside the planner:
%     * it starts when the vehicle has stood still for cfg.dec.reverse_after
%       seconds with nothing forward that moves (no feasible plan, or only
%       standing still), something close in front, the road behind clear for
%       cfg.dec.reverse_dist plus a margin, and not at the target
%     * it backs up at cfg.dec.reverse_speed, steering so the vehicle ends
%       up along the road angled cfg.dec.reverse_angle towards the side with
%       more room, for cfg.dec.reverse_dist
%       (stopping early if anything appears behind), brakes to rest, and
%       hands back to the planner -- then waits cfg.dec.reverse_cooldown
%       before it may back up again
%
%   rv is its state ([] to start); st the stack state (st.ra, st.info,
%   st.traj, st.tracks from the last cycle). active is true while it drives;
%   a_cmd and delta_cmd are then the commands, in reverse gear.

if isempty(rv)
    rv = struct('phase', 'idle', 'stuck_t', 0, 'x0', 0, 'y0', 0, 't', 0, ...
                'cool', 0, 'steer', 0, 'side', 0);
end
a_cmd = 0; delta_cmd = 0; active = false; why = '';

switch rv.phase
    case 'idle'
        rv.cool = max(0, rv.cool - dt);
        if local_boxed(ego, st, cfg)
            rv.stuck_t = rv.stuck_t + dt;
        else
            rv.stuck_t = 0;
        end
        if rv.stuck_t >= cfg.dec.reverse_after && rv.cool <= 0 && ...
                local_clear_behind(ego, st.tracks, cfg, cfg.dec.reverse_dist + 1.5) && ...
                local_on_road(ego, rp, cfg, cfg.dec.reverse_dist + 0.5)
            rv.phase = 'back';
            rv.x0 = ego.x; rv.y0 = ego.y; rv.t = 0;
            % Nose towards the side with more room.
            if st.ra.free_right >= st.ra.free_left
                rv.side = -1;
            else
                rv.side = 1;
            end
        else
            return;
        end
end

rv.t = rv.t + dt;
active = true;
switch rv.phase
    case 'back'
        moved = hypot(ego.x - rv.x0, ego.y - rv.y0);
        if moved >= cfg.dec.reverse_dist || rv.t > 8 || ~local_clear_behind(ego, st.tracks, cfg, 1.5) || ...
                ~local_on_road(ego, rp, cfg, 0.8)
            rv.phase = 'stop';
        end
        a_cmd = min(max(2.0 * (-cfg.dec.reverse_speed - ego.v), -1.5), 1.5);
        rv.steer = local_steer(ego, rv.side, rp, cfg);
        delta_cmd = rv.steer;
        why = sprintf('boxed in: backing up %.1f of %.0f m to steer round', moved, cfg.dec.reverse_dist);
    case 'stop'
        a_cmd = 2.0;                      % brakes towards rest (reverse gear)
        delta_cmd = rv.steer;
        why = 'backed up; stopping to drive on';
        if abs(ego.v) < 0.02
            rv.phase = 'idle';
            rv.stuck_t = 0;
            rv.cool = cfg.dec.reverse_cooldown;
            active = false;               % forward again from the next plan
        end
end
end

% -------------------------------------------------------------------------
function b = local_boxed(ego, st, cfg)
%LOCAL_BOXED Stopped, nothing forward that moves, something close in front.
b = false;
if abs(ego.v) > 0.1 || isempty(st.ra) || ~isfield(st, 'info') || isempty(st.info)
    return;
end
% At (or past) the target this is arriving, not being stuck.
if isfield(st.bp, 'reason') && strncmp(st.bp.reason, 'at the target', 13)
    return;
end
no_way = isfield(st.info, 'n_feasible') && st.info.n_feasible == 0;
if ~no_way && isfield(st, 'traj') && ~isempty(st.traj) && isfield(st.traj, 'v')
    no_way = max(abs(st.traj.v)) < 0.3;   % the best plan is to stand still
end
% Boxed in by something standing still: an auto passing a metre away will
% be gone in a second, and backing up for it only loses time.
close = (isfield(st.ra, 'min_clear_still') && st.ra.min_clear_still < cfg.dec.reverse_clear) || ...
        (isfield(st.ra, 'lead_still') && st.ra.lead_still && st.ra.lead_gap < cfg.dec.standoff - 1);
b = no_way && close;
end

% -------------------------------------------------------------------------
function ok = local_clear_behind(ego, tracks, cfg, reach)
%LOCAL_CLEAR_BEHIND Nothing tracked in the strip behind the vehicle.
ok = true;
c = cos(ego.psi); s = sin(ego.psi);
rear = -0.5 * (cfg.ego.length - cfg.ego.wheelbase);    % rear bumper, from the rear axle
half = 0.5 * cfg.ego.width + 0.5;
for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative'), continue; end
    fp = sih_footprint(t, cfg);
    dx = [fp.corners(:, 1); fp.cx] - ego.x;
    dy = [fp.corners(:, 2); fp.cy] - ego.y;
    u = c * dx + s * dy;                % along the vehicle
    v = -s * dx + c * dy;               % across it
    if any(u < rear + 0.3 & u > rear - reach & abs(v) < half)
        ok = false;
        return;
    end
end
end

% -------------------------------------------------------------------------
function ok = local_on_road(ego, rp, cfg, back)
%LOCAL_ON_ROAD Would the rear corners still be on the road, `back` metres back?
ok = true;
if isempty(rp), return; end
c = cos(ego.psi); s = sin(ego.psi);
rear = -0.5 * (cfg.ego.length - cfg.ego.wheelbase) - back;
for side = [-1, 1]
    px = ego.x + c * rear - s * side * 0.5 * cfg.ego.width;
    py = ego.y + s * rear + c * side * 0.5 * cfg.ego.width;
    [sp, dp] = sih_cart2frenet(rp, px, py);
    hw = interp1(rp.s, rp.halfwidth, min(max(sp, 0), rp.length), 'linear');
    if abs(dp) > hw + cfg.plan.road_overhang
        ok = false;
        return;
    end
end
end

% -------------------------------------------------------------------------
function delta = local_steer(ego, side, rp, cfg)
%LOCAL_STEER Back towards a heading a little off the road's, nose to the free side.
%   A fixed wheel angle turned the vehicle further with every back-up: after
%   three it stood 35 degrees across the road, and no forward path stayed on
%   it. Steering towards a target heading -- the road's, angled
%   cfg.dec.reverse_angle towards the open side -- converges instead.
if isempty(rp)
    delta = -side * cfg.dec.reverse_steer;
    return;
end
[~, ~, ~, psi_road] = sih_cart2frenet(rp, ego.x, ego.y);
err = psi_road + side * cfg.dec.reverse_angle - ego.psi;
err = atan2(sin(err), cos(err));
% In reverse the yaw rate is v/L tan(delta) with v < 0: turning the wheel
% the other way raises the heading.
delta = min(max(-2.0 * err, -cfg.dec.reverse_steer), cfg.dec.reverse_steer);
end
