function ra = sih_risk_assess(ego, tracks, rp, cfg)
%SIH_RISK_ASSESS Situational risk features driving the behaviour state machine.
%
%   ra = SIH_RISK_ASSESS(ego, tracks, rp, cfg) reduces the track list to the
%   handful of scalars the FSM actually switches on:
%       .ttc           [s] time to collision with the closest in-path agent
%       .lead_id            track id of that agent, 0 if none
%       .lead_gap      [m] gap to it along the road
%       .lead_speed    [m/s] its speed along the road
%       .min_clear     [m] closest current footprint clearance, any direction
%       .min_clear_moving [m] the same, to objects that are not still
%       .cross_ttc     [s] time until the most threatening crossing agent
%                          reaches the ego path
%       .cross_id           its track id, 0 if none
%       .n_near             confirmed tracks within 25 m
%       .free_left     [m] lateral room before the nearest obstacle on the left
%       .free_right    [m] the same on the right
%       .in_path            ids of the tracks whose footprint is in the path
%       .caution_gap   [m] gap to the nearest object in the path that is not
%                          yet established (t.established): one that may be
%                          a ghost. It only slows the vehicle (sih_behavior_fsm);
%                          everything above counts established objects only.
%       .caution_id         its track id, 0 if none
%
%   Geometry. Every object is its footprint (sih_footprint): its real centre,
%   the orientation of its long axis and its size -- not a class-sized box
%   turned by a motion heading that, for a parked object, is noise. And "in
%   the path" is measured along the road: the footprint's corners go to
%   road coordinates (s along, d across, sih_cart2frenet) and an object is in
%   the path when it overlaps the strip the ego occupies across the road,
%   ahead of it. Measured along the ego's straight-ahead axis instead, a
%   wall on the outside of a bend reads as "directly in front".
%
%   Separating this from the FSM keeps the state machine small enough to be
%   transcribed to a Stateflow chart later without carrying geometry into it.

ra.ttc        = Inf;
ra.lead_id    = 0;
ra.lead_gap   = Inf;
ra.lead_speed = 0;
ra.lead_still = false;   % the lead is standing still (parked, waiting)
ra.lead_box   = [];      % [s_near d_lo d_hi] of the lead, road coordinates
ra.still_ahead = zeros(0, 3);   % [s_near d_lo d_hi] of every established still object just ahead
ra.min_clear  = Inf;
ra.min_clear_moving = Inf;
ra.min_clear_still  = Inf;
ra.cross_ttc  = Inf;
ra.cross_id   = 0;
ra.n_near     = 0;
ra.free_left  = cfg.plan.corridor_halfwidth;
ra.free_right = cfg.plan.corridor_halfwidth;
ra.in_path    = zeros(1, 0);
ra.caution_gap = Inf;
ra.caution_id  = 0;

if isempty(tracks)
    return;
end

[ecx, ecy, erad] = sih_ego_discs(ego.x, ego.y, ego.psi, cfg);

% The ego in road coordinates: its rear axle and how far its nose reaches.
[s_e, d_e] = sih_cart2frenet(rp, ego.x, ego.y);
nose = cfg.ego.length - 0.5 * (cfg.ego.length - cfg.ego.wheelbase);

% Half-width of the strip that counts as "in the ego's path". The margin over
% the vehicle's own half-width is kept modest: on a narrow street a generous
% band sweeps in objects parked at the kerb, and an obstacle beside the road is
% a very different situation from one in front of it.
PATH_HALF = 0.5 * cfg.ego.width + 0.35;

for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative')
        continue;
    end
    fp = sih_footprint(t, cfg);
    established = ~isfield(t, 'established') || t.established;

    if established && hypot(fp.cx - ego.x, fp.cy - ego.y) < 25
        ra.n_near = ra.n_near + 1;
    end

    % ---- current clearance, disc to disc ------------------------------
    if established
        dd = Inf;
        for a = 1:numel(ecx)
            dd = min(dd, min(sqrt((ecx(a) - fp.ox).^2 + (ecy(a) - fp.oy).^2)));
        end
        ra.min_clear = min(ra.min_clear, dd - erad - fp.r);
        if ~fp.still
            ra.min_clear_moving = min(ra.min_clear_moving, dd - erad - fp.r);
        else
            ra.min_clear_still = min(ra.min_clear_still, dd - erad - fp.r);
        end
    end

    % ---- the footprint in road coordinates -----------------------------
    sc = zeros(4, 1); dc = zeros(4, 1);
    for k = 1:4
        [sc(k), dc(k)] = sih_cart2frenet(rp, fp.corners(k, 1), fp.corners(k, 2));
    end
    [~, ~, ~, psi_ref] = sih_cart2frenet(rp, fp.cx, fp.cy);
    ahead = min(sc) - s_e;              % from the rear axle to its nearest part
    lo = min(dc) - d_e;                 % its extent across the road, relative
    hi = max(dc) - d_e;                 %   to the ego's own offset

    % Its velocity along and across the road.
    v_s =  cos(psi_ref) * fp.vx + sin(psi_ref) * fp.vy;
    v_d = -sin(psi_ref) * fp.vx + cos(psi_ref) * fp.vy;

    in_path = hi > -PATH_HALF && lo < PATH_HALF;

    % Everything standing just ahead, in any part of the road: the planner
    % keeps room to pull out from behind each (sih_behavior_fsm, bp.hold).
    if established && fp.still && ahead > -1 && ahead < nose + 20
        ra.still_ahead(end+1, :) = [min(sc), min(dc), max(dc)];
    end

    % ---- not yet established: slow for it, do not stop for it ------------
    if ~established
        if in_path && max(sc) > s_e
            gap = max(ahead - nose, 0);
            if gap < ra.caution_gap
                ra.caution_gap = gap;
                ra.caution_id  = t.id;
            end
        end
        continue;
    end

    % ---- in-path lead ------------------------------------------------------
    if in_path && max(sc) > s_e
        ra.in_path(end+1) = t.id;
        gap = max(ahead - nose, 0);
        closing = ego.v - v_s;
        if closing > 0.1
            ttc = gap / closing;
            if ttc < ra.ttc
                ra.ttc        = ttc;
                ra.lead_id    = t.id;
                ra.lead_gap   = gap;
                ra.lead_speed = v_s;
                ra.lead_still = fp.still;
                ra.lead_box = [min(sc), min(dc), max(dc)];   % where it is along and across the road
            end
        elseif gap < ra.lead_gap
            % Still the lead vehicle even when not closing; the FSM uses the
            % gap to decide whether to follow.
            ra.lead_id    = t.id;
            ra.lead_gap   = gap;
            ra.lead_speed = v_s;
            ra.lead_still = fp.still;
            ra.lead_box = [min(sc), min(dc), max(dc)];   % where it is along and across the road
        end
    end

    % ---- crossing threat -------------------------------------------------
    % An agent beside the path moving toward it: the pedestrian stepping off
    % a verge, the cow walking across the road. A stationary footprint has
    % no velocity, so a parked object cannot look as if it is crossing.
    if ~in_path && ahead > 0 && ahead < 45
        % The lateral speed threshold is deliberately well above zero: a
        % tracked agent's velocity carries real noise.
        approaching = (lo > 0 && v_d < -0.5) || (hi < 0 && v_d > 0.5);
        if approaching
            lateral_gap = min(abs(lo), abs(hi)) - PATH_HALF;
            t_to_path = max(lateral_gap, 0) / max(abs(v_d), 0.1);

            % Time for the ego to reach the agent's station. The speed floor
            % keeps a stopped vehicle from computing an enormous t_ego that
            % makes every agent within 45 m "arrive first" -- which is
            % exactly when the vehicle most needs to be allowed to move off.
            t_ego = ahead / max(ego.v, 2.0);

            % A conflict beyond the prediction horizon is not actionable.
            horizon = cfg.pred.horizon;
            if t_to_path < min(t_ego + 2.0, horizon) && t_to_path < ra.cross_ttc
                ra.cross_ttc = t_to_path;
                ra.cross_id  = t.id;
            end
        end
    end

    % ---- lateral free space ahead ---------------------------------------
    % Only obstacles roughly abreast or just ahead constrain a nudge.
    % The gap between the ego's side and the object's near edge, on the side
    % the object is on (zero when it overlaps the ego's strip).
    if max(sc) > s_e - 2 && min(sc) < s_e + 20
        if lo + hi >= 0
            ra.free_left  = min(ra.free_left,  max(lo - cfg.ego.width / 2, 0));
        else
            ra.free_right = min(ra.free_right, max(-hi - cfg.ego.width / 2, 0));
        end
    end
end
end
