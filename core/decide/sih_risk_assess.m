function ra = sih_risk_assess(ego, tracks, rp, cfg)
%SIH_RISK_ASSESS Situational risk features driving the behaviour state machine.
%
%   ra = SIH_RISK_ASSESS(ego, tracks, rp, cfg) reduces the track list to the
%   handful of scalars the FSM actually switches on:
%       .ttc           [s] time to collision with the closest in-path agent
%       .lead_id            track id of that agent, 0 if none
%       .lead_gap      [m] longitudinal gap to it
%       .lead_speed    [m/s] its speed along the ego heading
%       .min_clear     [m] closest current footprint clearance, any direction
%       .cross_ttc     [s] time until the most threatening crossing agent
%                          reaches the ego path
%       .cross_id           its track id, 0 if none
%       .n_near             confirmed tracks within 25 m
%       .free_left     [m] lateral room before the nearest obstacle on the left
%       .free_right    [m] the same on the right
%
%   Separating this from the FSM keeps the state machine small enough to be
%   transcribed to a Stateflow chart later without carrying geometry into it.

ra.ttc        = Inf;
ra.lead_id    = 0;
ra.lead_gap   = Inf;
ra.lead_speed = 0;
ra.min_clear  = Inf;
ra.cross_ttc  = Inf;
ra.cross_id   = 0;
ra.n_near     = 0;
ra.free_left  = cfg.plan.corridor_halfwidth;
ra.free_right = cfg.plan.corridor_halfwidth;

if isempty(tracks)
    return;
end

[ecx, ecy, erad] = sih_ego_discs(ego.x, ego.y, ego.psi, cfg);

c = cos(ego.psi);
s = sin(ego.psi);

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

    props = sih_agent_props(t.class);

    dx = t.x(1) - ego.x;
    dy = t.x(2) - ego.y;

    % Position in the ego body frame: +x ahead, +y to the left.
    fx = c * dx + s * dy;
    fy = -s * dx + c * dy;

    if hypot(dx, dy) < 25
        ra.n_near = ra.n_near + 1;
    end

    % ---- current clearance, disc to disc ------------------------------
    [acx, acy] = sih_agent_discs(t.x(1), t.x(2), t.x(4), props);
    dd = Inf;
    for a = 1:numel(ecx)
        dd = min(dd, min(sqrt((ecx(a) - acx).^2 + (ecy(a) - acy).^2)));
    end
    ra.min_clear = min(ra.min_clear, dd - erad - props.radius);

    % ---- agent velocity in the ego frame -------------------------------
    vx = t.x(3) * cos(t.x(4));
    vy = t.x(3) * sin(t.x(4));
    vf =  c * vx + s * vy;      % along ego heading
    vl = -s * vx + c * vy;      % to the ego's left

    % ---- in-path lead vehicle ------------------------------------------
    if fx > 0 && abs(fy) < PATH_HALF + props.width / 2
        gap = fx - (cfg.ego.length - 0.5*(cfg.ego.length - cfg.ego.wheelbase)) ...
                 - props.length / 2;
        gap = max(gap, 0);
        closing = ego.v - vf;
        if closing > 0.1
            ttc = gap / closing;
            if ttc < ra.ttc
                ra.ttc        = ttc;
                ra.lead_id    = t.id;
                ra.lead_gap   = gap;
                ra.lead_speed = vf;
            end
        elseif gap < ra.lead_gap
            % Still the lead vehicle even when not closing; the FSM uses the
            % gap to decide whether to follow.
            ra.lead_id    = t.id;
            ra.lead_gap   = gap;
            ra.lead_speed = vf;
        end
    end

    % ---- crossing threat -------------------------------------------------
    % An agent off to the side moving toward the ego path. This is the
    % pedestrian stepping off a verge and the cow walking across the road, and
    % it is invisible to a pure in-path lead check because it is not in the
    % path yet.
    if fx > 0 && fx < 45 && abs(fy) > PATH_HALF
        % The lateral speed threshold is deliberately well above zero. A
        % tracked agent's velocity carries real noise, and at 0.2 m/s almost
        % any stationary object at the roadside intermittently looks like it is
        % drifting into the road.
        approaching = (fy > 0 && vl < -0.5) || (fy < 0 && vl > 0.5);
        if approaching
            t_to_path = (abs(fy) - PATH_HALF - props.width/2) / max(abs(vl), 0.1);

            % Time for the ego to reach the agent's station. The speed floor
            % matters: with a floor of 0.5 m/s a stopped vehicle computes an
            % enormous t_ego, every agent within 45 m then satisfies "arrives
            % before I do", and the behaviour layer yields to traffic that is
            % nowhere near it. Being stopped is exactly when this test was
            % firing spuriously, which is also exactly when the vehicle most
            % needs to be allowed to move off.
            t_ego = fx / max(ego.v, 2.0);

            % A conflict beyond the prediction horizon is not actionable: it
            % will be re-assessed many times before it can happen.
            horizon = cfg.pred.horizon;
            if t_to_path < min(t_ego + 2.0, horizon) && t_to_path < ra.cross_ttc
                ra.cross_ttc = t_to_path;
                ra.cross_id  = t.id;
            end
        end
    end

    % ---- lateral free space ahead ---------------------------------------
    % Only obstacles roughly abreast or just ahead constrain a nudge.
    if fx > -2 && fx < 20
        edge = abs(fy) - props.width / 2 - cfg.ego.width / 2;
        if fy >= 0
            ra.free_left  = min(ra.free_left,  max(edge, 0));
        else
            ra.free_right = min(ra.free_right, max(edge, 0));
        end
    end
end
end
