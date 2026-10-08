function pred = sih_predict_intent(tracks, cfg)
%   (Every object is its footprint, sih_footprint: real centre, long-axis
%   orientation and size. A stationary object gets one hypothesis -- it stays
%   where it is, as it is -- because rolling a parked object forward with its
%   filter heading and turn rate, which are noise, swept its footprint round
%   on the spot and across the road.)
%SIH_PREDICT_INTENT Class-conditioned multi-hypothesis motion prediction.
%
%   pred = SIH_PREDICT_INTENT(tracks, cfg) turns confirmed tracks into the
%   flat occupancy-disc structure consumed by sih_collision_check:
%       pred.t    [K x 1]  prediction times, 0 : cfg.pred.dt : cfg.pred.horizon
%       pred.x    [K x A]  predicted x of each occupancy disc
%       pred.y    [K x A]  predicted y
%       pred.r    [1 x A]  disc radius, inflated and growing with time
%       pred.w    [1 x A]  probability weight of the owning hypothesis
%       pred.hyp  [1 x A]  hypothesis id shared by one hypothesis's discs
%       pred.src  [1 x A]  originating track id (diagnostics and plotting)
%       pred.mode [1 x A]  hypothesis kind, an index into pred.mode_names
%
%   Why multi-hypothesis rather than a single extrapolation. On a laned road a
%   constant-turn-rate extrapolation is a good bet, because the lane tells the
%   vehicle where to go. On an unmarked Indian road there is no such
%   constraint: an auto-rickshaw holding a straight line at 6 m/s may continue,
%   may brake for a pothole, or may translate a metre sideways to overtake a
%   cyclist, and nothing observable distinguishes these until it happens.
%   Collapsing that to one trajectory does not make the uncertainty go away, it
%   just hides it from the planner. So each track emits several weighted
%   hypotheses, and the weights are conditioned on class: a bus is mostly
%   "carry on", cattle are mostly "who knows".
%
%   The hypotheses are deliberately few and interpretable rather than a sampled
%   distribution: the planner has to evaluate every candidate trajectory
%   against every hypothesis at 10 Hz, so the count directly costs latency.

pred.t   = (0:cfg.pred.dt:cfg.pred.horizon)';
K        = numel(pred.t);

pred.x   = zeros(K, 0);
pred.y   = zeros(K, 0);
pred.r   = zeros(1, 0);
pred.w   = zeros(1, 0);
pred.hyp = zeros(1, 0);
pred.src = zeros(1, 0);
pred.mode = zeros(1, 0);
pred.mode_names = {'keep', 'brake', 'left', 'right'};

if isempty(tracks)
    return;
end

hyp_id = 0;

for i = 1:numel(tracks)
    t = tracks(i);

    % Tentative tracks are usually clutter. Planning around them would make
    % the vehicle brake for measurement noise.
    if strcmp(t.status, 'tentative')
        continue;
    end

    props = sih_agent_props(t.class);
    fp = sih_footprint(t, cfg);

    % Positional uncertainty from the filter feeds straight into the footprint
    % the planner must avoid, so a poorly observed track is given a wider
    % berth than a well observed one without any special case downstream.
    sigma0 = sqrt(max(t.P(1,1), 0) + max(t.P(2,2), 0));

    [modes, weights] = local_hypotheses(props);
    if fp.still
        modes = {'keep'};
        weights = 1;
    end
    % An object not yet established may be a ghost. It slows the vehicle
    % (sih_behavior_fsm, ra.caution_gap) and nothing more: weighed in the
    % planner, even lightly, it was still enough to reject every path, and
    % "no plan" brought the car to an emergency stop for nothing. Real, it is
    % established within cfg.track.establish_hits scans and counts in full.
    if isfield(t, 'established') && ~t.established
        weights = weights * cfg.track.ghost_weight;
        if cfg.track.ghost_weight <= 0, continue; end
    end

    for m = 1:numel(modes)
        if weights(m) < 0.02
            continue;      % not worth the collision-checking cost
        end
        hyp_id = hyp_id + 1;

        if fp.still
            hx = fp.cx * ones(K, 1);
            hy = fp.cy * ones(K, 1);
            hpsi = fp.theta * ones(K, 1);
        else
            x0 = t.x;
            x0(1) = fp.cx; x0(2) = fp.cy;
            [hx, hy, hpsi] = local_rollout(x0, modes{m}, props, pred.t, cfg);
        end

        % Radius grows with horizon: prediction error accumulates, and for
        % erratic classes it accumulates faster.
        %
        % Growth is also scaled by how fast the agent is actually moving.
        % Uncertainty about a road user is dominated by where it might GO, and
        % a parked pushcart is not going anywhere -- inflating its footprint by
        % half a metre over the horizon, as a speed-independent rule does,
        % closed the gap beside every stall on a market street and left the
        % vehicle wedged with nowhere feasible to put itself. The floor keeps
        % some growth for a stopped agent that might move off.
        spd = abs(t.x(3)) * ~fp.still;
        mob = max(0.2, min(1.0, spd / max(props.v_typ, 0.5)));
        grow = cfg.pred.sigma_grow * (0.5 + props.erratic) * mob * pred.t;
        rad  = fp.r + cfg.risk.inflate + 0.5 * sigma0;

        % The footprint's discs, carried along the hypothesis: spaced along
        % its long axis while still, along the direction of travel while
        % moving (sih_footprint already chose which).
        nd = fp.n;
        offs = -fp.L / 2 + (fp.L / nd) * ((1:nd) - 0.5);
        cx = hx * ones(1, nd) + cos(hpsi) * offs;
        cy = hy * ones(1, nd) + sin(hpsi) * offs;

        % One radius per disc column. The time-varying growth cannot be
        % expressed in a per-column radius, so it is folded in at its
        % horizon-average value -- the collision checker compares against a
        % single radius per column by design, and averaging keeps the check
        % conservative early and slightly optimistic at the far end of the
        % horizon, where the trajectory will have been replanned long before.
        rcol = rad + sih_mean(grow);

        pred.x   = [pred.x,   cx];
        pred.y   = [pred.y,   cy];
        pred.r   = [pred.r,   rcol * ones(1, nd)];
        pred.w   = [pred.w,   weights(m) * ones(1, nd)];
        pred.hyp = [pred.hyp, hyp_id * ones(1, nd)];
        pred.src = [pred.src, t.id * ones(1, nd)];
        pred.mode = [pred.mode, m * ones(1, nd)];
    end
end
end

% -------------------------------------------------------------------------
function [modes, w] = local_hypotheses(props)
%LOCAL_HYPOTHESES Weighted intent set for a road-user class.
%
%   'keep'   continue the estimated CTRV motion
%   'brake'  decelerate, as when yielding or reacting to an obstacle
%   'left'   translate laterally left while continuing forward
%   'right'  translate laterally right
%
%   The weights are the class model. A bus concentrates almost all of its mass
%   on 'keep' because it is physically committed to its path. Cattle spread
%   mass nearly evenly, which is what forces the planner to leave real room
%   around them in the mandated cattle-crossing scenario.

e = props.erratic;      % 0 disciplined ... 1 unpredictable
y = props.yields;       % tendency to give way

w_keep  = 0.88 - 0.50 * e;
w_brake = 0.10 + 0.18 * y;
w_side  = max(0, 1 - w_keep - w_brake);

modes = {'keep', 'brake', 'left', 'right'};
w     = [w_keep, w_brake, w_side/2, w_side/2];
w     = w / sum(w);
end

% -------------------------------------------------------------------------
function [hx, hy, hpsi] = local_rollout(x0, mode, props, tvec, cfg)
%LOCAL_ROLLOUT Propagate one hypothesis over the prediction horizon.
K   = numel(tvec);
dt  = cfg.pred.dt;

hx   = zeros(K, 1);
hy   = zeros(K, 1);
hpsi = zeros(K, 1);

x = x0(:);

% Lateral drift speed is capped by the class's manoeuvrability and by its
% current speed: a stationary pushcart does not suddenly translate sideways at
% 1.6 m/s, and a fast two-wheeler can.
v_lat = 0;
switch mode
    case 'left'
        v_lat =  props.lat_agility * min(1, 0.3 + abs(x(3)) / max(props.v_typ, 1));
    case 'right'
        v_lat = -props.lat_agility * min(1, 0.3 + abs(x(3)) / max(props.v_typ, 1));
end

a_lon = 0;
if strcmp(mode, 'brake')
    a_lon = -0.6 * props.a_brake;
end

% Largest plausible sideways excursion within the horizon, and the running
% total against it.
LAT_CAP    = 2.5;    % [m]
lat_travel = 0;

hx(1)   = x(1);
hy(1)   = x(2);
hpsi(1) = x(4);

dt2 = dt^2;
for k = 2:K
    % sih_ctrv_motion's state update, inline: no Jacobian is needed here,
    % and a function call per step per hypothesis was a sizeable share of
    % a stack step. Same arithmetic, same result.
    v = x(3); psi = x(4); w = x(5);
    if abs(w) > 1e-4
        psi1 = psi + w * dt;
        x(1) = x(1) + (v / w) * (sin(psi1) - sin(psi));
        x(2) = x(2) + (v / w) * (cos(psi) - cos(psi1));
        x(4) = psi1;
    else
        x(1) = x(1) + v * cos(psi) * dt - 0.5 * v * sin(psi) * w * dt2;
        x(2) = x(2) + v * sin(psi) * dt + 0.5 * v * cos(psi) * w * dt2;
        x(4) = psi + w * dt;
    end

    % Longitudinal intent.
    x(3) = max(0, x(3) + a_lon * dt);

    % Lateral intent, applied as a translation along the left normal so that
    % the agent slides sideways rather than turning. That is what overtaking
    % traffic on an unmarked road actually looks like: the heading barely
    % changes while the vehicle occupies a different part of the road.
    %
    % The total excursion is capped. Integrating a constant lateral rate over
    % the whole horizon implies, for a pedestrian, walking four metres directly
    % sideways without ever turning -- which is not a manoeuvre, it is a new
    % heading, and the CTRV term would already be tracking it. Left uncapped
    % these cones spanned the entire carriageway and left the planner with
    % nothing feasible at any speed.
    if v_lat ~= 0 && abs(lat_travel) < LAT_CAP
        step = v_lat * dt;
        x(1) = x(1) - step * sin(x(4));
        x(2) = x(2) + step * cos(x(4));
        lat_travel = lat_travel + step;
    end

    hx(k)   = x(1);
    hy(k)   = x(2);
    hpsi(k) = x(4);
end
end
