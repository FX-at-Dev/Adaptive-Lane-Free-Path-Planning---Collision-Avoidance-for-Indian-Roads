function m = sih_metrics(result, log, cfg)
%SIH_METRICS Performance metrics for one scenario run.
%
%   m = SIH_METRICS(result, log, cfg) computes the quantities the problem
%   statement asks to be reported, plus the supporting safety numbers:
%
%     Replanning latency   mean / p95 / max wall-clock time of a planner cycle
%     Path smoothness      max and RMS longitudinal and lateral jerk, peak
%                          path curvature and curvature rate
%     Completion           whether the goal was reached, and how long it took
%     Safety               collisions, minimum ground-truth clearance, and the
%                          time spent inside the near-miss threshold
%
%   Latency is measured on the machine that runs the simulation and is not a
%   real-time claim about embedded hardware; it is a relative measure of
%   planner cost, which is what it is useful for when comparing scenarios.

dt = cfg.sim.dt;

% ---- replanning latency --------------------------------------------------
lat = log.latency_ms(isfinite(log.latency_ms));
if isempty(lat)
    lat = NaN;
end
m.latency_mean_ms = mean(lat);
m.latency_p95_ms  = local_percentile(lat, 95);
m.latency_max_ms  = max(lat);
m.latency_over_budget = sum(lat > cfg.metric.latency_budget);
m.n_replans       = numel(lat);

% ---- smoothness ----------------------------------------------------------
% Longitudinal jerk from the realised acceleration.
jerk_lon = [0; diff(log.a) / dt];

% Lateral acceleration follows from the bicycle model's yaw rate: a_lat is
% v^2 * kappa with kappa = tan(delta)/L. Differentiating gives lateral jerk,
% which is the component passengers actually notice on a winding village road.
kappa    = tan(log.delta) / cfg.ego.wheelbase;
a_lat    = log.v.^2 .* kappa;
jerk_lat = [0; diff(a_lat) / dt];

% Jerk is measured only while the vehicle is actually moving. Coming to rest
% produces a large artificial spike: the bicycle model truncates the final
% braking step so that speed lands exactly on zero rather than going negative,
% which registers as a step change in acceleration that no occupant would ever
% feel. Including it made the reported peak jerk a measure of how often the
% vehicle stopped rather than of ride quality.
moving = log.v > 0.1;
if ~any(moving)
    moving = true(size(log.v));
end

m.jerk_lon_max  = max(abs(jerk_lon(moving)));
m.jerk_lon_rms  = sqrt(mean(jerk_lon(moving).^2));
m.jerk_lat_max  = max(abs(jerk_lat(moving)));
m.jerk_lat_rms  = sqrt(mean(jerk_lat(moving).^2));
m.frac_moving   = mean(moving);
m.a_lat_max     = max(abs(a_lat));
m.curv_max      = max(abs(kappa));
m.curv_rate_max = max(abs([0; diff(kappa) / dt]));

% Fraction of the run within the comfort limits, which is a fairer summary
% than a single peak that one emergency manoeuvre can dominate.
m.frac_jerk_ok = mean(abs(jerk_lon(moving)) <= cfg.metric.jerk_limit);
m.frac_curv_ok = mean(abs(kappa(moving))    <= cfg.metric.curv_limit);

% ---- completion ----------------------------------------------------------
m.reached      = result.reached;
m.t_to_goal    = result.t_reached;
m.t_end        = result.t_end;
m.dist_travel  = sum(sqrt(diff(log.x).^2 + diff(log.y).^2));
m.v_mean       = mean(log.v);
m.v_max        = max(log.v);

% ---- safety --------------------------------------------------------------
m.collided       = result.collided;
m.min_clear      = min(log.min_clear);
m.t_near_miss    = sum(log.min_clear < cfg.metric.min_clearance) * dt;
m.frac_infeasible = mean(~log.feasible);
m.plan_risk_max  = max(log.plan_risk);

% ---- behaviour breakdown -------------------------------------------------
states = {'CRUISE', 'FOLLOW', 'NUDGE', 'YIELD', 'CREEP', 'STOP'};
m.state_names = states;
m.state_frac  = zeros(1, numel(states));
for i = 1:numel(states)
    m.state_frac(i) = mean(strcmp(log.state, states{i}));
end

% ---- perception ----------------------------------------------------------
m.tracks_mean    = mean(log.n_confirmed);
m.dets_mean      = mean(log.n_det);
m.hypotheses_max = max(log.n_hypotheses);
end

% -------------------------------------------------------------------------
function p = local_percentile(v, q)
%LOCAL_PERCENTILE Linear-interpolation percentile.
%   Written out rather than calling prctile, which lives in the Statistics
%   toolbox on MATLAB and in a package under Octave.
v = sort(v(:));
n = numel(v);
if n == 0
    p = NaN;
    return;
end
if n == 1
    p = v(1);
    return;
end
idx = (q/100) * (n - 1) + 1;
lo  = floor(idx);
hi  = ceil(idx);
if lo == hi
    p = v(lo);
else
    p = v(lo) + (idx - lo) * (v(hi) - v(lo));
end
end
