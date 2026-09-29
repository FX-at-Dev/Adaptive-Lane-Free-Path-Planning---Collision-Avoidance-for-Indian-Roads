function [a_cmd, delta_cmd, ctrl] = sih_controller(ego, traj, ctrl, cfg)
%SIH_CONTROLLER Pure-pursuit lateral control with feed-forward speed tracking.
%
%   [a_cmd, delta_cmd, ctrl] = SIH_CONTROLLER(ego, traj, ctrl, cfg)
%
%   Follows the planned trajectory traj (fields t, x, y, v, a) and tracks its
%   speed profile. ctrl carries the speed-loop integrator between calls; pass
%   ctrl = struct() on the first step.
%
%   Longitudinal control is feed-forward plus PI, not PI alone. The planner
%   emits a speed PROFILE, and the vehicle is expected to execute it; a pure
%   feedback loop can only react to the error the profile has already
%   accumulated. From a standstill that distinction is the difference between
%   moving and not: the planned ramp calls for around 1.9 m/s^2, while a
%   proportional term acting on the speed error a fraction of a second into
%   that ramp asks for roughly 0.1, so the vehicle under-drives its own plan,
%   replans from the same near-zero speed, and never pulls away. Feeding the
%   planned acceleration forward makes the loop execute the profile and leaves
%   the PI term doing what it should -- correcting the residual.
%
%   Lateral control is pure pursuit with a speed-dependent lookahead, so the
%   vehicle is damped at speed and responsive when creeping through a market.

if ~isfield(ctrl, 'e_int') || isempty(ctrl.e_int)
    ctrl.e_int = 0;
end

dt = cfg.sim.dt;

% ---- longitudinal --------------------------------------------------------
if isempty(traj) || numel(traj.t) < 2
    v_ref = 0;
    a_ff  = cfg.veh.a_min;          % no plan: shed speed
else
    % Where to sample the planned profile. This scales with the trajectory's
    % own horizon rather than being a fixed time. The longitudinal polynomial
    % starts and ends at zero acceleration, so its first moments are nearly
    % flat; sampling a fixed 0.8 s into a 5 s plan reads a speed of about
    % 0.12 m/s and tells the controller almost nothing about where the plan is
    % going. Scenarios using a longer horizon were left creeping because of it.
    t_look = cfg.ctrl.speed_lookahead * max(traj.T, 1.0);
    t_look = min(max(t_look, 0.4), traj.t(end));

    v_ref  = interp1(traj.t, traj.v, t_look, 'linear');
    if isfield(traj, 'a')
        a_ff = interp1(traj.t, traj.a, t_look, 'linear');
    else
        a_ff = 0;
    end

    % Never let the shape of the polynomial command a slowdown the planner did
    % not intend. When the chosen candidate's TERMINAL speed is above the
    % current speed, the plan is asking the vehicle to speed up, and a sampled
    % value below the current speed is an artefact of the profile's gentle
    % start rather than a request to brake. Acting on it produced a limit
    % cycle: brake hard, spend two seconds climbing back through the jerk
    % limit, creep forward, brake again.
    if isfield(traj, 'v_target') && traj.v_target > ego.v
        v_ref = max(v_ref, min(ego.v, traj.v_target));
    end
end

e = v_ref - ego.v;
ctrl.e_int = ctrl.e_int + e * dt;
ctrl.e_int = min(max(ctrl.e_int, -cfg.ctrl.i_clamp), cfg.ctrl.i_clamp);

a_cmd = a_ff + cfg.ctrl.kp_speed * e + cfg.ctrl.ki_speed * ctrl.e_int;
a_cmd = min(max(a_cmd, cfg.veh.a_emergency), cfg.veh.a_max);

% Jerk limit. A real driveline cannot step from full acceleration to full
% braking within one 50 ms sample, and allowing it here made the reported
% path-smoothness metric meaningless -- a single behaviour transition produced
% jerk of nearly 200 m/s^3. Normal motion is held to the comfort limit; an
% emergency demand is allowed a higher rate, because taking a second to build
% up to full braking would be its own safety problem.
if ~isfield(ctrl, 'a_prev') || isempty(ctrl.a_prev)
    ctrl.a_prev = 0;
end
% The limit is asymmetric. Building acceleration is a comfort question and is
% held to the comfort rate; braking harder is a safety question and is allowed
% the emergency rate. A symmetric limit delayed the onset of braking by around
% a second, which was enough to turn an avoidable encounter into a contact.
% A vehicle already at rest is not decelerating, whatever the plan asks for.
% Letting the acceleration STATE sit at the emergency value while stopped means
% the jerk limiter has to climb all the way back through it before the vehicle
% can move at all -- nearly two seconds of paralysis after every stop, during
% which the behaviour layer would often re-trigger the stop and start the clock
% again. Collapsing it to zero makes a stop recoverable.
if ego.v <= 0 && a_cmd < 0
    a_cmd = 0;
end

da_raw = a_cmd - ctrl.a_prev;
if da_raw >= 0
    rate = cfg.veh.jerk_max;         % accelerating, or easing off the brakes
else
    rate = cfg.veh.jerk_emergency;   % braking harder: never comfort limited
end
da    = min(max(da_raw, -rate * dt), rate * dt);
a_cmd = ctrl.a_prev + da;

ctrl.a_prev = a_cmd;
ctrl.v_ref  = v_ref;
ctrl.a_ff   = a_ff;

% ---- lateral: pure pursuit ----------------------------------------------
Ld = max(cfg.ctrl.lookahead_min, cfg.ctrl.lookahead_gain * ego.v);

if isempty(traj) || numel(traj.x) < 2
    % Nothing to track. Hold the wheel and let the speed loop bring the
    % vehicle to rest.
    delta_cmd  = 0;
    ctrl.Ld    = Ld;
    ctrl.alpha = 0;
    return;
end

px = traj.x(:);
py = traj.y(:);

% First trajectory point at least Ld ahead; fall back to the final point when
% the trajectory is shorter than the lookahead.
dist = sqrt((px - ego.x).^2 + (py - ego.y).^2);
idx  = find(dist >= Ld, 1, 'first');
if isempty(idx)
    idx = numel(px);
end

% Heading to the lookahead point, expressed in the vehicle frame.
alpha  = sih_wrap_pi(atan2(py(idx) - ego.y, px(idx) - ego.x) - ego.psi);
Ld_eff = max(dist(idx), 1e-3);

delta_cmd = atan2(2 * cfg.ego.wheelbase * sin(alpha), Ld_eff);
delta_cmd = min(max(delta_cmd, -cfg.veh.delta_max), cfg.veh.delta_max);

ctrl.Ld    = Ld_eff;
ctrl.alpha = alpha;
end
