function ego = sih_bicycle_step(ego, a_cmd, delta_cmd, dt, cfg, gear)
%SIH_BICYCLE_STEP Advance the kinematic bicycle model by one step.
%
%   ego = SIH_BICYCLE_STEP(ego, a_cmd, delta_cmd, dt, cfg, gear) integrates the
%   rear-axle kinematic bicycle model
%       x'   = v cos(psi)
%       y'   = v sin(psi)
%       psi' = v tan(delta) / L
%   subject to acceleration, steering angle and steering rate limits.
%
%   gear is +1 (forward, the default) or -1 (reverse). In forward gear the
%   speed stays at or above zero, in reverse at or below it, down to
%   -cfg.veh.v_reverse_max: braking stops the vehicle, it never carries it
%   into the other direction. a_cmd is the change of the signed speed, so in
%   reverse a negative a_cmd backs up faster and a positive one brakes.
%
%   ego has fields x, y, psi, v, delta. The actuator limits are applied first
%   (they are non-smooth saturations, so they do not belong inside the
%   integrator), then the pose is advanced with RK4 holding the steering angle
%   constant and letting speed vary linearly across the step. That input
%   profile is exactly what the model sees, so with a = 0 and a settled
%   steering angle the integration reproduces the closed-form circular arc to
%   near machine precision -- which is what tests/test_bicycle.m checks.

L = cfg.ego.wheelbase;

% ---- actuator limits ----------------------------------------------------
a = min(max(a_cmd, cfg.veh.a_emergency), cfg.veh.a_max);

d_target = min(max(delta_cmd, -cfg.veh.delta_max), cfg.veh.delta_max);
d_step   = cfg.veh.delta_rate * dt;
delta    = ego.delta + min(max(d_target - ego.delta, -d_step), d_step);
delta    = min(max(delta, -cfg.veh.delta_max), cfg.veh.delta_max);

% The speed may not cross zero within a gear: forward it is clamped at zero
% (the lattice planner plans forward only), in reverse at zero from below
% (sih_reverse backs out of a box). Clamp the acceleration too, otherwise
% the RK4 stages would integrate motion through a step in which the
% vehicle actually stops early.
if nargin < 6 || isempty(gear), gear = 1; end
v0 = ego.v;
if gear >= 0
    if v0 + a*dt < 0
        a = -v0 / dt;
    end
else
    if v0 + a*dt > 0
        a = -v0 / dt;
    elseif v0 + a*dt < -cfg.veh.v_reverse_max
        a = (-cfg.veh.v_reverse_max - v0) / dt;
    end
end

% ---- RK4 on (x, y, psi) with v(t) = v0 + a t, delta constant -------------
% Written with an explicit subfunction rather than a nested one: Octave's
% support for nested functions and their shared scope is unreliable, and this
% file has to run identically on both interpreters.
tanL = tan(delta) / L;

z0 = [ego.x; ego.y; ego.psi];
k1 = local_deriv(z0,                  0,     v0, a, tanL);
k2 = local_deriv(z0 + (dt/2) * k1,    dt/2,  v0, a, tanL);
k3 = local_deriv(z0 + (dt/2) * k2,    dt/2,  v0, a, tanL);
k4 = local_deriv(z0 +  dt    * k3,    dt,    v0, a, tanL);
z  = z0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

ego.x     = z(1);
ego.y     = z(2);
ego.psi   = sih_wrap_pi(z(3));
if gear >= 0
    ego.v = max(0, v0 + a*dt);
else
    ego.v = min(0, v0 + a*dt);
end
ego.delta = delta;
ego.a     = a;
end

% -------------------------------------------------------------------------
function dz = local_deriv(z, tau, v0, a, tanL)
%LOCAL_DERIV Bicycle kinematics with a linear speed ramp across the step.
v  = v0 + a * tau;
dz = [v * cos(z(3));
      v * sin(z(3));
      v * tanL];
end
