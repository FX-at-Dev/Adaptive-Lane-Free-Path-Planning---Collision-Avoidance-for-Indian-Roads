function ego = sih_bicycle_step(ego, a_cmd, delta_cmd, dt, cfg)
%SIH_BICYCLE_STEP Advance the kinematic bicycle model by one step.
%
%   ego = SIH_BICYCLE_STEP(ego, a_cmd, delta_cmd, dt, cfg) integrates the
%   rear-axle kinematic bicycle model
%       x'   = v cos(psi)
%       y'   = v sin(psi)
%       psi' = v tan(delta) / L
%   subject to acceleration, steering angle and steering rate limits.
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

% A reversing vehicle is out of scope for the lattice planner, so speed is
% clamped at zero. Clamp the deceleration too, otherwise the RK4 stages would
% integrate motion through a step in which the vehicle actually stops early.
v0 = ego.v;
if v0 + a*dt < 0
    a = -v0 / dt;
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
ego.v     = max(0, v0 + a*dt);
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
