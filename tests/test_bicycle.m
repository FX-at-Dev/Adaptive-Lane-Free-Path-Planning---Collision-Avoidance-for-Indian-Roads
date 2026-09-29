function test_bicycle()
%TEST_BICYCLE Kinematic bicycle model against closed-form motion.
%
%   With a constant speed and a settled steering angle the rear-axle bicycle
%   model traces an exact circle of radius R = L/tan(delta). Comparing forty
%   integration steps against that closed form validates both the kinematics
%   and the RK4 integrator far more sharply than a trajectory plot would.

cfg = sih_config();
L   = cfg.ego.wheelbase;
dt  = cfg.sim.dt;

% ---- constant-steer circular arc ----------------------------------------
delta = 0.20;               % rad, well inside the steering limit
v     = 5.0;                % m/s
Rc    = L / tan(delta);

ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', v, 'delta', delta);

n = 40;
for k = 1:n
    % Command the angle it already holds, so the rate limiter never engages.
    ego = sih_bicycle_step(ego, 0, delta, dt, cfg);
end

t   = n * dt;
psi = v * t / Rc;
sih_assert_close(ego.psi, sih_wrap_pi(psi), 1e-7, 'circle heading');
sih_assert_close(ego.x,   Rc * sin(psi),    1e-6, 'circle x');
sih_assert_close(ego.y,   Rc * (1 - cos(psi)), 1e-6, 'circle y');
sih_assert_close(ego.v,   v, 1e-12, 'circle speed held');

% Positive steering must turn left (+y), matching the left-positive lateral
% convention of the Frenet frame.
sih_assert_true(ego.y > 0, 'positive steer must yield a left turn');

% ---- straight line ------------------------------------------------------
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 6.0, 'delta', 0);
for k = 1:20
    ego = sih_bicycle_step(ego, 0, 0, dt, cfg);
end
sih_assert_close(ego.x, 6.0 * 20 * dt, 1e-9, 'straight distance');
sih_assert_close(ego.y, 0, 1e-12, 'straight lateral drift');

% ---- constant acceleration ----------------------------------------------
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 0, 'delta', 0);
a   = 2.0;
n   = 25;
for k = 1:n
    ego = sih_bicycle_step(ego, a, 0, dt, cfg);
end
t = n * dt;
sih_assert_close(ego.v, a*t,           1e-9,  'accel final speed');
sih_assert_close(ego.x, 0.5*a*t^2,     1e-9,  'accel distance');

% ---- the vehicle must stop, not reverse ---------------------------------
% A hard brake command from low speed has to leave v at exactly zero, and the
% distance travelled must match the partial step before the stop.
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 0.1, 'delta', 0);
ego = sih_bicycle_step(ego, -7.0, 0, dt, cfg);
sih_assert_close(ego.v, 0, 1e-12, 'braking clamps at zero speed');
sih_assert_true(ego.x > 0 && ego.x < 0.1*dt + 1e-9, ...
    'stopping step must travel forward but less than a constant-speed step');

% Once stopped, a further brake command must not move the vehicle backwards.
x_stop = ego.x;
ego = sih_bicycle_step(ego, -7.0, 0, dt, cfg);
sih_assert_close(ego.v, 0, 1e-12, 'stays stopped');
sih_assert_close(ego.x, x_stop, 1e-12, 'no reverse creep');

% ---- actuator limits ----------------------------------------------------
% Steering rate limiting: commanding full lock for one step may only move the
% angle by delta_rate*dt.
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 3.0, 'delta', 0);
ego = sih_bicycle_step(ego, 0, cfg.veh.delta_max, dt, cfg);
sih_assert_close(ego.delta, cfg.veh.delta_rate * dt, 1e-12, 'steering rate limit');

% Acceleration saturation.
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 3.0, 'delta', 0);
ego = sih_bicycle_step(ego, 99, 0, dt, cfg);
sih_assert_close(ego.a, cfg.veh.a_max, 1e-12, 'acceleration saturates');
end
