function test_ekf()
%TEST_EKF CTRV motion model, its Jacobian, and filter convergence.
%
%   Three independent things are checked, because they fail in different ways:
%   an analytic Jacobian that disagrees with the motion it linearises makes the
%   covariance quietly wrong (tracks look fine but gating misbehaves); a
%   discontinuity at the yaw-rate branch switch makes tracks jump; and a filter
%   that does not actually beat its own measurements is worse than no filter.

cfg = sih_config();
dt  = 0.1;

% ---- analytic Jacobian vs central differences, exact branch --------------
x = [3.2; -1.7; 8.0; 0.4; 0.25];
[~, F] = sih_ctrv_motion(x, dt);
sih_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (turning)');

x = [0; 0; 12.0; -0.9; -0.6];
[~, F] = sih_ctrv_motion(x, dt);
sih_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (hard turn)');

% ---- analytic Jacobian vs central differences, near-straight branch ------
x = [1.0; 2.0; 6.0; 0.3; 1e-6];
[~, F] = sih_ctrv_motion(x, dt);
sih_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (straight)');

% The turn-rate column must not be zero: the covariance coupling between
% heading and yaw rate has to survive a track passing through omega = 0.
sih_assert_true(abs(F(2,5)) > 1e-9, 'straight branch must retain d(py)/d(omega)');

% ---- continuity across the branch switch at |omega| = 1e-4 --------------
eps_w = 1e-9;
xa = [1.0; 2.0; 6.0; 0.3;  1e-4 + eps_w];   % exact branch
xb = [1.0; 2.0; 6.0; 0.3;  1e-4 - eps_w];   % expansion branch
sih_assert_close(sih_ctrv_motion(xa, dt), sih_ctrv_motion(xb, dt), 1e-8, ...
                 'motion continuous across the omega branch');

% ---- pure straight line: no lateral drift -------------------------------
x = [0; 0; 10; 0; 0];
for k = 1:20
    x = sih_ctrv_motion(x, dt);
end
sih_assert_close(x(1), 10 * 20 * dt, 1e-9, 'CTRV straight distance');
sih_assert_close(x(2), 0, 1e-12, 'CTRV straight has no drift');

% ---- exact circular motion ----------------------------------------------
% With constant v and omega the CTRV model traces a circle of radius v/omega.
v = 8; w = 0.25;
x = [0; 0; v; 0; w];
n = 30;
for k = 1:n
    x = sih_ctrv_motion(x, dt);
end
Rc  = v / w;
psi = w * n * dt;
sih_assert_close(x(1), Rc*sin(psi),     1e-9, 'CTRV circle x');
sih_assert_close(x(2), Rc*(1-cos(psi)), 1e-9, 'CTRV circle y');
sih_assert_close(x(3), v, 1e-12, 'CTRV holds speed');
sih_assert_close(x(5), w, 1e-12, 'CTRV holds turn rate');

% ---- filter convergence on a noisy synthetic track ----------------------
sih_rng(4242);

x_true = [0; 0; 7.0; 0.2; 0.12];
sigma  = 0.5;
R      = sigma^2 * eye(2);
N      = 160;

% Deliberately poor initialisation: position from the first measurement,
% speed and turn rate unknown. This is exactly how sih_tracker_step births a
% track, so convergence from here is the property that matters.
x_est = [x_true(1); x_true(2); 0; 0; 0];
P     = diag([cfg.fuse.p0_pos, cfg.fuse.p0_pos, cfg.fuse.p0_vel, ...
              cfg.fuse.p0_yaw, cfg.fuse.p0_omega]);

err_meas  = zeros(N, 1);
err_pos   = zeros(N, 1);
err_head  = zeros(N, 1);
err_speed = zeros(N, 1);
err_omega = zeros(N, 1);

for k = 1:N
    x_true = sih_ctrv_motion(x_true, dt);
    z = x_true(1:2) + sigma * randn(2, 1);

    [x_est, P] = sih_ekf_predict(x_est, P, dt, cfg);
    [x_est, P, ~, valid] = sih_ekf_update(x_est, P, z, R, []);
    sih_assert_true(valid, 'ungated update must always apply');

    err_meas(k)  = norm(z          - x_true(1:2));
    err_pos(k)   = norm(x_est(1:2) - x_true(1:2));
    err_head(k)  = sih_wrap_pi(x_est(4) - x_true(4));
    err_speed(k) = x_est(3) - x_true(3);
    err_omega(k) = x_est(5) - x_true(5);
end

% Assertions are made on statistics over the settled second half rather than on
% the final sample. A single instant sits roughly one standard deviation away
% from truth by construction, so asserting on it would produce a test that
% fails on an unlucky seed while telling us nothing about the filter. Mean
% absolute error measures accuracy; the mean itself measures bias, which is
% what actually breaks a tracker -- a biased heading estimate steadily walks
% predicted trajectories off to one side.
%
% Thresholds sit about 1.5x above the worst value observed across eight seeds,
% so a genuine regression trips them but sampling noise does not.
w = round(N/2):N;

sih_assert_true(mean(err_pos(w)) < 0.50, ...
    'position MAE %.3f m too high', mean(err_pos(w)));

sih_assert_true(mean(abs(err_head(w))) < 0.13, ...
    'heading MAE %.4f rad too high', mean(abs(err_head(w))));
sih_assert_true(abs(mean(err_head(w))) < 0.04, ...
    'heading bias %.4f rad -- estimator is systematically off', mean(err_head(w)));

sih_assert_true(mean(abs(err_speed(w))) < 0.40, ...
    'speed MAE %.3f m/s too high', mean(abs(err_speed(w))));
sih_assert_true(abs(mean(err_speed(w))) < 0.15, ...
    'speed bias %.3f m/s', mean(err_speed(w)));

sih_assert_true(mean(abs(err_omega(w))) < 0.13, ...
    'turn-rate MAE %.4f rad/s too high', mean(abs(err_omega(w))));
sih_assert_true(abs(mean(err_omega(w))) < 0.03, ...
    'turn-rate bias %.4f rad/s', mean(err_omega(w)));

% The filter must beat its own raw measurements, otherwise it is adding
% latency for nothing.
rms_meas = sqrt(mean(err_meas(w).^2));
rms_est  = sqrt(mean(err_pos(w).^2));
sih_assert_true(rms_est < 0.75 * rms_meas, ...
    'filtered RMS %.3f must improve on measurement RMS %.3f', rms_est, rms_meas);

% Covariance must stay positive definite and symmetric.
sih_assert_close(P, P.', 1e-12, 'covariance symmetric');
sih_assert_true(all(eig(P) > 0), 'covariance positive definite');

% ---- gating rejects an outlier and leaves the state untouched -----------
x_before = x_est;
P_before = P;
z_bad = x_est(1:2) + [40; 40];
[x_after, P_after, nis, valid] = sih_ekf_update(x_est, P, z_bad, R, cfg.fuse.gate_chi2);
sih_assert_true(~valid, 'a 40 m outlier must fail the gate');
sih_assert_true(nis > cfg.fuse.gate_chi2, 'outlier NIS must exceed the gate');
sih_assert_close(x_after, x_before, 0, 'rejected update leaves state unchanged');
sih_assert_close(P_after, P_before, 0, 'rejected update leaves covariance unchanged');

% A measurement on top of the estimate must pass the gate comfortably.
[~, ~, nis_ok, valid_ok] = sih_ekf_update(x_est, P, x_est(1:2), R, cfg.fuse.gate_chi2);
sih_assert_true(valid_ok && nis_ok < 1e-9, 'zero-innovation update must pass the gate');
end

% -------------------------------------------------------------------------
function J = local_num_jac(x, dt)
%LOCAL_NUM_JAC Central-difference Jacobian of the CTRV propagation.
h = 1e-6;
J = zeros(5, 5);
for i = 1:5
    xp = x; xp(i) = xp(i) + h;
    xm = x; xm(i) = xm(i) - h;
    fp = sih_ctrv_motion(xp, dt);
    fm = sih_ctrv_motion(xm, dt);
    J(:, i) = (fp - fm) / (2*h);
end
end
