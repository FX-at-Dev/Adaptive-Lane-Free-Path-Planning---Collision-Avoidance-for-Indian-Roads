function test_refpath()
%TEST_REFPATH Reference path geometry: arc length, heading and curvature.
%
%   The Frenet frame is only as trustworthy as this path, and a sign error in
%   the curvature would flip which side of the road the planner treats as
%   "inside" a bend. Both a straight line and a known circular arc are checked.

% ---- straight line -------------------------------------------------------
wp = [0 0; 50 0];
rp = sih_ref_path(wp, 0.25, 3.0);

sih_assert_close(rp.length, 50, 1e-9, 'straight path length');
sih_assert_close(rp.psi, zeros(size(rp.psi)), 1e-9, 'straight path heading');
sih_assert_close(rp.kappa, zeros(size(rp.kappa)), 1e-9, 'straight path curvature');
sih_assert_close(diff(rp.s), rp.ds * ones(numel(rp.s)-1, 1), 1e-9, 'uniform ds');

% ---- circular arc, counter-clockwise, radius 20 --------------------------
% A left-hand turn must produce POSITIVE curvature to match the left-positive
% lateral offset convention used by sih_cart2frenet.
Rc  = 20;
th  = linspace(0, pi, 40)';
wp  = [Rc*sin(th), Rc*(1 - cos(th))];
rp  = sih_ref_path(wp, 0.20, 3.0);

sih_assert_close(rp.length, Rc*pi, 0.05, 'arc length of half circle');

% Skip the first and last few samples: gradient() is one-sided at the ends and
% the spline is least constrained there.
m = 8:(numel(rp.kappa) - 8);
sih_assert_close(rp.kappa(m), (1/Rc) * ones(numel(m), 1), 2e-3, 'arc curvature');

% Heading is unwrapped, so it must sweep monotonically from 0 to pi.
sih_assert_close(rp.psi(1), 0, 1e-2, 'arc initial heading');
sih_assert_close(rp.psi(end), pi, 2e-2, 'arc final heading');
sih_assert_true(all(diff(rp.psi) > -1e-6), 'unwrapped heading must be monotonic on a CCW arc');

% ---- clockwise arc must flip the curvature sign --------------------------
wp = [Rc*sin(th), -Rc*(1 - cos(th))];
rp = sih_ref_path(wp, 0.20, 3.0);
m  = 8:(numel(rp.kappa) - 8);
sih_assert_close(rp.kappa(m), (-1/Rc) * ones(numel(m), 1), 2e-3, 'CW arc curvature sign');
end
