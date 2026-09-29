function test_frenet()
%TEST_FRENET Cartesian <-> Frenet transforms round-trip.
%
%   Every planned candidate is generated in Frenet coordinates and executed in
%   Cartesian ones, so a bias in either direction would show up as the vehicle
%   tracking a path offset from the one that was collision-checked.

% ---- straight path: transforms should be near exact ----------------------
rp = sih_ref_path([0 0; 60 0], 0.25, 3.0);

s_in = [5; 20; 41.7];
d_in = [0; 2.4; -1.9];
[x, y] = sih_frenet2cart(rp, s_in, d_in);

% Left-positive convention: on an eastbound path, +d must move north.
sih_assert_close(y, d_in, 1e-9, 'straight: +d is to the left');
sih_assert_close(x, s_in, 1e-9, 'straight: s maps to x');

for k = 1:numel(s_in)
    [s_out, d_out] = sih_cart2frenet(rp, x(k), y(k));
    sih_assert_close(s_out, s_in(k), 1e-6, 'straight round-trip s');
    sih_assert_close(d_out, d_in(k), 1e-6, 'straight round-trip d');
end

% ---- curved path: discretisation error must stay small -------------------
Rc = 25;
th = linspace(0, pi/2, 30)';
rp = sih_ref_path([Rc*sin(th), Rc*(1 - cos(th))], 0.20, 3.0);

s_in = [3; 12; 25; 35];
d_in = [0; 1.5; -2.2; 0.8];
[x, y] = sih_frenet2cart(rp, s_in, d_in);

% The two coordinates are held to different tolerances because they converge
% differently. The lateral offset d is limited by the chord-versus-arc error of
% interpolating between path samples and converges cleanly as O(ds^2). The arc
% length s is limited instead by where the query station happens to fall
% between two samples, which is a sampling-phase effect and does not shrink
% smoothly. At ds = 0.2 m that leaves roughly 1 cm of arc-length error on a
% 25 m radius -- two orders of magnitude below the 0.45 m safety margin, so it
% is irrelevant to planning -- while d, which IS safety critical because it
% decides which side of an obstacle the vehicle passes, holds to a millimetre.
for k = 1:numel(s_in)
    [s_out, d_out] = sih_cart2frenet(rp, x(k), y(k));
    sih_assert_close(s_out, s_in(k), 1.5e-2, 'curved round-trip s');
    sih_assert_close(d_out, d_in(k), 1.0e-3, 'curved round-trip d');
end

% Convergence is the real correctness statement: refining the path sampling
% must reduce the error, and roughly quadratically. A method error would show
% up here as a floor that refinement cannot get past.
err_coarse = local_max_d_err(Rc, th, 0.40);
err_fine   = local_max_d_err(Rc, th, 0.05);
sih_assert_true(err_fine < err_coarse / 10, ...
    'lateral error must converge with sampling: %.3e at ds=0.40 vs %.3e at ds=0.05', ...
    err_coarse, err_fine);

% ---- offset heading on a curve ------------------------------------------
% With dprime = 0 the offset curve is parallel to the reference, so its
% heading must equal the reference heading at that station.
[~, ~, psi_off] = sih_frenet2cart(rp, 20, 2.0, 0);
[~, ~, ~, psi_ref] = sih_cart2frenet(rp, ...
    interp1(rp.s, rp.x, 20) - 2.0*sin(interp1(rp.s, rp.psi, 20)), ...
    interp1(rp.s, rp.y, 20) + 2.0*cos(interp1(rp.s, rp.psi, 20)));
sih_assert_close(sih_wrap_pi(psi_off - psi_ref), 0, 5e-3, 'parallel offset heading');
end

% -------------------------------------------------------------------------
function e = local_max_d_err(Rc, th, ds)
%LOCAL_MAX_D_ERR Worst lateral round-trip error at a given path sampling.
rp = sih_ref_path([Rc*sin(th), Rc*(1 - cos(th))], ds, 3.0);
s_in = [3; 12; 25; 35];
d_in = [0; 1.5; -2.2; 0.8];
[x, y] = sih_frenet2cart(rp, s_in, d_in);
e = 0;
for k = 1:numel(s_in)
    [~, d_out] = sih_cart2frenet(rp, x(k), y(k));
    e = max(e, abs(d_out - d_in(k)));
end
end
