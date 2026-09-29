function test_poly()
%TEST_POLY Quintic and quartic primitives satisfy their boundary conditions.
%
%   These polynomials are the backbone of every lattice candidate, so an error
%   in the coefficient solve would silently distort every planned trajectory
%   rather than throwing. The boundary conditions are therefore checked
%   exactly rather than by eyeballing a plot.

tol = 1e-9;

% ---- quintic: all six boundary conditions constrained --------------------
x0 = -1.3; v0 = 2.1; a0 = 0.4;
x1 =  2.7; v1 = -0.8; a1 = -0.25;
T  = 3.2;

c = sih_quintic(x0, v0, a0, x1, v1, a1, T);
sih_assert_true(numel(c) == 6, 'quintic must return 6 coefficients, got %d', numel(c));

[p, v, a] = sih_polyder_eval(c, [0; T]);
sih_assert_close(p(1), x0, tol, 'quintic x(0)');
sih_assert_close(v(1), v0, tol, 'quintic v(0)');
sih_assert_close(a(1), a0, tol, 'quintic a(0)');
sih_assert_close(p(2), x1, tol, 'quintic x(T)');
sih_assert_close(v(2), v1, tol, 'quintic v(T)');
sih_assert_close(a(2), a1, tol, 'quintic a(T)');

% ---- quartic: terminal position deliberately free ------------------------
c = sih_quartic(x0, v0, a0, v1, a1, T);
sih_assert_true(numel(c) == 5, 'quartic must return 5 coefficients, got %d', numel(c));

[p, v, a] = sih_polyder_eval(c, [0; T]);
sih_assert_close(p(1), x0, tol, 'quartic x(0)');
sih_assert_close(v(1), v0, tol, 'quartic v(0)');
sih_assert_close(a(1), a0, tol, 'quartic a(0)');
sih_assert_close(v(2), v1, tol, 'quartic v(T)');
sih_assert_close(a(2), a1, tol, 'quartic a(T)');

% ---- derivative chain agrees with finite differences ---------------------
% Guards against an indexing slip in sih_polyder_eval that would leave the
% cost function integrating the wrong quantity.
h  = 1e-6;
t0 = 1.1;
[pm, vm, am] = sih_polyder_eval(c, t0 - h);
[pp, vp, ap] = sih_polyder_eval(c, t0 + h);
[~,  vc, ac, jc] = sih_polyder_eval(c, t0);

sih_assert_close((pp - pm) / (2*h), vc, 1e-5, 'dp/dt == v');
sih_assert_close((vp - vm) / (2*h), ac, 1e-5, 'dv/dt == a');
sih_assert_close((ap - am) / (2*h), jc, 1e-4, 'da/dt == j');
end
