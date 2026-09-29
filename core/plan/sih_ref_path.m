function rp = sih_ref_path(wp, ds, halfwidth)
%SIH_REF_PATH Build an arc-length parameterised reference path from waypoints.
%
%   rp = SIH_REF_PATH(wp, ds, halfwidth) takes an Nx2 waypoint polyline and
%   returns a struct sampled uniformly in arc length with fields:
%       .s        [M x 1] arc length, uniform spacing ds
%       .x, .y    [M x 1] position
%       .psi      [M x 1] heading, unwrapped (continuous, may exceed +/-pi)
%       .kappa    [M x 1] signed curvature [1/m]
%       .ds       scalar sample spacing
%       .length   total path length
%       .halfwidth [M x 1] drivable corridor half-width at each station
%
%   On unstructured roads this is deliberately NOT a lane centreline. It is a
%   coarse drivable-corridor reference: the planner is free to depart from it
%   by up to +/-halfwidth, and does so routinely. Waypoints are splined so the
%   reference stays curvature-continuous even when the input polyline has
%   sharp corners, which keeps the Frenet frame well defined.

if nargin < 2 || isempty(ds),        ds = 0.25;  end
if nargin < 3 || isempty(halfwidth), halfwidth = 3.0; end

wp = double(wp);
if size(wp, 2) ~= 2 || size(wp, 1) < 2
    error('sih_ref_path:badInput', 'wp must be Nx2 with at least 2 rows.');
end

% Drop repeated points; they make the chord parameterisation singular.
keep = [true; sum(diff(wp, 1, 1).^2, 2) > 1e-12];
wp   = wp(keep, :);
n    = size(wp, 1);
if n < 2
    error('sih_ref_path:degenerate', 'wp collapsed to a single point.');
end

% ---- pass 1: dense resample in the chord parameter -----------------------
seg    = sqrt(sum(diff(wp, 1, 1).^2, 2));
t_raw  = [0; cumsum(seg)];
L_est  = t_raw(end);

n_dense = max(400, ceil(4 * L_est / ds));
tq      = linspace(0, L_est, n_dense)';

if n >= 3
    method = 'spline';   % curvature-continuous through interior waypoints
else
    method = 'linear';   % two points define a straight line; spline is moot
end
xd = interp1(t_raw, wp(:,1), tq, method);
yd = interp1(t_raw, wp(:,2), tq, method);

% ---- pass 2: reparameterise the dense curve by true arc length -----------
sd = [0; cumsum(sqrt(diff(xd).^2 + diff(yd).^2))];
L  = sd(end);

% Exactly uniform spacing, with ds adjusted to divide L evenly. Appending a
% short final interval instead would leave the grid non-uniform, and
% sih_frenet2cart depends on uniformity to index by arithmetic rather than by
% interp1 -- which is the difference between a planner cycle inside its latency
% budget and one several times over it.
n_s = max(3, ceil(L / ds) + 1);
s   = linspace(0, L, n_s)';
ds  = s(2) - s(1);

% Duplicate arc-length values would break interp1; they only arise from
% coincident dense samples on a degenerate segment.
[sd_u, iu] = unique(sd);
x = interp1(sd_u, xd(iu), s, 'linear');
y = interp1(sd_u, yd(iu), s, 'linear');

% ---- differential geometry ----------------------------------------------
% Uniform spacing in s lets gradient() act as a centred difference operator.
dx  = gradient(x, ds);
dy  = gradient(y, ds);
ddx = gradient(dx, ds);
ddy = gradient(dy, ds);

psi = atan2(dy, dx);
psi = unwrap(psi);   % keep continuous so interpolation in s is well behaved

denom = (dx.^2 + dy.^2).^1.5;
denom(denom < 1e-12) = 1e-12;
kappa = (dx .* ddy - dy .* ddx) ./ denom;

rp.s         = s;
rp.x         = x;
rp.y         = y;
rp.psi       = psi;
rp.kappa     = kappa;
rp.ds        = ds;
rp.length    = L;
if isscalar(halfwidth)
    rp.halfwidth = halfwidth * ones(size(s));
else
    rp.halfwidth = interp1(linspace(0, L, numel(halfwidth))', halfwidth(:), s, 'linear');
end
end
