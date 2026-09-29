function [cx, cy, r] = sih_ego_discs(x, y, psi, cfg)
%SIH_EGO_DISCS Cover the ego footprint with discs for fast collision checks.
%
%   [cx, cy, r] = SIH_EGO_DISCS(x, y, psi, cfg) returns the centres of
%   cfg.ego.n_discs discs and the single radius r that guarantees the discs
%   cover the rectangular footprint. x, y, psi may be column vectors, in which
%   case cx and cy are [numel(x) x n_discs].
%
%   Disc covering is used instead of exact rectangle intersection because the
%   planner evaluates on the order of a hundred candidate trajectories per
%   cycle against dozens of predicted agent hypotheses; a distance comparison
%   between discs is a handful of flops, and the covering is conservative, so
%   a candidate this test clears is genuinely clear.
%
%   The pose (x, y, psi) is the REAR AXLE, matching sih_bicycle_step.

n = cfg.ego.n_discs;
L = cfg.ego.length;
W = cfg.ego.width;

% Footprint spans from the rear overhang to the front bumper, measured from
% the rear axle. Wheelbase plus a front overhang makes up the front section.
rear_overhang = 0.5 * (L - cfg.ego.wheelbase);
x_back  = -rear_overhang;
x_front =  L - rear_overhang;

% Disc i covers a longitudinal slice of length L/n; the covering radius is the
% half-diagonal of that slice.
seg = L / n;
r   = sqrt((seg/2)^2 + (W/2)^2);

x = x(:); y = y(:); psi = psi(:);
offs = x_back + seg * ((1:n) - 0.5);       % 1 x n longitudinal disc centres

c = cos(psi);
s = sin(psi);

cx = x * ones(1, n) + c * offs;
cy = y * ones(1, n) + s * offs;
end
