function [x, y, psi] = sih_frenet2cart(rp, s, d, dprime)
%SIH_FRENET2CART Map Frenet coordinates back to the Cartesian frame.
%
%   [x,y,psi] = SIH_FRENET2CART(rp, s, d, dprime) places a point at arc length
%   s and signed lateral offset d (positive left). Optional dprime = dd/ds
%   yields the true path heading of the offset curve; without it, psi is just
%   the reference heading.
%
%   s and d may be vectors of equal length, which is how whole candidate
%   trajectories are converted in one call.
%
%   The reference path is sampled uniformly in arc length (guaranteed by
%   sih_ref_path), so the station lookup is index arithmetic plus a linear
%   blend rather than four interp1 calls. The planner converts on the order of
%   a hundred candidates per cycle, and interp1's per-call overhead dominated
%   the cycle time when this was written the obvious way.

s = s(:);
d = d(:);
if nargin < 4 || isempty(dprime)
    dprime = zeros(size(s));
else
    dprime = dprime(:);
end

N  = numel(rp.s);
ds = rp.ds;

sc = min(max(s, 0), rp.length);

% Index of the sample at or before each station, and the blend factor to the
% next one. Clamped so the final station interpolates within the last cell.
i0 = floor(sc / ds) + 1;
i0 = min(max(i0, 1), N - 1);
i1 = i0 + 1;
w  = (sc - (i0 - 1) * ds) / ds;

rx  = rp.x(i0)     .* (1 - w) + rp.x(i1)     .* w;
ry  = rp.y(i0)     .* (1 - w) + rp.y(i1)     .* w;
rps = rp.psi(i0)   .* (1 - w) + rp.psi(i1)   .* w;   % rp.psi is unwrapped
rk  = rp.kappa(i0) .* (1 - w) + rp.kappa(i1) .* w;

% Left normal of the reference is [-sin(psi), cos(psi)].
x = rx - d .* sin(rps);
y = ry + d .* cos(rps);

% Heading of the offset curve: the lateral rate dd/ds tilts it away from the
% reference, and the (1 - kappa*d) factor accounts for the offset curve being
% stretched or compressed on the inside or outside of a bend.
one_minus_kd = 1 - rk .* d;
one_minus_kd(abs(one_minus_kd) < 1e-6) = 1e-6;
psi = sih_wrap_pi(rps + atan2(dprime, one_minus_kd));
end
