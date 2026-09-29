function [s, d, kappa_ref, psi_ref] = sih_cart2frenet(rp, x, y)
%SIH_CART2FRENET Project a Cartesian point onto the reference path.
%
%   [s,d,kappa_ref,psi_ref] = SIH_CART2FRENET(rp, x, y) returns arc length s
%   of the closest point on rp, signed lateral offset d (positive to the LEFT
%   of the direction of travel), and the reference curvature and heading at
%   that station.
%
%   The nearest sample is found by brute force, then refined by projecting
%   onto the local tangent. With rp.ds around 0.25 m that first-order
%   refinement is accurate to well under a millimetre on realistic curvature.

dx = rp.x - x;
dy = rp.y - y;
[~, i] = min(dx.^2 + dy.^2);

ps = rp.psi(i);
ct = cos(ps);
st = sin(ps);

% Vector from the sampled reference point to the query point.
ex = x - rp.x(i);
ey = y - rp.y(i);

% Tangential component refines s; normal component is the lateral offset.
s_off = ct * ex + st * ey;
d     = -st * ex + ct * ey;

s = rp.s(i) + s_off;
s = min(max(s, 0), rp.length);

kappa_ref = rp.kappa(i);
psi_ref   = ps;
end
