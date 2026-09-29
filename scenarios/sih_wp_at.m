function [x, y] = sih_wp_at(rp, s, d)
%SIH_WP_AT One point on a corridor, given arc length and lateral offset.
%   [x,y] = SIH_WP_AT(rp, s, d) places a point d metres to the left of the
%   corridor centreline at station s, clamped to the corridor's extent.
%
%   Scenarios are authored in corridor coordinates rather than world ones
%   because that is how the situations are actually described: "an oncoming
%   rickshaw 1 m to the right of centre, 190 m up the road". Converting here
%   keeps the scenario files readable and lets a road be reshaped without
%   every agent having to be repositioned by hand.
s = min(max(s, 0), rp.length);
[x, y] = sih_frenet2cart(rp, s, d);
end
