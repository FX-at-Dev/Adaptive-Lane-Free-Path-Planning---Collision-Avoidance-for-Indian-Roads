function d = sih_lane_centre(halfwidth, cfg)
%SIH_LANE_CENTRE Lateral offset of the lane the vehicle keeps to.
%
%   d = SIH_LANE_CENTRE(halfwidth, cfg) for a road of half-width halfwidth
%   (metres from the reference line to its edge) returns the offset of the
%   centre of the left lane (d is positive to the left), or 0 for a road
%   too narrow for two lanes, which is driven down the middle. India drives
%   on the left.
d = 0;
if isfield(cfg.plan, 'keep_left') && cfg.plan.keep_left && 2 * halfwidth >= cfg.plan.two_lane_width
    d = halfwidth / 2;
end
end
