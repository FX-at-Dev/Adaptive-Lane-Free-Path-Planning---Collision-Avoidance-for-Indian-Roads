function [bp, info] = sih_arrive(bp, ego, goal, rp, cfg)
%SIH_ARRIVE Come to a stop at the target point, and only there.
%
%   [bp, info] = SIH_ARRIVE(bp, ego, goal, rp, cfg) caps the speed the
%   behaviour layer allows (bp.v_cap) so the vehicle brakes smoothly and
%   stops with its centre at the target goal = [x y]: the target's station
%   along the road, in the vehicle's own lane. Every other stop is the
%   behaviour layer's, for something actually in the way.
%
%   The cap is the speed from which a comfortable deceleration
%   (cfg.dec.arrive_decel) stops the vehicle at the target, aimed a little
%   short of it (cfg.dec.arrive_lead) because the planner reaches a new speed
%   over its horizon, not at once.
%
%   info.dist   [m] how far the vehicle's centre is short of the target
%               along the road (negative once past it)
%   info.active true once the cap is binding

info.dist = Inf;
info.active = false;
if isempty(goal) || any(~isfinite(goal))
    return;
end
s_goal = sih_cart2frenet(rp, goal(1), goal(2));
s_ego = sih_cart2frenet(rp, ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre, ...
                            ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre);
info.dist = s_goal - s_ego;

d = max(info.dist - cfg.dec.arrive_lead, 0);
v_stop = sqrt(2 * cfg.dec.arrive_decel * d);
if info.dist <= cfg.dec.arrive_tol
    v_stop = 0;
end
if v_stop < bp.v_cap
    bp.v_cap = v_stop;
    info.active = true;
    if v_stop == 0
        bp.reason = sprintf('at the target (%.1f m)', info.dist);
    else
        bp.reason = sprintf('%s; stopping at the target, %.0f m ahead', bp.reason, info.dist);
    end
end
end
