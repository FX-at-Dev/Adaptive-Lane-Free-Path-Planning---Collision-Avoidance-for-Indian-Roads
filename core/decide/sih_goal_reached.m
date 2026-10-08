function [reached, dist] = sih_goal_reached(ego, goal, rp, cfg)
%SIH_GOAL_REACHED Has the vehicle stopped at the target?
%
%   [reached, dist] = SIH_GOAL_REACHED(ego, goal, rp, cfg) is true once the
%   vehicle's centre is within cfg.dec.goal_tol of the target's station along
%   the road (or past it) and it has come to rest (sih_arrive brings it there). dist is
%   how far short of the target it is, along the road.
s_goal = sih_cart2frenet(rp, goal(1), goal(2));
s_ego = sih_cart2frenet(rp, ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre, ...
                            ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre);
dist = s_goal - s_ego;
% Short of it by no more than goal_tol, or past it: a car that rolled a few
% metres beyond the target has arrived, not failed -- counting only +-3 m
% left one stopped 3.2 m past it waiting for ever.
reached = dist <= cfg.dec.goal_tol && ego.v < cfg.dec.arrive_v;
end
