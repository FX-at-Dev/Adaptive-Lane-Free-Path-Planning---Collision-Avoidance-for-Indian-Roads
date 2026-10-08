function fp = sih_footprint(t, cfg)
%SIH_FOOTPRINT Where a tracked object is, which way it faces and how big it is.
%
%   fp = SIH_FOOTPRINT(t, cfg) for one track t returns
%       .cx .cy    centre, world frame [m]
%       .theta     orientation of its long axis [rad]
%       .L .W      length and width [m]
%       .still     true when it is not moving (t.still)
%       .vx .vy    velocity [m/s], zero when still
%       .n         discs covering it, spaced along theta ...
%       .ox .oy    ... their centres [1 x n]
%       .r         ... and their common radius
%       .corners   4 x 2, world frame
%
%   This is the one place a track becomes geometry. Risk assessment, the
%   predictor, the planner's collision check and the 3D view all use it, so
%   they cannot disagree about where an object is.
%
%   The orientation is the measured outline (the LiDAR's box) once a few
%   boxes have settled it, whether the object moves or not. Before that it is
%   the motion heading while the object is moving and the track is mature,
%   otherwise the outline so far, or the direction of the road at the
%   object, and only as a last resort the filter's heading: for a parked
%   object that heading is noise, and a footprint turned by it spins on the
%   spot.
%
%   The size is the measured shape where there is one, the class's typical
%   footprint otherwise.

props = sih_agent_props(t.class);
fp.cx = t.x(1);
fp.cy = t.x(2);
fp.still = isfield(t, 'still') && t.still;
has_shape = isfield(t, 'shape') && ~isempty(t.shape) && all(isfinite(t.shape));
has_road = isfield(t, 'road_psi') && ~isempty(t.road_psi) && isfinite(t.road_psi);
moving = ~fp.still && t.x(3) >= cfg.track.v_moving && t.hits >= 3;

settled = has_shape && isfield(t, 'shape_n') && t.shape_n >= cfg.track.shape_settled;
if settled
    % The measured outline, moving or not: the filter's heading wanders by
    % tens of degrees at walking pace, the outline does not. Turned to face
    % the way it is going.
    fp.theta = t.shape(3);
    if moving && cos(t.x(4) - fp.theta) < 0
        fp.theta = sih_wrap_pi(fp.theta + pi);
    end
elseif moving || ~(has_shape || has_road)
    fp.theta = t.x(4);
elseif has_shape
    fp.theta = t.shape(3);
else
    fp.theta = t.road_psi;
end

if has_shape
    fp.L = max(t.shape(1), 0.3);
    fp.W = max(t.shape(2), 0.3);
    % Discs close enough that the row bulges at most 0.15 m past the sides.
    fp.n = max(1, min(cfg.track.max_discs, ceil(fp.L / (2 * sqrt(0.15 * (fp.W + 0.15))))));
    fp.r = sqrt((fp.L / (2 * fp.n))^2 + (fp.W / 2)^2);
else
    fp.L = props.length;
    fp.W = props.width;
    fp.n = props.n_discs;
    fp.r = props.radius;
end

if fp.still
    fp.vx = 0; fp.vy = 0;
else
    fp.vx = t.x(3) * cos(t.x(4));
    fp.vy = t.x(3) * sin(t.x(4));
end

c = cos(fp.theta); s = sin(fp.theta);
offs = -fp.L / 2 + (fp.L / fp.n) * ((1:fp.n) - 0.5);
fp.ox = fp.cx + c * offs;
fp.oy = fp.cy + s * offs;
hl = fp.L / 2; hw = fp.W / 2;
u = [hl, hl, -hl, -hl]; v = [hw, -hw, -hw, hw];
fp.corners = [fp.cx + c * u - s * v; fp.cy + s * u + c * v].';
end
