function dets = sih_camera_detect(B, ego, ctx, cfg)
%SIH_CAMERA_DETECT Objects from a camera detector's 2D boxes.
%
%   dets = SIH_CAMERA_DETECT(B, ego, ctx, cfg)
%
%   B has one row per box the image detector reported: [u0 v0 u1 v1 class
%   score], pixels from the top-left of a cfg.sensor.camera.image_size image,
%   class an index into sih_class_names. The camera is a pinhole with
%   horizontal field of view cfg.sensor.camera.fov, at the ego's geometric
%   centre, facing forward, level, cfg.sensor.camera.mount_height above a
%   flat road.
%
%   Bearing comes from the box centre and is sharp. Range comes from where
%   the box meets the road -- the ray through its bottom edge hits the ground
%   plane -- and when that edge is too close to the horizon to trust, from
%   the box height and the class's typical height. Either way depth is the
%   camera's weakness, and the covariance says so: the range error grows
%   with range (cfg.sensor.camera.sigma_range), as in sih_sense. The range
%   is to the object's near side (dets.surface); the tracker, which knows the
%   object's shape, moves it to the centre.
%
%   A box cut off by the edge of the image does not show the whole object.
%   Cut off at the bottom (something close and big), its bottom edge is the
%   image's, not the road contact: the class height is used instead, or the
%   box is dropped if its top is cut off too. Cut off at a side, its centre
%   is not the object's bearing: it is dropped (the LiDAR covers the sides).
%
%   dets is in the form sih_sense returns (sensor 'camera', with class).

dets = sih_dets_empty();
if isempty(B)
    return;
end
B = double(B);
sp = cfg.sensor.camera;
W = sp.image_size(1); H = sp.image_size(2);
f = (W / 2) / tan(sp.fov / 2);               % square pixels
cxp = W / 2; cyp = H / 2;
h = sp.mount_height;
names = sih_class_names();
heights = [1.5 3.2 3.0 1.85 1.55 1.65 1.7 1.45 1.3 1.3];

um = (B(:,1) + B(:,3)) / 2;
left = (cxp - um) / f;                        % tan of the bearing, left positive
down = (B(:,4) - cyp) / f;                    % tan of the bottom edge below the horizon
ci = min(max(round(B(:,5)), 1), numel(names));

% Range along the optical axis: ground plane where it is well conditioned,
% class height otherwise.
fwd_ground = h ./ max(down, 1e-6);
fwd_height = heights(ci).' * f ./ max(B(:,4) - B(:,2), 1);
edge = cfg.percep.camera_edge_px;
cut_bottom = B(:,4) >= H - edge;
cut_top = B(:,2) <= edge;
cut_side = B(:,1) <= edge | B(:,3) >= W - edge;
valid = ~cut_side & ~(cut_bottom & cut_top);
use_ground = down > tan(deg2rad(1.0)) & ~cut_bottom;
fwd = fwd_height;
fwd(use_ground) = fwd_ground(use_ground);
lat = fwd .* left;
rng = hypot(fwd, lat);

% The ground point is the near edge of the object, not its centre. How far
% the centre lies beyond it depends on the object's size and which way it
% faces -- half a car's width side-on, half a bus's length end-on -- which
% the tracker knows and this does not: the detection is marked as a near-
% surface measurement and the tracker moves it (sih_track_shape).

c = cos(ego.psi); s = sin(ego.psi);
sx = ego.x + c * cfg.ego.rear_axle_to_centre;
sy = ego.y + s * cfg.ego.rear_axle_to_centre;
x = sx + c * fwd - s * lat;
y = sy + s * fwd + c * lat;
keep = valid & rng <= sp.range & sih_roi(ctx, x, y) > 0;
idx = find(keep);
m = numel(idx);
dets.x = x(idx);
dets.y = y(idx);
dets.R = zeros(2, 2, m);
for i = 1:m
    k = idx(i);
    b = atan2(y(k) - sy, x(k) - sx);
    sr = sp.sigma_range * rng(k);
    Rot = [cos(b), -sin(b); sin(b), cos(b)];
    dets.R(:,:,i) = Rot * diag([sr^2, (rng(k) * sp.sigma_bear)^2]) * Rot.';
end
dets.surface = true(m, 1);
dets.origin = repmat([sx, sy], m, 1);
dets.road_psi = NaN(m, 1);              % the road's direction there
if isfield(ctx, 'rp')
    for i = 1:m
        [~, ~, ~, dets.road_psi(i)] = sih_cart2frenet(ctx.rp, dets.x(i), dets.y(i));
    end
end
dets.class = reshape(names(ci(idx)), [], 1);
dets.sensor = repmat({'camera'}, m, 1);
dets.truth_id = zeros(m, 1);
end
