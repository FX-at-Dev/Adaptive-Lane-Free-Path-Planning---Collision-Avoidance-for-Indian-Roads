function [dets, info] = sih_lidar_detect(P, ego, ctx, cfg)
%SIH_LIDAR_DETECT Objects from a raw LiDAR scan.
%
%   [dets, info] = SIH_LIDAR_DETECT(P, ego, ctx, cfg)
%
%   P is the scan as returned by the sensor: one row per return, [x y z] in
%   metres in the sensor frame (x forward, y left, z up), the sensor sitting
%   at the ego's geometric centre at cfg.sensor.lidar.mount_height. ego is the
%   pose the scan was taken from (rear axle, as everywhere in the stack).
%
%   The pipeline is the classic one, vectorised throughout:
%     1. to the world frame; drop ground (below cfg.percep.ground_height) and
%        overhead returns
%     2. keep returns inside the corridor zones of ctx (sih_perception_init)
%        and off the static map (walls, houses, trees, poles)
%     3. cluster on a bird's-eye grid: occupied cells, 8-connected
%     4. fit each cluster's oriented box to its outline (L-shape fitting)
%     5. near the road keep every cluster; in the wide margin keep only those
%        the size of a person or an animal, which drops walls, trees, houses
%        and poles without a map of them
%   Each object becomes a detection at the centre of the box of what was seen,
%   with that box (dets.box: extents and the axis of the longer one), the
%   sensor position and the road direction. A LiDAR sees only the faces
%   turned towards it; the tracker (sih_track_shape) decides from these which
%   faces they were and moves the centre to the object's real one.
%
%   dets is in the form sih_sense returns (sensor 'lidar', no class).
%   info reports point and cluster counts, for display.

dets = sih_dets_empty();
info = struct('points', size(P, 1), 'kept', 0, 'clusters', 0, 'objects', 0, 'kept_pts', zeros(0, 4, 'single'));
if isempty(P)
    return;
end
p = double(P);
pc = cfg.percep;

% ---- 1. world frame, ground and overhead removed ---------------------------
c = cos(ego.psi);
s = sin(ego.psi);
bx = p(:,1) + cfg.ego.rear_axle_to_centre;
by = p(:,2);
z  = p(:,3) + cfg.sensor.lidar.mount_height;
wx = ego.x + c .* bx - s .* by;
wy = ego.y + s .* bx + c .* by;
keep = z > pc.ground_height & z < pc.max_height;

% ---- 2. corridor zones ----------------------------------------------------------
zone = zeros(size(wx));
zone(keep) = sih_roi(ctx, wx(keep), wy(keep));
keep = keep & zone > 0;
keep(keep) = ~sih_on_map(ctx, wx(keep), wy(keep));
wx = wx(keep); wy = wy(keep); z = z(keep); zone = zone(keep);
info.kept = numel(wx);
info.kept_pts = single([wx, wy, z, zone]);     % for recording and replay

% ---- 3.-5. objects from what is left (sih_lidar_objects) -------------------
[dets, o] = sih_lidar_objects(wx, wy, z, zone, ego, ctx, cfg);
info.clusters = o.clusters;
info.objects = o.objects;
end

