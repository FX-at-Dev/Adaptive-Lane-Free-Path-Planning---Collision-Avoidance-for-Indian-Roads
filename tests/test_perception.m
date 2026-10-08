function test_perception()
%TEST_PERCEPTION LiDAR, radar and camera perception on synthetic data.
%
%   A straight road, the ego at its start, and a scan built by hand: a car
%   and a pedestrian on the road, ground everywhere, and beside the road a
%   mapped garden wall, an unmapped long wall and a tree. Perception must
%   find exactly the two road users, near their true centres, and nothing
%   else. Then a car seen only from behind, a stationary radar return off a
%   wall against a moving one, and a camera box projected from a known
%   position.

cfg = sih_config();

% ---- a straight road along x, 3.5 m half width -----------------------------
rp.ds = 0.25;
rp.s = (0:rp.ds:120).';
rp.x = rp.s;
rp.y = zeros(size(rp.s));
rp.psi = zeros(size(rp.s));
rp.halfwidth = 3.5 * ones(size(rp.s));
rp.length = rp.s(end);
rp.kappa = zeros(size(rp.s));
% A mapped garden wall 1.8 m beyond the road edge: [cx cy length width heading height].
wall = [43, 5.3, 6, 0.3, 0, 1.1];
ctx = sih_perception_init(rp, cfg, wall);

ego.x = 10; ego.y = 0; ego.psi = 0; ego.v = 5;
sx = ego.x + cfg.ego.rear_axle_to_centre;
h = cfg.sensor.lidar.mount_height;

P = zeros(0, 3);
P = [P; local_box(30, 1, 4.2, 1.8, 1.5, 0.15)];        % car on the road
P = [P; local_box(20, -2, 0.6, 0.6, 1.7, 0.1)];        % pedestrian on the road
P = [P; local_box(43, 5.3, 6, 0.3, 1.1, 0.15)];        % the mapped wall
P = [P; local_box(70, -5.5, 8, 0.3, 1.3, 0.15)];       % unmapped long wall, 2 m off the road
P = [P; local_box(55, 7, 0.4, 0.4, 2.5, 0.1); local_box(55, 7, 3, 3, 1.5, 0.3) + [0 0 2.5]];   % tree
[gx, gy] = meshgrid(12:0.5:50, -6:0.5:6);
P = [P; gx(:), gy(:), 0.02 * ones(numel(gx), 1)];         % ground
P = [P(:,1) - sx, P(:,2) - ego.y, P(:,3) - h];           % to the sensor frame

[d, info] = sih_lidar_detect(single(P), ego, ctx, cfg);
sih_assert_true(numel(d.x) == 2, sprintf('two road users, not %d (clusters %d)', numel(d.x), info.clusters));
[~, i] = min(hypot(d.x - 30, d.y - 1));
sih_assert_true(hypot(d.x(i) - 30, d.y(i) - 1) < 0.3, 'car centre within 0.3 m');
[~, j] = min(hypot(d.x - 20, d.y + 2));
sih_assert_true(hypot(d.x(j) - 20, d.y(j) + 2) < 0.3, 'pedestrian centre within 0.3 m');
sih_assert_true(all(strcmp(d.sensor, 'lidar')) && all(cellfun(@isempty, d.class)), 'lidar, no class');
% Each object reports the box it saw: the car 4.2 x 1.8 along the road.
sih_assert_true(abs(d.box(i, 1) - 4.2) < 0.4 && abs(d.box(i, 2) - 1.8) < 0.4, 'car box extents');
sih_assert_true(abs(sin(d.box(i, 3))) < 0.1, 'car box along the road');
sih_assert_true(all(~d.box_full) && all(abs(d.road_psi) < 1e-9), 'partial boxes, road direction known');
for k = 1:numel(d.x)
    R = d.R(:,:,k);
    sih_assert_true(all(eig((R + R.') / 2) > 0), 'covariance positive definite');
end

% ---- seen from behind only: found, on the near side ---------------------------
B = local_face(30, 1, 1.8, 1.5);
[d, ~] = sih_lidar_detect(single([B(:,1) - sx, B(:,2), B(:,3) - h]), ego, ctx, cfg);
sih_assert_true(numel(d.x) == 1, 'rear face is one object');
sih_assert_true(d.x < 30 && d.x > 27, 'its centre is between the ego and the true centre (conservative)');

% ---- radar: a still return off the wall goes, a moving car stays ---------------
b_wall = atan2(5.3 - 0.5 - ego.y, 43 - sx);
r_wall = hypot(43 - sx, 4.8);
b_car = atan2(1, 30 - sx);
r_car = hypot(30 - sx, 1);
Rr = [r_wall, b_wall - deg2rad(2.5), -ego.v * cos(b_wall - deg2rad(2.5));     % stationary, scattered onto the road
      r_car,  b_car,                (3 - ego.v) * cos(b_car)];            % moving at 3 m/s
d = sih_radar_detect(single(Rr), ego, ctx, cfg);
sih_assert_true(numel(d.x) == 1, sprintf('radar keeps only the moving car, kept %d', numel(d.x)));
sih_assert_true(hypot(d.x - 30, d.y - 1) < 1.0, 'radar car position');

% ---- camera: a pedestrian box from a known position ---------------------------
C = cfg.sensor.camera;
W = C.image_size(1); H = C.image_size(2);
f = (W / 2) / tan(C.fov / 2);
fwd = 20 - sx; lat = -2;                                  % pedestrian centre, camera frame
near = fwd - 0.3;                                         % its near face
u0 = W/2 - f * (lat + 0.3) / near; u1 = W/2 - f * (lat - 0.3) / near;
v1 = H/2 + f * C.mount_height / near;                     % feet on the road
v0 = H/2 - f * (1.7 - C.mount_height) / near;
d = sih_camera_detect([u0 v0 u1 v1 7 0.9], ego, ctx, cfg);
sih_assert_true(numel(d.x) == 1 && strcmp(d.class{1}, 'pedestrian'), 'camera pedestrian');
sih_assert_true(hypot(d.x - 20, d.y + 2) < 0.6, sprintf('camera position (%.2f, %.2f)', d.x, d.y));
end

% -------------------------------------------------------------------------
function P = local_box(cx, cy, L, W, H, step)
%LOCAL_BOX Points on the four sides and top of an axis-aligned box.
P = zeros(0, 3);
for z = step:step:H
    for x = cx - L/2:step:cx + L/2
        P = [P; x, cy - W/2, z; x, cy + W/2, z]; %#ok<AGROW>
    end
    for y = cy - W/2:step:cy + W/2
        P = [P; cx - L/2, y, z; cx + L/2, y, z]; %#ok<AGROW>
    end
end
end

function P = local_face(cx, cy, W, H)
%LOCAL_FACE Points on the rear face only (the side towards -x).
[y, z] = meshgrid(cy - W/2:0.1:cy + W/2, 0.3:0.1:H);
P = [(cx - 2.1) * ones(numel(y), 1), y(:), z(:)];
end
