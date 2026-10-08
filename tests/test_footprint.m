function test_footprint()
%TEST_FOOTPRINT Objects are where they are, face the way they face, and are
%   in the path only when they are in the path.
%
%   1. A parked truck seen only from behind -- LiDAR boxes of its rear face,
%      the camera calling it a truck -- must end up as a still footprint at
%      its true centre, along the road, 8.5 m long: not a 2.5 m box four
%      metres too close, and not a box that turns with the filter's heading.
%   2. On a bend, an object beside the road that is straight ahead of the
%      ego's nose is not in its path; the same object on the road is.

cfg = sih_config();

% ---- 1. the parked truck ----------------------------------------------------
truck = [40, -2];                        % centre; it lies along +x
L = 8.5; W = 2.5;
sensor = [21.35, 0];                     % the ego's centre
tracks = [];
next_id = 1;
for k = 1:60
    face_x = truck(1) - L / 2;
    dets = sih_dets_empty();
    % LiDAR: a thin line of returns across the rear, a little noise.
    dets.x(1,1) = face_x + 0.03 * sin(k);
    dets.y(1,1) = truck(2) + 0.05 * cos(1.7 * k);
    dets.R(:,:,1) = diag([0.1, 0.4].^2);
    dets.class{1,1} = '';
    dets.sensor{1,1} = 'lidar';
    dets.truth_id(1,1) = 0;
    dets.box(1,:) = [W, 0.12, pi / 2 + 0.03 * sin(2.3 * k)];
    dets.box_full(1,1) = false;
    dets.origin(1,:) = sensor;
    dets.road_psi(1,1) = 0;
    dets.surface(1,1) = false;
    % Camera: a truck, poor depth, roughly at the centre.
    dets.x(2,1) = truck(1) + 0.8 * sin(0.7 * k);
    dets.y(2,1) = truck(2) + 0.1 * cos(k);
    dets.R(:,:,2) = diag([1.6, 0.2].^2);
    dets.class{2,1} = 'truck';
    dets.sensor{2,1} = 'camera';
    dets.truth_id(2,1) = 0;
    dets.box(2,:) = [NaN, NaN, NaN];
    dets.box_full(2,1) = false;
    dets.origin(2,:) = sensor;
    dets.road_psi(2,1) = NaN;
    dets.surface(2,1) = false;
    [tracks, next_id] = sih_tracker_step(tracks, dets, cfg.sim.dt, next_id, cfg);
end
conf = tracks(~strcmp({tracks.status}, 'tentative'));
sih_assert_true(numel(conf) == 1, sprintf('one truck, not %d tracks', numel(conf)));
fp = sih_footprint(conf(1), cfg);
sih_assert_true(fp.still, 'the parked truck is still');
sih_assert_true(hypot(fp.cx - truck(1), fp.cy - truck(2)) < 0.6, ...
    sprintf('truck centre at (%.2f, %.2f), true (40, -2)', fp.cx, fp.cy));
sih_assert_true(abs(sin(fp.theta)) < 0.1, sprintf('truck along the road, theta %.2f', fp.theta));
sih_assert_true(abs(fp.L - L) < 0.6, sprintf('truck length %.2f', fp.L));

% ---- 2. in the path, measured along a bend ------------------------------------
rp = sih_ref_path([0 0; 30 0; 50 15; 60 40], 0.25, 3.2);
ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 6, 'delta', 0, 'a', 0);
% Beside the road on the outside of the bend, where the ego's nose points.
[ox, oy] = sih_frenet2cart(rp, 42, -6.0);
[ix, iy] = sih_frenet2cart(rp, 42, 0.0);
off_road = local_track(1, ox, oy, cfg);
on_road  = local_track(2, ix, iy, cfg);
ra = sih_risk_assess(ego, off_road, rp, cfg);
sih_assert_true(isempty(ra.in_path) && ra.lead_id == 0, 'beside the road on the bend: not in the path');
ra = sih_risk_assess(ego, on_road, rp, cfg);
sih_assert_true(any(ra.in_path == 2) && ra.lead_id == 2, 'on the road on the bend: in the path');
end

% -------------------------------------------------------------------------
function t = local_track(id, x, y, cfg)
%LOCAL_TRACK A confirmed, still pushcart at (x, y) whose filter heading is noise.
t.id = id;
t.x = [x; y; 0.05; 2.1; 0.3];            % heading pointing anywhere
t.P = 0.05 * eye(5);
t.class = 'pushcart';
t.cls_names = {'pushcart'};
t.cls_counts = 5;
t.shape = [2.0, 1.2, 0.0];
t.shape_n = 10;
t.still = true;
t.slow_t = 5;
t.road_psi = 0;
t.hits = 20; t.misses = 0; t.age = 20;
t.status = 'confirmed';
t.hist = true(1, 4);
t.last_z = [x; y];
t.z_age = 0;
t.seen = true;
end
