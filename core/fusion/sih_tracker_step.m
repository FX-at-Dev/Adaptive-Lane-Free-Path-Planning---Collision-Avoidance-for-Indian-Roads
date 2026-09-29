function [tracks, next_id, diag] = sih_tracker_step(tracks, dets, dt, next_id, cfg)
%SIH_TRACKER_STEP One cycle of multi-sensor multi-object tracking.
%
%   [tracks, next_id, diag] = SIH_TRACKER_STEP(tracks, dets, dt, next_id, cfg)
%
%   Structure of one cycle:
%     1. predict every track forward by dt
%     2. associate and update SEQUENTIALLY, one sensor at a time
%     3. birth new tracks from detections no sensor could associate, spatially
%        clustered so one new object yields one new track
%     4. apply M-of-N track management once per scan
%
%   The sequential sensor update in step 2 is the important structural choice.
%   Association is one-to-one, so pooling all three sensors into a single
%   assignment problem lets only ONE of them update a given track and leaves
%   the other two looking like new objects -- which births a duplicate track
%   per sensor per target. Updating sensor by sensor lets a camera, a radar
%   and a LiDAR return all fold into the same track in the same scan, which is
%   the entire point of carrying three of them.
%
%   tracks is a struct array with fields id, x (5x1 CTRV state), P, class,
%   hits, misses, age, status ('tentative' | 'confirmed' | 'coasting').

if isempty(tracks)
    tracks = local_empty_track_array();
end

nd = numel(dets.x);

% ---- 1. predict ----------------------------------------------------------
for i = 1:numel(tracks)
    [tracks(i).x, tracks(i).P] = sih_ekf_predict(tracks(i).x, tracks(i).P, dt, cfg);
    tracks(i).age   = tracks(i).age + 1;
    tracks(i).z_age = tracks(i).z_age + 1;      % scans since last measurement
    tracks(i).seen  = false;                    % updated by any sensor yet?
end

% ---- 2. sequential per-sensor association and update ---------------------
sensor_names = {'camera', 'radar', 'lidar'};
leftover = false(nd, 1);

for si = 1:numel(sensor_names)
    idx = find(strcmp(dets.sensor, sensor_names{si}));
    if isempty(idx)
        continue;
    end
    [tracks, unassigned] = local_associate(tracks, dets, idx, dt, cfg);
    leftover(unassigned) = true;
end

% ---- 3. birth ------------------------------------------------------------
% Cluster the leftovers first: a genuinely new object is usually seen by more
% than one sensor in the same scan, and birthing one track per detection would
% reintroduce exactly the duplication that step 2 exists to avoid.
groups = local_cluster(dets, find(leftover));
for g = 1:numel(groups)
    tracks(end+1) = local_birth(next_id, dets, groups{g}); %#ok<AGROW>
    next_id = next_id + 1;
end

% ---- 4. management, once per scan ---------------------------------------
keep = true(1, numel(tracks));
for i = 1:numel(tracks)
    t = tracks(i);

    if t.seen
        t.hits   = t.hits + 1;
        t.misses = 0;
        t.hist   = [t.hist, true];
    else
        t.misses = t.misses + 1;
        t.hist   = [t.hist, false];
    end

    recent = t.hist(max(1, end - cfg.fuse.confirm_N + 1):end);
    if strcmp(t.status, 'tentative') && sum(recent) >= cfg.fuse.confirm_M
        t.status = 'confirmed';
    end

    if t.misses > 0 && strcmp(t.status, 'confirmed')
        t.status = 'coasting';
    elseif t.misses == 0 && strcmp(t.status, 'coasting')
        t.status = 'confirmed';
    end

    % A tentative track that misses is almost always clutter; a confirmed one
    % is allowed to coast, which is how a pedestrian survives being hidden
    % behind a bus for half a second.
    if strcmp(t.status, 'tentative') && t.misses >= 2
        keep(i) = false;
    elseif t.misses >= cfg.fuse.max_misses
        keep(i) = false;
    end

    tracks(i) = t;
end
tracks = tracks(keep);

% ---- 5. duplicate suppression -------------------------------------------
% Occlusion and re-detection routinely leave two confirmed tracks on one
% object: the original coasts on its prediction while a fresh one is born from
% the re-acquisition. Both then look like obstacles, and the planner brakes for
% a vehicle that is not there. Collapse pairs that are close in both position
% and velocity, keeping the better-supported track.
tracks = local_prune_duplicates(tracks);

% ---- diagnostics ---------------------------------------------------------
diag.n_det       = nd;
diag.n_track     = numel(tracks);
diag.n_confirmed = 0;
for i = 1:numel(tracks)
    if ~strcmp(tracks(i).status, 'tentative')
        diag.n_confirmed = diag.n_confirmed + 1;
    end
end
diag.n_new = numel(groups);
end

% -------------------------------------------------------------------------
function tracks = local_prune_duplicates(tracks)
%LOCAL_PRUNE_DUPLICATES Drop confirmed tracks that shadow a better one.
%   Two tracks are treated as one object when they are close in position and
%   moving similarly. Position alone is not enough: on a crowded market road a
%   pedestrian and a stopped rickshaw can sit two metres apart and are
%   genuinely distinct, whereas a duplicate shares its target's velocity.
POS_TOL   = 3.5;    % [m]
SPD_TOL   = 4.0;    % [m/s]
HEAD_TOL  = 0.7;    % [rad]
POS_HARD  = 1.2;    % [m] this close, treat as the same object regardless

n = numel(tracks);
if n < 2
    return;
end

drop = false(1, n);
for i = 1:n
    if drop(i) || strcmp(tracks(i).status, 'tentative')
        continue;
    end
    for j = (i+1):n
        if drop(j) || strcmp(tracks(j).status, 'tentative')
            continue;
        end
        d = hypot(tracks(i).x(1) - tracks(j).x(1), ...
                  tracks(i).x(2) - tracks(j).x(2));
        if d > POS_TOL
            continue;
        end

        same = (d < POS_HARD);
        if ~same
            dv = abs(tracks(i).x(3) - tracks(j).x(3));
            dh = abs(sih_wrap_pi(tracks(i).x(4) - tracks(j).x(4)));
            same = (dv < SPD_TOL) && (dh < HEAD_TOL);
        end
        if ~same
            continue;
        end

        % Keep the track with more supporting measurements; break ties on age
        % so the established track survives a freshly born shadow.
        if tracks(i).hits > tracks(j).hits || ...
          (tracks(i).hits == tracks(j).hits && tracks(i).age >= tracks(j).age)
            drop(j) = true;
        else
            drop(i) = true;
            break;
        end
    end
end
tracks = tracks(~drop);
end

% -------------------------------------------------------------------------
function [tracks, unassigned] = local_associate(tracks, dets, idx, dt, cfg)
%LOCAL_ASSOCIATE Gated global-nearest-neighbour for one sensor's detections.
BIG = 1e6;

nt = numel(tracks);
nj = numel(idx);
unassigned = idx;

if nt == 0 || nj == 0
    return;
end

C = BIG * ones(nt, nj);
for i = 1:nt
    for jj = 1:nj
        j = idx(jj);
        [~, ~, nis, valid] = sih_ekf_update(tracks(i).x, tracks(i).P, ...
            [dets.x(j); dets.y(j)], dets.R(:,:,j), cfg.fuse.gate_chi2);
        if valid
            C(i, jj) = nis;
        end
    end
end

assign = sih_hungarian(C);
took   = false(nj, 1);

for i = 1:nt
    jj = assign(i);
    % The solver must match every row it can, so a pairing the gate rejected
    % can still surface in the optimal assignment. Drop those explicitly.
    if jj > 0 && C(i, jj) < BIG / 2
        tracks(i) = local_apply_update(tracks(i), dets, idx(jj), dt);
        took(jj)  = true;
    end
end

unassigned = idx(~took);
end

% -------------------------------------------------------------------------
function t = local_apply_update(t, dets, j, dt)
z = [dets.x(j); dets.y(j)];

% Two-point differencing on the first follow-up measurement. A track born with
% zero velocity needs seconds for the EKF to spin its speed and heading up,
% which is far too slow to plan against; seeding from two positions collapses
% that to a single scan. z_age counts scans since the last measurement, so it
% remains the correct elapsed time even when the track coasted through an
% occlusion.
if t.hits == 1 && ~t.seen && t.z_age >= 1
    elapsed = t.z_age * dt;
    vx = (z(1) - t.last_z(1)) / elapsed;
    vy = (z(2) - t.last_z(2)) / elapsed;
    sp = hypot(vx, vy);
    if sp > 0.3          % below this the direction is measurement noise
        t.x(3)   = min(sp, 25);
        t.x(4)   = atan2(vy, vx);
        t.P(3,3) = 4.0;
        t.P(4,4) = 0.5;
    end
end

[t.x, t.P] = sih_ekf_update(t.x, t.P, z, dets.R(:,:,j), []);

% A CTRV state with negative speed describes exactly the same motion as the
% positive-speed state heading the other way, and the filter drifts into it
% freely. Downstream it is not equivalent at all: the predictor is class
% conditioned and assumes heading IS the direction of travel, so an
% auto-rickshaw reported at -5 m/s on heading 0 would have its lateral drift
% hypotheses generated on the wrong side. Normalise to the positive-speed
% representation, transforming the covariance through the same map.
if t.x(3) < 0
    t.x(3) = -t.x(3);
    t.x(4) = sih_wrap_pi(t.x(4) + pi);
    J = diag([1, 1, -1, 1, 1]);
    t.P = J * t.P * J.';
end

t.seen   = true;
t.last_z = z;
t.z_age  = 0;

% Class voting. Only the camera classifies, so evidence accumulates across
% scans rather than the label flipping frame to frame.
cls = dets.class{j};
if ~isempty(cls)
    k = find(strcmp(t.cls_names, cls), 1);
    if isempty(k)
        t.cls_names{end+1}  = cls;
        t.cls_counts(end+1) = 1;
    else
        t.cls_counts(k) = t.cls_counts(k) + 1;
    end
    [~, best] = max(t.cls_counts);
    t.class = t.cls_names{best};
end
end

% -------------------------------------------------------------------------
function groups = local_cluster(dets, idx)
%LOCAL_CLUSTER Greedy spatial clustering of unassociated detections.
%   The radius is generous enough to merge three sensors' views of one object,
%   but tight enough to keep a pedestrian distinct from the rickshaw beside
%   them.
groups = {};
if isempty(idx)
    return;
end

RADIUS = 2.0;
used   = false(numel(idx), 1);

for a = 1:numel(idx)
    if used(a)
        continue;
    end
    g = idx(a);
    used(a) = true;
    for b = (a+1):numel(idx)
        if used(b)
            continue;
        end
        if hypot(dets.x(idx(b)) - dets.x(idx(a)), ...
                 dets.y(idx(b)) - dets.y(idx(a))) < RADIUS
            g = [g; idx(b)]; %#ok<AGROW>
            used(b) = true;
        end
    end
    groups{end+1} = g; %#ok<AGROW>
end
end

% -------------------------------------------------------------------------
function t = local_birth(id, dets, group)
%LOCAL_BIRTH Create a track from one cluster of detections.
%   Positions are combined in information form, so a sharp LiDAR return
%   dominates a vague camera range estimate instead of being averaged with it.
Info = zeros(2, 2);
Ivec = zeros(2, 1);
cls  = '';
for k = 1:numel(group)
    j    = group(k);
    Ri   = inv(dets.R(:,:,j));
    Info = Info + Ri;
    Ivec = Ivec + Ri * [dets.x(j); dets.y(j)];
    if isempty(cls) && ~isempty(dets.class{j})
        cls = dets.class{j};
    end
end
P0 = inv(Info);
z0 = P0 * Ivec;

t.id = id;

% Heading and speed are unobservable from position alone at birth, so they
% start at zero with wide covariance and are seeded on the next measurement.
t.x = [z0(1); z0(2); 0; 0; 0];
t.P = diag([max(P0(1,1), 1.0), max(P0(2,2), 1.0), 9.0, 1.0, 0.5]);

if isempty(cls)
    t.class      = 'car';        % neutral prior until the camera weighs in
    t.cls_names  = {};
    t.cls_counts = [];
else
    t.class      = cls;
    t.cls_names  = {cls};
    t.cls_counts = 1;
end

t.hits   = 1;
t.misses = 0;
t.age    = 1;
t.status = 'tentative';
t.hist   = true;
t.last_z = z0;
t.z_age  = 0;
t.seen   = true;
end

% -------------------------------------------------------------------------
function t = local_empty_track_array()
t = struct('id', {}, 'x', {}, 'P', {}, 'class', {}, 'cls_names', {}, ...
           'cls_counts', {}, 'hits', {}, 'misses', {}, 'age', {}, ...
           'status', {}, 'hist', {}, 'last_z', {}, 'z_age', {}, 'seen', {});
end
