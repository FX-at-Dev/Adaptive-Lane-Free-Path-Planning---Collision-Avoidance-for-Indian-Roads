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
%   hits, misses, age, status ('tentative' | 'confirmed' | 'coasting'), and
%   the object's shape separate from its motion: shape [L W theta] (NaN until
%   a box is seen, sih_track_shape), still (stationary for cfg.track.t_still)
%   and road_psi (road direction at the object, when a sensor gave it).
%   sih_footprint turns all of this into the geometry every decision uses.

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
    tracks(i).lidar_age = tracks(i).lidar_age + 1;   % scans since the LiDAR saw it
end

% ---- 1b. one detection per known object ------------------------------------
% Up close a LiDAR sees a stall's posts and awning, or a bus side at a
% grazing angle, as several separate pieces. Association takes one
% detection per track, so it matched one piece -- a different one each scan,
% each read as the visible end -- and the box was dragged metres along the
% object while the other pieces became objects of their own. Pieces lying in
% one tracked footprint are first merged into the one box they make up.
dets = local_merge_pieces(tracks, dets, cfg);
nd = numel(dets.x);

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
% A LiDAR piece no track took whose centre lies inside an object already
% tracked is part of it -- a cart's handle, a lorry's tail -- and not a new
% object: association is one detection per track per sensor, so the second
% piece of one object always looks new, and each became a box of its own.
% Its centre must be inside the footprint itself, with no margin, so a cow
% stepping out beside a parked lorry is never taken for part of it.
leftover = leftover & ~local_fragment(tracks, dets, cfg);
groups = local_cluster(dets, find(leftover));
for g = 1:numel(groups)
    tracks(end+1) = local_birth(next_id, dets, groups{g}, cfg); %#ok<AGROW>
    next_id = next_id + 1;
end

% ---- 4. management, once per scan ---------------------------------------
% Where the LiDAR was this scan, if it reported anything (empty if not).
lidar_at = [];
if isfield(dets, 'origin') && ~isempty(dets.origin)
    k = find(strcmp(dets.sensor(:), 'lidar') & all(isfinite(dets.origin), 2), 1);
    if ~isempty(k), lidar_at = dets.origin(k, :); end
end
keep = true(1, numel(tracks));
for i = 1:numel(tracks)
    t = tracks(i);

    % Stationary: slow for long enough, or a class that never moves. With
    % hysteresis: a parked object's filtered speed is noise, and one noisy
    % update above the threshold used to make it "moving" -- its footprint
    % then turned with the noisy heading, on the spot. It stops being still
    % only after it has been clearly moving for cfg.track.t_move.
    if t.x(3) < cfg.track.v_still
        t.slow_t = t.slow_t + dt;
    elseif ~t.still
        t.slow_t = 0;
    end
    % Clearly moving: a walking cow or pedestrian manages 0.7 m/s, and
    % taking one for a parked object because it never reached 1 m/s put a
    % moving animal's footprint, standing still, next to the car.
    if t.x(3) >= cfg.track.v_unstill
        t.fast_t = t.fast_t + dt;
    else
        t.fast_t = 0;
    end
    if t.still && t.fast_t >= cfg.track.t_move
        t.still = false;
        t.slow_t = 0;
    elseif ~t.still
        t.still = t.slow_t >= cfg.track.t_still;
    end
    t.still = t.still || strcmp(t.class, 'static');

    % Established: something to stop for. A new detection in front may be a
    % ghost -- a reflection, a camera box with a range metres out, a stray
    % LiDAR cluster -- so the vehicle only slows for it (sih_risk_assess,
    % ra.caution) until it has been tracked for cfg.track.establish_hits
    % scans and, within the LiDAR's reach, the LiDAR still sees it. A ghost
    % is gone by then and the vehicle never stopped; a real object is
    % established while there is still room to stop for it. Once
    % established it stays so while it is tracked: up close a low object
    % drops below the LiDAR's lowest beam, and it is no less there.
    in_reach = ~isempty(lidar_at) && hypot(t.x(1) - lidar_at(1), t.x(2) - lidar_at(2)) <= ...
               cfg.track.lidar_trust * cfg.sensor.lidar.range;
    t.established = t.established || (t.hits >= cfg.track.establish_hits && ...
                    (~in_reach || t.lidar_age <= cfg.track.lidar_recent));

    if t.seen
        t.hits   = t.hits + 1;
        t.misses = 0;
        t.hist   = [t.hist, true];
    else
        t.misses = t.misses + 1;
        t.hist   = [t.hist, false];
    end

    % Confirmed: seen in M of the last N scans, and by a sensor that can be
    % trusted to say something is there. A LiDAR return is a solid surface;
    % a camera box alone can be a shadow, a poster or a range guess metres
    % out, and a radar return alone can be the road, a kerb or a multipath
    % echo -- each became a track that the planner stopped for. Either can
    % confirm an object the other also reports, which covers the range
    % beyond the LiDAR's.
    recent = t.hist(max(1, end - cfg.fuse.confirm_N + 1):end);
    if strcmp(t.status, 'tentative') && sum(recent) >= cfg.fuse.confirm_M && local_corroborated(t, lidar_at, cfg)
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
    if strcmp(t.status, 'tentative') && (t.misses >= 2 || t.age > cfg.fuse.tentative_max_age)
        keep(i) = false;
    elseif t.misses >= cfg.fuse.max_misses && ~(t.still && t.misses < cfg.fuse.max_misses_still)
        % A still object is not going anywhere: it may coast longer, so a
        % parked cart seen only now and then at the edge of the LiDAR's
        % range keeps one box instead of a new one every few scans.
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
tracks = local_prune_duplicates(tracks, cfg);

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
function dets = local_merge_pieces(tracks, dets, cfg)
%LOCAL_MERGE_PIECES Merge LiDAR boxes whose centres lie in one tracked footprint.
n = numel(dets.x);
if n < 2 || isempty(tracks) || ~isfield(dets, 'box')
    return;
end
isl = strcmp(dets.sensor(:), 'lidar') & all(isfinite(dets.box), 2) & ~dets.box_full(:);
if sum(isl) < 2
    return;
end
used = false(n, 1);
drop = false(n, 1);
for i = 1:numel(tracks)
    t = tracks(i);
    % A settled measured shape, or before that the class's size once the
    % camera has said what it is (a bus's side in slivers, before the LiDAR
    % has outlined it).
    if strcmp(t.status, 'tentative') || ...
            ((t.shape_n < cfg.track.shape_settled || ~all(isfinite(t.shape))) && isempty(t.cls_names))
        continue;
    end
    fp = sih_footprint(t, cfg);
    u = [cos(fp.theta); sin(fp.theta)];
    v = [-u(2); u(1)];
    du = (dets.x(:) - fp.cx) * u(1) + (dets.y(:) - fp.cy) * u(2);
    dv = (dets.x(:) - fp.cx) * v(1) + (dets.y(:) - fp.cy) * v(2);
    cand = find(isl & ~used & abs(du) <= fp.L / 2 + 0.3 & abs(dv) <= fp.W / 2 + 0.3);
    if numel(cand) < 2
        continue;
    end
    % The corners of every piece, in the footprint's frame.
    pu = zeros(4 * numel(cand), 1);
    pv = pu;
    for q = 1:numel(cand)
        j = cand(q);
        c = cos(dets.box(j, 3)); s = sin(dets.box(j, 3));
        hl = dets.box(j, 1) / 2; hw = dets.box(j, 2) / 2;
        cx = dets.x(j) + c * [hl, hl, -hl, -hl] - s * [hw, -hw, -hw, hw];
        cy = dets.y(j) + s * [hl, hl, -hl, -hl] + c * [hw, -hw, -hw, hw];
        pu(4*q-3:4*q) = (cx - fp.cx) * u(1) + (cy - fp.cy) * u(2);
        pv(4*q-3:4*q) = (cx - fp.cx) * v(1) + (cy - fp.cy) * v(2);
    end
    u0 = min(pu); u1 = max(pu); v0 = min(pv); v1 = max(pv);
    % Only pieces that together still fit the object: two cows side by
    % side are two cows, and merging them grew one box until it swallowed
    % the herd.
    if u1 - u0 > fp.L + 0.5 || v1 - v0 > fp.W + 0.5
        continue;
    end
    % What the pieces make up locates the object; it does not make it
    % bigger -- repeated merges that each added a little grew a 3 m stall
    % to 5 m and closed the gap beside it.
    um = (u0 + u1) / 2; vm = (v0 + v1) / 2;
    u0 = max(u0, um - fp.L / 2); u1 = min(u1, um + fp.L / 2);
    v0 = max(v0, vm - fp.W / 2); v1 = min(v1, vm + fp.W / 2);
    k = cand(1);
    ctr = [fp.cx; fp.cy] + u * (u0 + u1) / 2 + v * (v0 + v1) / 2;
    dets.x(k) = ctr(1);
    dets.y(k) = ctr(2);
    if u1 - u0 >= v1 - v0
        dets.box(k, :) = [u1 - u0, v1 - v0, fp.theta];
    else
        dets.box(k, :) = [v1 - v0, u1 - u0, sih_wrap_pi(fp.theta + pi / 2)];
    end
    sp = cfg.sensor.lidar.sigma_pos;
    sl = sqrt(sp^2 + (dets.box(k, 1) / 8)^2);
    sw = sqrt(sp^2 + (dets.box(k, 2) / 8)^2);
    Rot = [cos(dets.box(k, 3)), -sin(dets.box(k, 3)); sin(dets.box(k, 3)), cos(dets.box(k, 3))];
    dets.R(:, :, k) = Rot * diag([sl^2, sw^2]) * Rot.';
    used(cand) = true;
    drop(cand(2:end)) = true;
end
if any(drop)
    keep = ~drop;
    f = fieldnames(dets);
    for q = 1:numel(f)
        val = dets.(f{q});
        if strcmp(f{q}, 'R')
            dets.R = val(:, :, keep);
        elseif size(val, 1) == n
            dets.(f{q}) = val(keep, :);
        end
    end
end
end

% -------------------------------------------------------------------------
function in = local_fragment(tracks, dets, cfg)
%LOCAL_FRAGMENT LiDAR detections whose centre is inside a tracked footprint.
in = false(numel(dets.x), 1);
for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative')
        continue;
    end
    fp = sih_footprint(t, cfg);
    u = [cos(fp.theta), sin(fp.theta)];
    dx = dets.x - fp.cx;
    dy = dets.y - fp.cy;
    along = abs(u(1) * dx + u(2) * dy);
    across = abs(-u(2) * dx + u(1) * dy);
    in = in | (strcmp(dets.sensor, 'lidar') & along <= fp.L / 2 & across <= fp.W / 2);
end
end

% -------------------------------------------------------------------------
function ok = local_corroborated(t, lidar_at, cfg)
%LOCAL_CORROBORATED Has a LiDAR seen it -- or, out of the LiDAR's reach,
%   both the camera and the radar? Within its reach the LiDAR would have:
%   a camera box and a radar echo that happened to agree nine metres ahead
%   of the car were confirmed as a car and stopped it dead.
ok = t.src(3) > 0;
if ~ok && t.src(1) > 0 && t.src(2) > 0
    ok = isempty(lidar_at) || ...
         hypot(t.x(1) - lidar_at(1), t.x(2) - lidar_at(2)) > cfg.track.lidar_trust * cfg.sensor.lidar.range;
end
end

% -------------------------------------------------------------------------
function k = local_sensor_index(name)
switch name
    case 'camera', k = 1;
    case 'radar',  k = 2;
    otherwise,     k = 3;
end
end

% -------------------------------------------------------------------------
function in = local_inside(a, b, cfg)
%LOCAL_INSIDE Is a's centre inside b's footprint (grown 0.3 m)?
in = false;
if strcmp(b.status, 'tentative') || ~isfield(b, 'shape') || ~all(isfinite(b.shape))
    return;
end
fp = sih_footprint(b, cfg);
u = [cos(fp.theta); sin(fp.theta)];
d = [a.x(1) - fp.cx; a.x(2) - fp.cy];
in = abs(u.' * d) <= fp.L / 2 + 0.3 && abs([-u(2), u(1)] * d) <= fp.W / 2 + 0.3;
end

% -------------------------------------------------------------------------
function on = local_on_footprint(t, dets, j, cfg)
%LOCAL_ON_FOOTPRINT Does LiDAR box detection j lie on track t's footprint?
on = false;
if strcmp(t.status, 'tentative') || ~isfield(dets, 'box') || size(dets.box, 1) < j || ...
        any(~isfinite(dets.box(j, :))) || ~isfield(t, 'shape') || ~all(isfinite(t.shape))
    return;
end
fp = sih_footprint(t, cfg);
u = [cos(fp.theta); sin(fp.theta)];
d = [dets.x(j) - fp.cx; dets.y(j) - fp.cy];
% 0.3 m: a road user beside it (a cow stepping out past a parked truck) has
% its centre at least half its own width outside, and is never taken for it.
on = abs(u.' * d) <= fp.L / 2 + 0.3 && abs([-u(2), u(1)] * d) <= fp.W / 2 + 0.3;
end

% -------------------------------------------------------------------------
function tracks = local_prune_duplicates(tracks, cfg)
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
        if d > POS_TOL && d > 7
            continue;                % beyond any footprint
        end

        same = (d < POS_HARD);
        % Extended objects: a LiDAR sees the ends and side of an 11 m bus as
        % separate clusters, and each became a bus-sized track. One whose
        % centre lies inside the other's footprint is the same object, if
        % they move alike.
        if ~same && (local_inside(tracks(i), tracks(j), cfg) || local_inside(tracks(j), tracks(i), cfg))
            same = abs(tracks(i).x(3) - tracks(j).x(3)) < SPD_TOL;
        end
        % A track the LiDAR has never seen, beside one it has, moving alike:
        % the camera's or radar's copy of that object, placed off by its
        % poor range or bearing. Heading is no test while both are slow.
        if ~same && d <= POS_TOL && xor(tracks(i).src(3) > 0, tracks(j).src(3) > 0)
            slow = tracks(i).x(3) < cfg.track.v_moving && tracks(j).x(3) < cfg.track.v_moving;
            dh = abs(sih_wrap_pi(tracks(i).x(4) - tracks(j).x(4)));
            same = abs(tracks(i).x(3) - tracks(j).x(3)) < SPD_TOL && (slow || dh < HEAD_TOL);
            if same
                % keep the LiDAR's, whatever the hit counts
                if tracks(i).src(3) > 0, drop(j) = true; else, drop(i) = true; break; end
                continue;
            end
        end
        if ~same && d <= POS_TOL     % beyond it only the footprint test merges
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
% A cheap first cut: a detection further from a track than both their
% lengths, plus three standard deviations of both their positions, cannot
% pass the gate below or lie on its footprint, and skipping the full
% shape-and-centre computation for those pairs -- most of them -- took the
% tracker from the slowest part of a step to a small one.
Lt = zeros(nt, 1); St = zeros(nt, 1); Xt = zeros(nt, 2);
for i = 1:nt
    Lt(i) = sih_agent_props(tracks(i).class).length;
    if all(isfinite(tracks(i).shape)), Lt(i) = max(Lt(i), tracks(i).shape(1)); end
    St(i) = max(tracks(i).P(1,1), 0) + max(tracks(i).P(2,2), 0);
    Xt(i, :) = tracks(i).x(1:2).';
end
Lb = zeros(1, nj);
if isfield(dets, 'box') && size(dets.box, 1) >= max(idx)
    Lb = dets.box(idx, 1).';
    Lb(~isfinite(Lb)) = 0;
end
Sd = reshape(dets.R(1,1,idx) + dets.R(2,2,idx), 1, nj);
reach = (Lt + Lb) / 2 + 3.1 * sqrt(St + Sd) + 1.0;              % [nt x nj]
near = hypot(Xt(:, 1) - dets.x(idx).', Xt(:, 2) - dets.y(idx).') <= reach;
for i = 1:nt
    for jj = find(near(i, :))
        j = idx(jj);
        % Gate on where the detection says the object's CENTRE is: a
        % LiDAR box of a truck's rear face is four metres from it.
        [~, zc, ~, Rc] = sih_track_shape(tracks(i), dets, j, cfg);
        [~, ~, nis, valid] = sih_ekf_update(tracks(i).x, tracks(i).P, ...
            zc, Rc, cfg.fuse.gate_chi2);
        if valid
            C(i, jj) = nis;
        elseif local_on_footprint(tracks(i), dets, j, cfg)
            % A LiDAR box lying on a confirmed object's footprint is part of
            % it -- an end or a side of a long vehicle -- even when its
            % centre is too far from the object's for the point gate.
            C(i, jj) = 0.9 * cfg.fuse.gate_chi2;
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
        tracks(i) = local_apply_update(tracks(i), dets, idx(jj), dt, cfg);
        took(jj)  = true;
    end
end

unassigned = idx(~took);
end

% -------------------------------------------------------------------------
function t = local_apply_update(t, dets, j, dt, cfg)
t = local_vote(t, dets.class{j});           % first: the class sizes the shape

% The shape, and the centre corrected for the faces the sensor could not see.
% A better size estimate moves the centre; the track moves with it, rather
% than the filter reading the jump as motion.
[t, z, shift, Rz] = sih_track_shape(t, dets, j, cfg);
t.x(1:2) = t.x(1:2) + shift;

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

[t.x, t.P] = sih_ekf_update(t.x, t.P, z, Rz, []);

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
si = local_sensor_index(dets.sensor{j});
t.src(si) = t.src(si) + 1;
if si == 3, t.lidar_age = 0; end
end

% -------------------------------------------------------------------------
function t = local_vote(t, cls)
%LOCAL_VOTE Class voting. Only the camera classifies, so evidence
%   accumulates across scans rather than the label flipping frame to frame.
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
function t = local_birth(id, dets, group, cfg)
%LOCAL_BIRTH Create a track from one cluster of detections.
%   Positions are combined in information form, so a sharp LiDAR return
%   dominates a vague camera range estimate instead of being averaged with it.
cls  = '';
for k = 1:numel(group)
    if isempty(cls) && ~isempty(dets.class{group(k)})
        cls = dets.class{group(k)};
    end
end

t.id = id;
t.x = zeros(5, 1);
t.P = eye(5);

if isempty(cls)
    t.class      = 'car';        % neutral prior until the camera weighs in
    t.cls_names  = {};
    t.cls_counts = [];
else
    t.class      = cls;
    t.cls_names  = {cls};
    t.cls_counts = 1;
end
t.shape    = [NaN, NaN, NaN];
t.shape_n  = 0;
t.still    = false;
t.slow_t   = 0;
t.fast_t   = 0;
t.road_psi = NaN;
t.src      = zeros(1, 3);   % updates by camera, radar, LiDAR
t.lidar_age = 1e6;          % scans since the LiDAR last saw it
t.established = false;      % seen long enough, and by the LiDAR, to stop for
for k = 1:numel(group)
    si = local_sensor_index(dets.sensor{group(k)});
    t.src(si) = t.src(si) + 1;
    if si == 3, t.lidar_age = 0; end
end

% Positions are combined in information form, each first moved to the
% object's centre where its sensor saw only part of it (sih_track_shape).
Info = zeros(2, 2);
Ivec = zeros(2, 1);
for k = 1:numel(group)
    j    = group(k);
    [t, zj] = sih_track_shape(t, dets, j, cfg);
    Ri   = inv(dets.R(:,:,j));
    Info = Info + Ri;
    Ivec = Ivec + Ri * zj;
end
P0 = inv(Info);
z0 = P0 * Ivec;

% Heading and speed are unobservable from position alone at birth, so they
% start at zero with wide covariance and are seeded on the next measurement.
t.x = [z0(1); z0(2); 0; 0; 0];
t.P = diag([max(P0(1,1), 1.0), max(P0(2,2), 1.0), 9.0, 1.0, 0.5]);

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
           'cls_counts', {}, 'shape', {}, 'shape_n', {}, 'still', {}, ...
           'slow_t', {}, 'fast_t', {}, 'road_psi', {}, 'src', {}, 'lidar_age', {}, 'established', {}, 'hits', {}, 'misses', {}, 'age', {}, ...
           'status', {}, 'hist', {}, 'last_z', {}, 'z_age', {}, 'seen', {});
end
