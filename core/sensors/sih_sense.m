function dets = sih_sense(world, cfg, use)
%SIH_SENSE Run the camera, radar and LiDAR models over the ground truth.
%
%   dets = SIH_SENSE(world, cfg) returns the fused detection set for one scan:
%       dets.x, dets.y   [N x 1] position in the global frame
%       dets.R           [2 x 2 x N] position covariance, global frame
%       dets.class       {N x 1} class label, empty when the sensor cannot
%                        classify
%       dets.sensor      {N x 1} originating sensor
%       dets.truth_id    [N x 1] ground-truth agent id, or 0 for clutter
%                        (diagnostics only -- never read by the tracker)
%       dets.box, box_full, origin, road_psi   the object's extent, where the
%                        sensor measures one (sih_dets_empty): this
%                        object-level LiDAR reports the whole box, true size
%                        and orientation, as a LiDAR's object list would
%
%   The three sensors are deliberately complementary and individually poor.
%   The camera has good bearing and weak depth, the radar the reverse, and the
%   LiDAR is accurate but short ranged and blind to class. No single one is
%   sufficient, which is what makes the fusion step earn its place rather than
%   being decorative.
%
%   dets = SIH_SENSE(world, cfg, use) runs only the sensors flagged in use,
%   a logical [camera radar lidar]. The co-simulation uses this when Unity
%   supplies some sensors itself (core/perception); a sensor that is skipped
%   draws no random numbers, and with all three on the result is identical
%   to the two-argument call.

if nargin < 3
    use = true(1, 3);
end

specs = {cfg.sensor.camera, cfg.sensor.radar, cfg.sensor.lidar};
names = {'camera', 'radar', 'lidar'};

dets = sih_dets_empty();

% Precompute occlusion blockers once: every disc of every active agent.
[bx, by, br, bid] = local_blockers(world.agents);

for si = 1:numel(specs)
    if ~use(si)
        continue;
    end
    d = local_scan(world, specs{si}, names{si}, bx, by, br, bid, cfg);
    dets = sih_dets_cat(dets, d);
end
end

% -------------------------------------------------------------------------
function dets = local_scan(world, spec, name, bx, by, br, bid, cfg)
%LOCAL_SCAN One sensor's view of the world.

dets = sih_dets_empty();
ego  = world.ego;

for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end

    dx = a.x - ego.x;
    dy = a.y - ego.y;
    rng_true = hypot(dx, dy);

    if rng_true > spec.range || rng_true < 0.1
        continue;
    end

    bearing_global = atan2(dy, dx);
    bearing_rel    = sih_wrap_pi(bearing_global - ego.psi);
    if abs(bearing_rel) > spec.fov / 2
        continue;
    end

    % Line of sight, ignoring the target's own discs.
    if cfg.sensor.occlusion
        own = (bid == a.id);
        if sih_ray_blocked(ego.x, ego.y, a.x, a.y, bx(~own), by(~own), br(~own))
            continue;
        end
    end

    % Detection probability falls off with range.
    pd = spec.pd * (1 - 0.40 * (rng_true / spec.range)^2);
    if rand() > pd
        continue;
    end

    % ---- sensor-specific noise, expressed as a Cartesian ellipse --------
    switch name
        case 'camera'
            % Shallow depth: range error scales with range, bearing is sharp.
            sr = spec.sigma_range * rng_true;
            sb = spec.sigma_bear;
        case 'radar'
            % Sharp depth, coarse angular resolution.
            sr = spec.sigma_range;
            sb = spec.sigma_bear;
        case 'lidar'
            sr = spec.sigma_pos;
            sb = spec.sigma_pos / max(rng_true, 1.0);
        otherwise
            error('sih_sense:sensor', 'Unknown sensor "%s".', name);
    end

    r_meas = rng_true + sr * randn();
    b_meas = bearing_global + sb * randn();

    zx = ego.x + r_meas * cos(b_meas);
    zy = ego.y + r_meas * sin(b_meas);

    % Rotate the along-range / cross-range ellipse into the global frame.
    c = cos(bearing_global);
    s = sin(bearing_global);
    Rot = [c, -s; s, c];
    R = Rot * diag([sr^2, (rng_true * sb)^2]) * Rot.';

    % ---- class label ----------------------------------------------------
    if spec.classifies
        if rand() < spec.class_acc
            cls = a.class;
        else
            cls = local_confuse(a.class);
        end
    else
        cls = '';
    end

    dets = local_push(dets, zx, zy, R, cls, name, a.id);
    % The road direction there, which every sensor's perception knows.
    [~, ~, ~, road_psi] = sih_cart2frenet(world.rp, zx, zy);
    n = numel(dets.x);
    dets.road_psi(n) = road_psi;
    if strcmp(name, 'lidar')
        % The object's whole box.
        dets.box(n, :) = [a.props.length, a.props.width, a.psi];
        dets.box_full(n) = true;
        dets.origin(n, :) = [ego.x, ego.y];
    end
end

% ---- clutter -------------------------------------------------------------
% False alarms are uniform in the sensor's field of view. They exist to make
% the association gate do real work; without them every detection is a true
% one and a broken gate would never show.
n_fa = local_poisson(cfg.sensor.clutter_rate);
for k = 1:n_fa
    rr = spec.range * sqrt(rand());                     % area-uniform in range
    bb = ego.psi + (rand() - 0.5) * spec.fov;
    zx = ego.x + rr * cos(bb);
    zy = ego.y + rr * sin(bb);
    R  = diag([1.0, 1.0]);
    dets = local_push(dets, zx, zy, R, '', name, 0);
end
end

% -------------------------------------------------------------------------
function [bx, by, br, bid] = local_blockers(agents)
%LOCAL_BLOCKERS Flatten every active agent's footprint into occluding discs.
bx = []; by = []; br = []; bid = [];
for k = 1:numel(agents)
    a = agents(k);
    if ~a.active
        continue;
    end
    [cx, cy] = sih_agent_discs(a.x, a.y, a.psi, a.props);
    n = numel(cx);
    bx  = [bx;  cx(:)];
    by  = [by;  cy(:)];
    br  = [br;  a.props.radius * ones(n, 1)];
    bid = [bid; a.id * ones(n, 1)];
end
end

% -------------------------------------------------------------------------
function cls = local_confuse(true_class)
%LOCAL_CONFUSE Pick a plausible misclassification.
%   Confusions follow real detector behaviour: an auto-rickshaw reads as a car
%   far more often than as a cow. This matters because the predictor is class
%   conditioned, so a confusion changes the predicted motion model, not just a
%   label on a plot.
switch true_class
    case 'car',         opts = {'auto', 'truck'};
    case 'auto',        opts = {'car', 'pushcart'};
    case 'bus',         opts = {'truck'};
    case 'truck',       opts = {'bus', 'car'};
    case 'two_wheeler', opts = {'bicycle'};
    case 'bicycle',     opts = {'two_wheeler', 'pedestrian'};
    case 'pedestrian',  opts = {'bicycle', 'cattle'};
    case 'cattle',      opts = {'pedestrian', 'pushcart'};
    case 'pushcart',    opts = {'cattle', 'auto'};
    otherwise,          opts = {'car'};
end
cls = opts{min(numel(opts), 1 + floor(rand() * numel(opts)))};
end

% -------------------------------------------------------------------------
function n = local_poisson(lambda)
%LOCAL_POISSON Knuth sampler; adequate for the small rates used here and
%   available without the statistics package under Octave.
if lambda <= 0
    n = 0;
    return;
end
L = exp(-lambda);
k = 0;
p = 1;
while true
    k = k + 1;
    p = p * rand();
    if p <= L
        break;
    end
end
n = k - 1;
end

% -------------------------------------------------------------------------
function d = local_push(d, x, y, R, cls, sensor, tid)
n = numel(d.x) + 1;
d.x(n,1) = x;
d.y(n,1) = y;
d.R(:,:,n) = R;
d.class{n,1} = cls;
d.sensor{n,1} = sensor;
d.truth_id(n,1) = tid;
d.box(n,:) = [NaN, NaN, NaN];
d.box_full(n,1) = false;
d.origin(n,:) = [NaN, NaN];
d.road_psi(n,1) = NaN;
d.surface(n,1) = false;
end
