function [dets, info] = sih_lidar_objects(wx, wy, z, zone, ego, ctx, cfg)
%SIH_LIDAR_OBJECTS Road users from LiDAR returns already filtered to the road.
%
%   [dets, info] = SIH_LIDAR_OBJECTS(wx, wy, z, zone, ego, ctx, cfg)
%
%   wx, wy, z are the returns in the world frame, ground, overhead and static
%   structure already removed, zone their corridor zone (sih_roi); ego is the
%   pose the scan was taken from. Steps 3-5 of sih_lidar_detect: cluster,
%   fit a box to each cluster's outline, keep the road users. Separate so a
%   recorded scan can be replayed through it.

dets = sih_dets_empty();
info = struct('clusters', 0, 'objects', 0);
pc = cfg.percep;
if numel(wx) < pc.min_points
    return;
end
c = cos(ego.psi);
s = sin(ego.psi);

% ---- 3. bird's-eye clustering --------------------------------------------------
nj = max(1, round(pc.join / pc.cell));         % joining reach, in cells
gx = floor((wx - ctx.x0) / pc.cell);
gy = floor((wy - ctx.y0) / pc.cell) + nj + 1;  % rows from nj+1, so a neighbour row is never < 1
K = max(gy) + nj + 2;                          % ... nor spills into the next column
[keys, ~, cell_of] = unique(gx * K + gy);
nc = numel(keys);
cx = floor(keys / K);
cy = keys - cx * K;
% Cells within pc.join of each other are connected: a long side seen at a
% grazing angle returns sparsely, and joining only touching cells broke one
% bus into a row of fragments.
I = zeros(0, 1);
J = zeros(0, 1);
for ox = 0:nj
    for oy = -nj:nj
        if ox == 0 && oy <= 0, continue; end          % each pair once
        if hypot(ox, oy) > nj + 0.5, continue; end
        [tf, loc] = ismember((cx + ox) * K + (cy + oy), keys);
        I = [I; find(tf)];   %#ok<AGROW>
        J = [J; loc(tf)];    %#ok<AGROW>
    end
end
lab = local_components(nc, I, J);
[~, ~, lab] = unique(lab);
obj = lab(cell_of);                         % cluster of every return
no = max(obj);
info.clusters = no;

% ---- 4. oriented box per cluster -----------------------------------------------
% The box is fitted to the outline, not to the spread of the returns. A LiDAR
% sees a vehicle as an L (its end and one side) or a line, never a filled
% shape; the principal axis of an L is its diagonal, and it swings from one
% face to the other as the view changes, so a parked truck's box turned on
% the spot while the car drove past it. Instead every orientation in a
% quarter turn is tried and the one whose rectangle hugs the returns closest
% is kept (L-shape fitting by the closeness criterion, Zhang et al. 2017).
n   = accumarray(obj, 1, [no 1]);
mx  = accumarray(obj, wx, [no 1]) ./ n;
my  = accumarray(obj, wy, [no 1]) ./ n;
dx  = wx - mx(obj);
dy  = wy - my(obj);
na  = pc.fit_angles;
ang = (0:na-1) * (pi / 2) / na;
U   = dx * cos(ang) + dy * sin(ang);            % returns x angles
V   = -dx * sin(ang) + dy * cos(ang);
sub = [repmat(obj, na, 1), kron((1:na).', ones(numel(obj), 1))];
U0  = accumarray(sub, U(:), [no na], @min);  U1 = accumarray(sub, U(:), [no na], @max);
V0  = accumarray(sub, V(:), [no na], @min);  V1 = accumarray(sub, V(:), [no na], @max);
oi  = repmat(obj, 1, na);
ai  = repmat(1:na, numel(obj), 1);
li  = oi + (ai - 1) * no;
du  = min(U - U0(li), U1(li) - U);              % distance to the nearer edge, each axis
dv  = min(V - V0(li), V1(li) - V);
close_ = 1 ./ max(min(du, dv), pc.fit_d0);
score = accumarray(sub, close_(:), [no na]);
% An outline that fits about as well along the road is taken along it: a
% vehicle on a road is far more often aligned with it, and a near tie
% decided by noise is what makes a box flicker between orientations.
if isfield(ctx, 'rp')
    rpsi = zeros(no, 1);
    for i = 1:no
        [~, ~, ~, rpsi(i)] = sih_cart2frenet(ctx.rp, mx(i), my(i));
    end
    off = abs(mod(ang - mod(rpsi, pi / 2) + pi / 4, pi / 2) - pi / 4);   % no x na
    score = score .* (1 + pc.fit_road_bias * (off < deg2rad(1) + (pi / 2) / na));
end
[~, best] = max(score, [], 2);
bl  = (1:no).' + (best - 1) * no;
a   = ang(best).';
u0  = U0(bl); u1 = U1(bl); v0 = V0(bl); v1 = V1(bl);
ocx = mx + cos(a) .* (u0 + u1) / 2 - sin(a) .* (v0 + v1) / 2;
ocy = my + sin(a) .* (u0 + u1) / 2 + cos(a) .* (v0 + v1) / 2;
len = u1 - u0;
wid = v1 - v0;
swap = wid > len;                               % the longer extent is the axis
th  = a + swap * pi / 2;
t_  = len(swap); len(swap) = wid(swap); wid(swap) = t_;
zt  = accumarray(obj, z, [no 1], @max);
zn  = accumarray(obj, zone, [no 1], @max);  % 2 if any of it is near the road

% ---- 5. which clusters are road users ---------------------------------------------
ok = n >= pc.min_points & len <= pc.max_extent;
small = zt <= pc.wide_max_height & max(len, wid) <= pc.wide_max_extent;
ok = ok & (zn == 2 | small);
idx = find(ok);
info.objects = numel(idx);
if isempty(idx)
    return;
end

% Each object: the box of what was seen, where it was seen from and which
% way the road runs there. The tracker (sih_track_shape) works out from these
% which faces were visible and where the hidden rest of the object is.
sx = ego.x + c * cfg.ego.rear_axle_to_centre;
sy = ego.y + s * cfg.ego.rear_axle_to_centre;
m = numel(idx);
dets.x = ocx(idx);
dets.y = ocy(idx);
dets.R = zeros(2, 2, m);
dets.box = [len(idx), wid(idx), th(idx)];
dets.box_full = false(m, 1);
dets.origin = repmat([sx, sy], m, 1);
dets.road_psi = NaN(m, 1);
sp = cfg.sensor.lidar.sigma_pos;
for i = 1:m
    % The centre of the visible part moves a little as the view changes.
    sl = sqrt(sp^2 + (len(idx(i)) / 8)^2);
    sw = sqrt(sp^2 + (wid(idx(i)) / 8)^2);
    ct = cos(th(idx(i))); st = sin(th(idx(i)));
    Rot = [ct, -st; st, ct];
    dets.R(:,:,i) = Rot * diag([sl^2, sw^2]) * Rot.';
    if isfield(ctx, 'rp')
        [~, ~, ~, dets.road_psi(i)] = sih_cart2frenet(ctx.rp, dets.x(i), dets.y(i));
    end
end
dets.class = repmat({''}, m, 1);
dets.sensor = repmat({'lidar'}, m, 1);
dets.truth_id = zeros(m, 1);
end

% -------------------------------------------------------------------------
function lab = local_components(n, I, J)
%LOCAL_COMPONENTS Connected components of n nodes joined by edges (I, J):
%   min-label propagation with pointer jumping, vectorised.
lab = (1:n).';
if isempty(I)
    return;
end
while true
    m = min(lab(I), lab(J));
    nl = min(lab, accumarray([I; J], [m; m], [n 1], @min, n + 1));
    nl = nl(nl);                             % jump: label of my label
    if isequal(nl, lab)
        return;
    end
    lab = nl;
end
end
