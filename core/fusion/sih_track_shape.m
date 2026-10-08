function [t, z, shift, Rz] = sih_track_shape(t, dets, j, cfg)
%SIH_TRACK_SHAPE Learn a track's shape from a box detection; correct its centre.
%
%   [t, z] = SIH_TRACK_SHAPE(t, dets, j, cfg)
%
%   Detection j may carry a box (dets.box(j,:) = [Lv Wv thv]): the extents of
%   what the sensor saw and the axis of the longer one. dets.box_full(j) says
%   whether that is the whole object (an object-level model) or only the
%   faces turned towards the sensor at dets.origin(j,:) (a LiDAR). z is the
%   detection's position, moved where needed to the object's centre.
%
%   A LiDAR sees one face, or two around a corner, never the far side:
%     * two faces (the box is not thin): the axis and extents are measured
%     * one face (a thin line of returns): it is either a long side or an
%       end. Once the axis is settled, the one it lines up with. Before:
%       longer than any road user is wide, a side; otherwise the motion
%       heading, then the road direction (a parked vehicle lies along the
%       road), then the camera's class (a 2.5 m face of a truck is its end).
%   The shape keeps the largest extents seen (decaying slowly), never less
%   than the class's typical size once the camera has classified it, and an
%   axis smoothed modulo half a turn (an axis has no front).
%
%   The measured centre is the centre of the visible part. The hidden part
%   lies beyond it, away from the sensor, so the centre is moved along each
%   axis by half the hidden extent. Without this a truck seen from behind
%   is placed four metres too close and blocks the road.
%
%   Rz is the covariance to update with: the detection's own, widened along
%   the object's axis when what was seen is only a piece of a longer object
%   whose size is known. A LiDAR sees the side of a bus at a grazing angle as
%   a row of slivers, one per column of returns; each tells where the side
%   is, not where along the bus it is, and taking each for the visible end
%   dragged the box five metres along the bus and shrank it to the sliver.
%
%   shift is how far the object's centre moved because the shape estimate
%   changed (the same visible face, a longer object behind it). The tracker
%   moves the track by it before the update, so a better guess at the size
%   -- the camera calling a 2.5 m-wide face a truck -- does not read as the
%   object driving off at 10 m/s.

z = [dets.x(j); dets.y(j)];
shift = [0; 0];
Rz = dets.R(:, :, j);
% The direction of the road where it was seen, from any sensor that knows.
if isfield(dets, 'road_psi') && numel(dets.road_psi) >= j && isfinite(dets.road_psi(j))
    t.road_psi = dets.road_psi(j);
end
if ~isfield(dets, 'box') || size(dets.box, 1) < j || any(~isfinite(dets.box(j, 1:3)))
    % No box. A near-surface measurement (a radar's range, a camera's
    % ground contact) is moved back along the line of sight by the object's
    % half depth in that direction, so it lands on the centre the track is
    % kept at. The shape is the measured one, or before the LiDAR has
    % measured one, the class's typical size along the road: left at the
    % near face, a bus seen from behind was kept 5.5 m short of its centre,
    % its box stuck out into the road behind it, and the LiDAR's correct box
    % then failed to match it.
    shp = [];
    if t.shape_n > 0 && all(isfinite(t.shape))
        shp = t.shape;
    elseif ~isempty(t.cls_names)
        pr = sih_agent_props(t.class);
        th0 = t.road_psi;
        if t.x(3) >= cfg.track.v_moving && t.hits >= 3, th0 = t.x(4); end
        if isfinite(th0), shp = [pr.length, pr.width, th0]; end
    end
    if isfield(dets, 'surface') && numel(dets.surface) >= j && dets.surface(j) && ...
            ~isempty(shp) && all(isfinite(dets.origin(j, :)))
        los = z - dets.origin(j, :).';
        r = norm(los);
        if r > 0.1
            a = atan2(los(2), los(1)) - shp(3);
            h = abs(shp(1) / 2 * cos(a)) + abs(shp(2) / 2 * sin(a));
            z = z + h * los / r;
        end
    end
    return;
end
Lv = dets.box(j, 1); Wv = dets.box(j, 2); thv = dets.box(j, 3);
full = isfield(dets, 'box_full') && numel(dets.box_full) >= j && dets.box_full(j);

classified = ~isempty(t.cls_names);
prior = [];
if classified
    prior = sih_agent_props(t.class);
end
moving = t.x(3) >= cfg.track.v_moving && t.hits >= 3;

% ---- which axis is long, and what of each extent was seen -----------------
% A box fitted to returns gives the object's orientation only up to a
% quarter turn: its longer VISIBLE edge need not be the object's long side.
% A cart seen from behind shows its 1.2 m end and a sliver of its side, and
% taking the end for the long side turned its box across the road. So the
% box says which two directions the sides run in, and something else says
% which of them is the long one:
%   * an edge longer than any road user is wide is a long side
%   * else the established axis, once a few boxes have settled it
%   * else the direction of travel, for a moving object
%   * else the road: a parked vehicle lies along it
%   * else the class's length (a 2.5 m face of a truck is its end)
% A view of one face only (a thin line) measures that face and nothing of
% the other extent.
two_faces = full || Wv >= cfg.track.two_face_width;
wmax = 2.6;
if classified, wmax = prior.width + 0.4; end
if full
    along = true;                       % an object-level box is the object
elseif Lv > wmax
    along = true;
elseif t.shape_n >= cfg.track.shape_settled && all(isfinite(t.shape))
    along = abs(local_axis_diff(thv, t.shape(3))) < pi / 4;
elseif moving
    along = abs(local_axis_diff(thv, t.x(4))) < pi / 4;
elseif isfield(t, 'road_psi') && isfinite(t.road_psi)
    along = abs(local_axis_diff(thv, t.road_psi)) < pi / 4;
elseif classified && ~two_faces
    along = Lv >= 0.7 * prior.length;
else
    along = true;
end
Wseen = Wv * two_faces;                 % the other extent, if it was seen
if along
    axis = thv; Lo = Lv; Wo = Wseen; seen_u = Lv; seen_v = Wv;
else
    axis = thv + pi / 2; Lo = Wseen; Wo = Lv; seen_u = Wv; seen_v = Lv;
end

% Only a piece of the face it lies on? A LiDAR sees the side of a bus at a
% grazing angle as a row of slivers, one per column of returns: each says
% where the side is, not where along the bus it is, and taking each for the
% whole visible face dragged the box metres along the bus and shrank it.
% (A whole end seen from behind is not a piece: the end is all there is.)
piece_u = false; piece_v = false;
if ~full && t.shape_n >= cfg.track.shape_settled && all(isfinite(t.shape))
    if along
        piece_u = seen_u < cfg.track.piece_frac * t.shape(1);
    else
        piece_v = seen_v < cfg.track.piece_frac * t.shape(2);
    end
end
piece = piece_u || piece_v;

% ---- update the shape ------------------------------------------------------
old_shape = t.shape;
if t.shape_n == 0 || any(~isfinite(t.shape))
    th = axis;
    W = max(Wo, 0.3);
    if Wo == 0, W = 0.6; end
    L = max([Lo, W, 0.6]);          % never shorter than it is wide
else
    % The axis follows the measurements slowly, and an object that is not
    % moving does not turn: for a still track only a view of two faces --
    % the outline of a corner, which fixes the orientation -- may turn it.
    g = cfg.track.theta_gain;
    if t.still && ~two_faces
        g = 0;
    elseif t.still
        g = g * cfg.track.theta_still_gain;
    end
    % A piece of the object says nothing about its orientation either.
    if piece, g = 0; end
    th = t.shape(3) + g * local_axis_diff(axis, t.shape(3));
    % The extents keep the largest seen, easing back towards what is seen
    % now -- but only when the whole outline is in view (two faces): one
    % face, or a piece, says nothing about the size of what is out of
    % sight, and easing back on it shrank a stall seen end-on to 0.3 m.
    d = cfg.track.extent_decay;
    if piece || ~two_faces, d = 1; end
    L = max(Lo, d * t.shape(1));
    W = max(Wo, d * t.shape(2));
end
if classified
    % Never smaller than the class's typical size, nor much bigger: two
    % animals seen as one cluster are not one 5 m animal.
    cap = cfg.track.class_size_cap;
    L = min(max(L, prior.length), cap * prior.length);
    W = min(max(W, prior.width), cap * prior.width);
end
t.shape = [L, W, sih_wrap_pi(th)];
t.shape_n = t.shape_n + 1;

% ---- the centre: the hidden part lies beyond the visible one ---------------
if ~full && isfield(dets, 'origin') && size(dets.origin, 1) >= j && all(isfinite(dets.origin(j, :)))
    z_seen = z;
    u = [cos(t.shape(3)); sin(t.shape(3))];
    v = [-u(2); u(1)];
    if piece_u
        % A piece of a side: which side is known, where along it is not.
        % Corrected across the axis only; its position along the axis
        % counts for next to nothing.
        z = local_centre(z_seen, t.shape, t.shape(1), seen_v, dets.origin(j, :).');
        Rz = Rz + (max(0, t.shape(1) - seen_u) / 2)^2 * (u * u.');
    elseif piece_v
        % A piece of an end, likewise across it.
        z = local_centre(z_seen, t.shape, seen_u, t.shape(2), dets.origin(j, :).');
        Rz = Rz + (max(0, t.shape(2) - seen_v) / 2)^2 * (v * v.');
    else
        z = local_centre(z_seen, t.shape, seen_u, seen_v, dets.origin(j, :).');
        if all(isfinite(old_shape))
            shift = z - local_centre(z_seen, old_shape, seen_u, seen_v, dets.origin(j, :).');
        end
    end
end
end

% -------------------------------------------------------------------------
function z = local_centre(z, shape, seen_u, seen_v, origin)
%LOCAL_CENTRE The centre of an object of this shape, given the centre of
%   what was seen of it from origin.
u = [cos(shape(3)); sin(shape(3))];
v = [-u(2); u(1)];
los = z - origin;
hid_u = max(0, shape(1) - seen_u);
hid_v = max(0, shape(2) - seen_v);
z = z + local_sign(u.' * los) * hid_u / 2 * u + local_sign(v.' * los) * hid_v / 2 * v;
end

% -------------------------------------------------------------------------
function d = local_axis_diff(a, b)
%LOCAL_AXIS_DIFF a - b for axes, which repeat every half turn: in (-pi/2, pi/2].
d = mod(a - b + pi / 2, pi) - pi / 2;
end

function s = local_sign(x)
s = 1;
if x < 0, s = -1; end
end
