function dets = sih_radar_detect(Rr, ego, ctx, cfg)
%SIH_RADAR_DETECT Objects from radar returns.
%
%   dets = SIH_RADAR_DETECT(Rr, ego, ctx, cfg)
%
%   Rr has one row per return as the sensor reports it: [range bearing
%   range_rate], range in metres, bearing in radians counter-clockwise from
%   straight ahead, range rate in m/s; the radar sits at the ego's geometric
%   centre facing forward. Each return is already one object (the sensor's own
%   clustering of its beam hits). Radar cannot tell a parked lorry from a wall
%   by size, but it measures speed: with the ego's own motion taken out, a
%   return whose ground speed along the beam is below
%   cfg.percep.radar_static_speed is stationary. Stationary returns near the
%   static map (grown by the radar's coarse-bearing margin) are the walls
%   and poles beside the road and are dropped; moving returns, and
%   stationary ones away from mapped structure (a stopped car), are kept.
%   Only returns on the road itself (corridor zone 2) are considered.
%   The covariance is the radar's: sharp in range, coarse in bearing.
%
%   dets is in the form sih_sense returns (sensor 'radar', no class).

dets = sih_dets_empty();
if isempty(Rr)
    return;
end
Rr = double(Rr);
sp = cfg.sensor.radar;
sx = ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre;
sy = ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre;
b = ego.psi + Rr(:,2);
x = sx + Rr(:,1) .* cos(b);
y = sy + Rr(:,1) .* sin(b);
ground_speed = Rr(:,3) + ego.v * cos(Rr(:,2));      % the ego's own motion taken out
still = abs(ground_speed) < cfg.percep.radar_static_speed;
keep = Rr(:,1) <= sp.range & sih_roi(ctx, x, y) == 2 & ~(still & sih_on_map(ctx, x, y, true));
x = x(keep); y = y(keep); b = b(keep); r = Rr(keep, 1);
m = numel(x);
dets.x = x;
dets.y = y;
dets.R = zeros(2, 2, m);
for i = 1:m
    c = cos(b(i)); s = sin(b(i));
    Rot = [c, -s; s, c];
    dets.R(:,:,i) = Rot * diag([sp.sigma_range^2, (r(i) * sp.sigma_bear)^2]) * Rot.';
end
dets.surface = true(m, 1);              % a radar ranges to the near surface
dets.road_psi = local_road_psi(ctx, x, y);
dets.origin = repmat([sx, sy], m, 1);
dets.class = repmat({''}, m, 1);
dets.sensor = repmat({'radar'}, m, 1);
dets.truth_id = zeros(m, 1);
end

% -------------------------------------------------------------------------
function r = local_road_psi(ctx, x, y)
%LOCAL_ROAD_PSI Direction of the road at each detection (NaN without a road).
r = NaN(numel(x), 1);
if ~isfield(ctx, 'rp'), return; end
for i = 1:numel(x)
    [~, ~, ~, r(i)] = sih_cart2frenet(ctx.rp, x(i), y(i));
end
end
