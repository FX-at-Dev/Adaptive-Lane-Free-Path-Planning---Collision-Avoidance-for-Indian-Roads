function on = sih_on_map(ctx, x, y, radar)
%SIH_ON_MAP True for world points on known static structure (sih_perception_init).
%
%   on = SIH_ON_MAP(ctx, x, y)         LiDAR margin
%   on = SIH_ON_MAP(ctx, x, y, true)   the wider radar margin

if nargin < 4, radar = false; end
gx = floor((x - ctx.x0) / ctx.map_cell) + 1;
gy = floor((y - ctx.y0) / ctx.map_cell) + 1;
in = gx >= 1 & gx <= ctx.mnx & gy >= 1 & gy <= ctx.mny;
on = false(size(x));
if radar
    on(in) = ctx.static_radar(sub2ind([ctx.mny, ctx.mnx], gy(in), gx(in)));
else
    on(in) = ctx.static(sub2ind([ctx.mny, ctx.mnx], gy(in), gx(in)));
end
end
