function z = sih_roi(ctx, x, y)
%SIH_ROI Corridor zone of world points: 2 near the road, 1 wide margin, 0 beyond.
%
%   z = SIH_ROI(ctx, x, y) looks the points up in the mask built by
%   sih_perception_init.

gx = floor((x - ctx.x0) / ctx.cell) + 1;
gy = floor((y - ctx.y0) / ctx.cell) + 1;
in = gx >= 1 & gx <= ctx.nx & gy >= 1 & gy <= ctx.ny;
z = zeros(size(x));
z(in) = double(ctx.roi(sub2ind([ctx.ny, ctx.nx], gy(in), gx(in))));
end
