function grid = sih_occupancy_grid(ego, pred, cfg)
%SIH_OCCUPANCY_GRID Rasterise predicted occupancy into a time-sliced grid.
%
%   grid = SIH_OCCUPANCY_GRID(ego, pred, cfg) returns an ego-centred,
%   axis-aligned binary grid with one slice per time band:
%       .occ   [nx x ny x nt] logical, true where the ego centre may not go
%       .x0,.y0  world coordinates of cell (1,1)
%       .res     cell size [m]
%       .dt      seconds per time slice
%       .nt      number of slices
%
%   Cells are inflated by the ego's own covering radius plus the safety
%   margin, so a cell being free means the EGO CENTRE may occupy it -- the
%   search can then test a pose with a single lookup instead of a swept-disc
%   computation.
%
%   This exists for the Hybrid A* fallback. That search expands thousands of
%   nodes per call, and running the full collision check at each one would cost
%   seconds; rasterising once up front turns every node test into an array
%   index. The grid is deliberately coarse, and it is conservative in the
%   direction that matters: a disc is marked over every cell its inflated
%   radius touches, so the search can only ever be more cautious than the exact
%   test, never less.

res    = cfg.plan.hybrid.grid_res;
ahead  = cfg.risk.ahead;
behind = cfg.risk.behind;
lat    = cfg.risk.lateral;

% Axis-aligned bounds large enough to hold the region of interest whatever the
% ego heading, which keeps the indexing arithmetic trivial.
R  = max(ahead, max(behind, lat)) + 6;
x0 = ego.x - R;
y0 = ego.y - R;
n  = ceil(2*R / res) + 1;

% Time slices spanning the prediction horizon.
nt = max(1, ceil(cfg.pred.horizon / cfg.plan.hybrid.slice_dt));
dt = cfg.plan.hybrid.slice_dt;

grid.occ = false(n, n, nt);
grid.x0  = x0;
grid.y0  = y0;
grid.res = res;
grid.dt  = dt;
grid.nt  = nt;
grid.n   = n;

if isempty(pred) || ~isfield(pred, 'x') || isempty(pred.x)
    return;
end

% Ego inflation: a cell is blocked if the ego CENTRE there would conflict.
% The search plans to a REDUCED margin. It is the last resort before stopping,
% and the situations it handles -- easing out from beside a parked cart -- are
% precisely the ones where the full cruising margin does not fit on the road.
% The result is re-checked afterwards against the exact, full-margin test, so
% nothing unsafe reaches the controller; this only widens what the search is
% willing to consider.
[~, ~, rego] = sih_ego_discs(ego.x, ego.y, ego.psi, cfg);
ego_infl = rego + cfg.plan.hybrid.margin_scale * cfg.plan.safety_margin;

K = numel(pred.t);
A = size(pred.x, 2);

for it = 1:nt
    % Representative prediction frame for this slice.
    t_mid = (it - 0.5) * dt;
    ki = min(max(round(t_mid / cfg.pred.dt) + 1, 1), K);

    for a = 1:A
        % Very unlikely hypotheses are skipped: the fallback is already a
        % last resort, and blocking on a five-per-cent branch would leave it
        % with nothing to find.
        if pred.w(a) < cfg.plan.hybrid.min_weight
            continue;
        end

        cx = pred.x(ki, a);
        cy = pred.y(ki, a);
        rr = pred.r(a) + ego_infl;

        ix_lo = floor((cx - rr - x0) / res) + 1;
        ix_hi = ceil( (cx + rr - x0) / res) + 1;
        iy_lo = floor((cy - rr - y0) / res) + 1;
        iy_hi = ceil( (cy + rr - y0) / res) + 1;

        ix_lo = max(ix_lo, 1);  ix_hi = min(ix_hi, n);
        iy_lo = max(iy_lo, 1);  iy_hi = min(iy_hi, n);
        if ix_lo > ix_hi || iy_lo > iy_hi
            continue;
        end

        gx = x0 + ((ix_lo:ix_hi) - 1) * res;
        gy = y0 + ((iy_lo:iy_hi) - 1) * res;

        dx = gx(:) * ones(1, numel(gy)) - cx;
        dy = ones(numel(gx), 1) * gy - cy;

        mask = (dx.^2 + dy.^2) <= rr^2;
        if any(mask(:))
            block = grid.occ(ix_lo:ix_hi, iy_lo:iy_hi, it);
            grid.occ(ix_lo:ix_hi, iy_lo:iy_hi, it) = block | mask;
        end
    end
end
end
