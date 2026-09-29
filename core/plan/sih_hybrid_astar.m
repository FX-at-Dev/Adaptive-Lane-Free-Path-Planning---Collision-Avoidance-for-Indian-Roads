function [traj, info] = sih_hybrid_astar(ego, rp, pred, bp, cfg)
%SIH_HYBRID_ASTAR Search-based fallback for when the lattice finds nothing.
%
%   [traj, info] = SIH_HYBRID_ASTAR(ego, rp, pred, bp, cfg) searches over
%   kinematically feasible motion primitives for a way forward along the
%   corridor. Returns a trajectory struct in the same shape as the lattice
%   planner's, or [] if no route exists.
%
%   WHY A SECOND PLANNER. The lattice samples terminal states on a regular
%   grid and connects to each in one smooth polynomial. That is fast and gives
%   comfortable motion, but it can only express manoeuvres reachable in a
%   single sweep -- and the situation it cannot handle is precisely the one
%   that matters on a crowded street: the vehicle has come to a stop beside a
%   parked cart, moving forward conflicts with the footprint of the thing it is
%   already alongside, and shifting sideways needs speed it does not have. Every
%   lattice candidate is then correctly rejected and the vehicle waits forever.
%
%   A search can compose a sequence of small steering actions, so it finds the
%   several-metre shuffle -- ease out, roll forward, straighten -- that no
%   single polynomial expresses. It is slower and its output is less smooth,
%   which is exactly why it is a fallback and not the primary planner.
%
%   Node collision tests are grid lookups (see sih_occupancy_grid); running the
%   exact swept-disc check at every one of a few thousand expansions would cost
%   seconds per cycle.

t_start = tic;

traj = [];
info = struct('expanded', 0, 'found', false, 'latency_ms', 0);

H = cfg.plan.hybrid;

% ---- goal: a station ahead on the corridor ------------------------------
[s0, d0] = sih_cart2frenet(rp, ego.x, ego.y);
s_goal   = min(s0 + H.lookahead, rp.length);
if s_goal - s0 < 2.0
    info.latency_ms = toc(t_start) * 1000;
    return;                      % already at the end of the road
end

% Aim for the corridor centre, but no further across than the behaviour layer
% currently permits.
d_goal = max(min(0, bp.d_max), -bp.d_max);
[gx, gy] = sih_frenet2cart(rp, s_goal, d_goal);

grid = sih_occupancy_grid(ego, pred, cfg);

% ---- search state -------------------------------------------------------
nyaw = H.yaw_bins;
cap  = H.max_expand;

nx    = zeros(cap, 1);  ny = zeros(cap, 1);  npsi = zeros(cap, 1);
ng    = zeros(cap, 1);  nf = zeros(cap, 1);  nt   = zeros(cap, 1);
npar  = zeros(cap, 1);  nst = zeros(cap, 1);

% visited[ix, iy, iyaw] marks a discretised pose already expanded. This is what
% makes the search "hybrid": the state is continuous, but the closed set is a
% grid, so the branching factor stays bounded.
visited = false(grid.n, grid.n, nyaw);

nx(1) = ego.x;  ny(1) = ego.y;  npsi(1) = ego.psi;
ng(1) = 0;      nt(1) = 0;      npar(1) = 0;  nst(1) = 0;
nf(1) = hypot(gx - ego.x, gy - ego.y);

open_idx = 1;
n_used   = 1;
best_goal = 0;

v_f  = max(H.speed, 0.5);
step = H.step;
dt_p = step / v_f;               % time to traverse one primitive

while ~isempty(open_idx)
    [~, oi] = min(nf(open_idx));
    cur = open_idx(oi);
    open_idx(oi) = [];

    % ---- goal test -------------------------------------------------------
    if hypot(nx(cur) - gx, ny(cur) - gy) < H.goal_tol_xy
        best_goal = cur;
        break;
    end

    [ix, iy, iyaw] = local_index(nx(cur), ny(cur), npsi(cur), grid, nyaw);
    if ix < 1 || iy < 1 || ix > grid.n || iy > grid.n
        continue;
    end
    if visited(ix, iy, iyaw)
        continue;
    end
    visited(ix, iy, iyaw) = true;

    info.expanded = info.expanded + 1;
    if info.expanded > cap - numel(H.steer_set) - 1
        break;
    end

    % ---- expand ----------------------------------------------------------
    for si = 1:numel(H.steer_set)
        delta = H.steer_set(si);

        % Integrate the bicycle model over one primitive. Two sub-steps keep
        % the arc accurate enough for the grid resolution.
        x = nx(cur); y = ny(cur); psi = npsi(cur);
        half = step / 2;
        ok = true;
        for sub = 1:2
            psi = psi + (half / cfg.ego.wheelbase) * tan(delta);
            x   = x + half * cos(psi);
            y   = y + half * sin(psi);
        end
        psi = sih_wrap_pi(psi);

        t_new = nt(cur) + dt_p;
        if t_new > H.max_time
            continue;
        end
        % Note the search is allowed to reason PAST the prediction horizon,
        % holding the last predicted frame (the occupancy grid clamps to its
        % final slice, exactly as the collision checker does). Cutting it off
        % at the horizon instead made the search useless: creeping at 1.6 m/s
        % through a 3 s horizon allows four primitives, under five metres,
        % while the goal sits twenty metres up the road. It expanded about
        % twenty nodes and gave up every single time.

        % Stay inside the drivable corridor.
        [s_n, d_n] = sih_cart2frenet(rp, x, y);
        hw_n = interp1(rp.s, rp.halfwidth, min(max(s_n, 0), rp.length), 'linear');
        if abs(d_n) > hw_n
            continue;
        end
        if s_n < s0 - 1.0
            continue;            % never plan backwards along the corridor
        end

        % Occupancy lookup.
        if local_blocked(x, y, t_new, grid)
            continue;
        end

        [jx, jy, jyaw] = local_index(x, y, psi, grid, nyaw);
        if jx < 1 || jy < 1 || jx > grid.n || jy > grid.n || visited(jx, jy, jyaw)
            continue;
        end

        % Cost: distance, plus penalties that keep the result drivable rather
        % than a sequence of alternating jerks.
        g = ng(cur) + step ...
            + H.w_steer  * abs(delta) * step ...
            + H.w_change * abs(delta - nst(cur)) ...
            + H.w_offset * abs(d_n) * step / max(hw_n, 0.5);

        h = hypot(gx - x, gy - y);

        n_used = n_used + 1;
        if n_used > cap
            n_used = cap;
            break;
        end
        nx(n_used)   = x;    ny(n_used)  = y;   npsi(n_used) = psi;
        ng(n_used)   = g;    nf(n_used)  = g + H.w_heur * h;
        nt(n_used)   = t_new;
        npar(n_used) = cur;  nst(n_used) = delta;

        open_idx(end+1) = n_used; %#ok<AGROW>
    end
end

if best_goal == 0
    info.latency_ms = toc(t_start) * 1000;
    return;
end

% ---- reconstruct --------------------------------------------------------
chain = best_goal;
while npar(chain(1)) > 0
    chain = [npar(chain(1)), chain]; %#ok<AGROW>
end

px = nx(chain);
py = ny(chain);
pp = npsi(chain);
pt = nt(chain);

if numel(px) < 3
    info.latency_ms = toc(t_start) * 1000;
    return;
end

% Resample onto the standard prediction time grid so the result is
% interchangeable with a lattice trajectory everywhere downstream.
tv = (0:cfg.pred.dt:min(pt(end), cfg.pred.horizon))';
if numel(tv) < 3
    info.latency_ms = toc(t_start) * 1000;
    return;
end

[pt_u, iu] = unique(pt);
traj.t     = tv;
traj.x     = interp1(pt_u, px(iu), tv, 'linear', 'extrap');
traj.y     = interp1(pt_u, py(iu), tv, 'linear', 'extrap');
traj.psi   = sih_wrap_pi(interp1(pt_u, unwrap(pp(iu)), tv, 'linear', 'extrap'));
traj.v     = v_f * ones(size(tv));
traj.a     = zeros(size(tv));
traj.kappa = tan(interp1(pt_u, nst(chain(iu)), tv, 'linear', 'extrap')) / cfg.ego.wheelbase;

[traj.s, traj.d] = deal(zeros(size(tv)));
for k = 1:numel(tv)
    [traj.s(k), traj.d(k)] = sih_cart2frenet(rp, traj.x(k), traj.y(k));
end

% Verify with the EXACT collision test before handing it over. The grid is
% conservative, but it is coarse, and the fallback must not be the one path in
% the system that skips the real check.
[collides, min_clear, risk] = sih_collision_check( ...
    traj.x, traj.y, traj.psi, tv, pred, cfg);

traj.cost      = ng(best_goal);
traj.risk      = risk;
traj.min_clear = min_clear;
traj.T         = tv(end);
traj.d_target  = d_goal;
traj.v_target  = v_f;
traj.emergency = false;
traj.feasible  = ~collides || min_clear > 0;
traj.hybrid    = true;

info.found      = true;
info.latency_ms = toc(t_start) * 1000;
end

% -------------------------------------------------------------------------
function [ix, iy, iyaw] = local_index(x, y, psi, grid, nyaw)
%LOCAL_INDEX Discretise a continuous pose into the closed-set grid.
ix = floor((x - grid.x0) / grid.res) + 1;
iy = floor((y - grid.y0) / grid.res) + 1;
iyaw = mod(floor((psi + pi) / (2*pi) * nyaw), nyaw) + 1;
end

% -------------------------------------------------------------------------
function tf = local_blocked(x, y, t, grid)
%LOCAL_BLOCKED Occupancy lookup at a pose and time.
ix = floor((x - grid.x0) / grid.res) + 1;
iy = floor((y - grid.y0) / grid.res) + 1;
if ix < 1 || iy < 1 || ix > grid.n || iy > grid.n
    tf = true;                 % outside the rasterised region: treat as unsafe
    return;
end
it = min(max(floor(t / grid.dt) + 1, 1), grid.nt);
tf = grid.occ(ix, iy, it);
end
