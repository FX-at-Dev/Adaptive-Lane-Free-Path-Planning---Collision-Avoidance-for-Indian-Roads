function [result, log] = sih_run_scenario(scn, cfg, opts)
%SIH_RUN_SCENARIO Closed-loop simulation of one scenario.
%
%   [result, log] = SIH_RUN_SCENARIO(scn, cfg, opts)
%
%   Runs the full pipeline at cfg.sim.dt:
%       world -> sensors -> fusion -> prediction -> risk -> behaviour ->
%       planning -> control -> vehicle dynamics -> world
%
%   Perception and control run every step; prediction, decision and planning
%   run at cfg.sim.replan_dt, which is the rate the replanning-latency metric
%   is quoted against. Between planner cycles the controller keeps tracking the
%   trajectory it already has, which is what a real stack does and is why a
%   stale plan has to remain safe for at least one cycle.
%
%   opts (optional):
%       .verbose    print a progress line each second (default cfg.verbose)
%       .snapshots  capture full state for animation (default true)
%       .think      also capture what the stack was thinking -- detections,
%                   predictions, candidate paths, the reason for the current
%                   behaviour -- for the 3D visualiser (default cfg.debug.think)
%
%   result summarises the run; log holds per-step time series for metrics and
%   plotting.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'verbose'),   opts.verbose   = cfg.verbose; end
if ~isfield(opts, 'snapshots'), opts.snapshots = true; end
if ~isfield(opts, 'think')
    opts.think = isfield(cfg, 'debug') && isfield(cfg.debug, 'think') && cfg.debug.think;
end
% The planner records its candidates only when asked, and it reads the flag
% from cfg, so the option has to be reflected there to take effect.
cfg.debug.think = opts.think;

world = sih_world_init(scn, cfg);
st    = sih_stack_init(world, cfg, opts.think);

dt = cfg.sim.dt;
N  = round(cfg.sim.t_end / dt);

log = local_init_log(N);
snaps = {};

collided    = false;
reached     = false;
t_reached   = NaN;
collide_t   = NaN;

for k = 1:N
    t = k * dt;

    % ---- ground truth advances -----------------------------------------
    world = sih_world_step(world, dt, cfg);

    % ---- the stack: perception, decision and planning, control ----------
    % The same step the Unity co-simulation runs (cosim/sih_cosim_serve.m);
    % only the plant that moves the ego differs.
    [st, a_cmd, delta_cmd, tk] = sih_stack_tick(st, world, k, cfg);
    world.ego = sih_bicycle_step(world.ego, a_cmd, delta_cmd, dt, cfg, tk.gear);

    % ---- safety and goal, measured on ground truth ----------------------
    [min_clear, hit, worst_id] = sih_clearance_truth(world, cfg);
    if hit && ~collided
        collided  = true;
        collide_t = t;
    end

    [at_goal, d_goal] = sih_goal_reached(world.ego, scn.goal, scn.rp, cfg);
    if ~reached && at_goal
        reached   = true;
        t_reached = t;
    end

    % ---- log ------------------------------------------------------------
    log.t(k)            = t;
    log.x(k)            = world.ego.x;
    log.y(k)            = world.ego.y;
    log.psi(k)          = world.ego.psi;
    log.v(k)            = world.ego.v;
    log.a(k)            = world.ego.a;
    log.delta(k)        = world.ego.delta;
    log.min_clear(k)    = min_clear;
    log.worst_id(k)     = worst_id;
    log.latency_ms(k)   = tk.latency_ms;
    log.state{k}        = st.bp.state;
    log.v_cap(k)        = st.bp.v_cap;
    log.n_tracks(k)     = tk.tdiag.n_track;
    log.n_confirmed(k)  = tk.tdiag.n_confirmed;
    log.n_det(k)        = tk.tdiag.n_det;
    log.n_hypotheses(k) = tk.n_hyp;
    log.feasible(k)     = st.info.feasible;
    log.plan_risk(k)    = st.traj.risk;
    log.d_goal(k)       = d_goal;

    if opts.snapshots && mod(k - 1, 2) == 0
        snaps{end+1} = sih_snapshot(world, st, tk, t, min_clear); %#ok<AGROW>
    end

    if opts.verbose && mod(k, round(1/dt)) == 0
        fprintf('  t=%5.1f  v=%4.1f  %-7s  clear=%6.2f  tracks=%2d  lat=%5.1fms\n', ...
                t, world.ego.v, st.bp.state, min_clear, tk.tdiag.n_confirmed, ...
                local_last_finite(log.latency_ms, k));
    end

    if reached
        break;
    end
end

n = k;
log = local_trim_log(log, n);
log.snaps = snaps;

result.name       = scn.name;
result.reached    = reached;
result.collided   = collided;
result.t_reached  = t_reached;
result.t_end      = log.t(end);
result.collide_t  = collide_t;
result.min_clear  = min(log.min_clear);
result.n_steps    = n;
end

% -------------------------------------------------------------------------
function v = local_last_finite(arr, k)
v = NaN;
for i = k:-1:1
    if isfinite(arr(i))
        v = arr(i);
        return;
    end
end
end

% -------------------------------------------------------------------------
function log = local_init_log(N)
z = zeros(N, 1);
log.t = z; log.x = z; log.y = z; log.psi = z; log.v = z; log.a = z;
log.delta = z; log.min_clear = z; log.worst_id = z; log.latency_ms = z;
log.v_cap = z; log.n_tracks = z; log.n_confirmed = z; log.n_det = z;
log.n_hypotheses = z; log.feasible = true(N,1); log.plan_risk = z;
log.d_goal = z;
log.state = cell(N, 1);
end

% -------------------------------------------------------------------------
function log = local_trim_log(log, n)
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if numel(v) >= n
        log.(f{i}) = v(1:n);
    end
end
end
