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
%
%   result summarises the run; log holds per-step time series for metrics and
%   plotting.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'verbose'),   opts.verbose   = cfg.verbose; end
if ~isfield(opts, 'snapshots'), opts.snapshots = true; end

world = sih_world_init(scn, cfg);
rp    = scn.rp;

tracks   = [];
next_id  = 1;
ctrl     = struct();
fsm      = [];
traj     = [];
info     = struct('feasible', true);
bp       = struct('v_cap', cfg.plan.v_max, 'd_max', 2.5, ...
                  'risk_tol', cfg.plan.risk_threshold, 'state', 'CRUISE');
ra       = sih_risk_assess(world.ego, [], rp, cfg);

dt          = cfg.sim.dt;
N           = round(cfg.sim.t_end / dt);
replan_every = max(1, round(cfg.sim.replan_dt / dt));

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

    % ---- perception -----------------------------------------------------
    dets = sih_sense(world, cfg);
    [tracks, next_id, tdiag] = sih_tracker_step(tracks, dets, dt, next_id, cfg);

    % ---- decision and planning, at the replan rate ----------------------
    did_replan = (mod(k - 1, replan_every) == 0);
    if did_replan
        ra          = sih_risk_assess(world.ego, tracks, rp, cfg);
        [fsm, bp]   = sih_behavior_fsm(fsm, world.ego, ra, info, cfg);
        pred        = sih_predict_intent(tracks, cfg);
        [traj, info] = sih_lattice_plan(world.ego, rp, pred, bp, cfg);
        last_latency = info.latency_ms;
        n_hyp = 0;
        if ~isempty(pred.hyp)
            n_hyp = numel(unique(pred.hyp));
        end
    else
        last_latency = NaN;
        n_hyp = log.n_hypotheses(max(k-1, 1));
    end

    % ---- control --------------------------------------------------------
    % The controller reads the planned speed AND acceleration profile itself,
    % so no separate speed setpoint is passed in.
    [a_cmd, delta_cmd, ctrl] = sih_controller(world.ego, traj, ctrl, cfg);
    world.ego = sih_bicycle_step(world.ego, a_cmd, delta_cmd, dt, cfg);

    % ---- safety and goal, measured on ground truth ----------------------
    [min_clear, hit, worst_id] = sih_clearance_truth(world, cfg);
    if hit && ~collided
        collided  = true;
        collide_t = t;
    end

    d_goal = hypot(world.ego.x - scn.goal(1), world.ego.y - scn.goal(2));
    if ~reached && d_goal < cfg.dec.goal_tol
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
    log.latency_ms(k)   = last_latency;
    log.state{k}        = bp.state;
    log.v_cap(k)        = bp.v_cap;
    log.n_tracks(k)     = tdiag.n_track;
    log.n_confirmed(k)  = tdiag.n_confirmed;
    log.n_det(k)        = tdiag.n_det;
    log.n_hypotheses(k) = n_hyp;
    log.feasible(k)     = info.feasible;
    log.plan_risk(k)    = traj.risk;
    log.d_goal(k)       = d_goal;

    if opts.snapshots && mod(k - 1, 2) == 0
        snaps{end+1} = local_snapshot(world, tracks, traj, bp, t, min_clear); %#ok<AGROW>
    end

    if opts.verbose && mod(k, round(1/dt)) == 0
        fprintf('  t=%5.1f  v=%4.1f  %-7s  clear=%6.2f  tracks=%2d  lat=%5.1fms\n', ...
                t, world.ego.v, bp.state, min_clear, tdiag.n_confirmed, ...
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
function s = local_snapshot(world, tracks, traj, bp, t, min_clear)
%LOCAL_SNAPSHOT Minimal state needed to redraw a frame later.
s.t   = t;
s.ego = world.ego;
s.min_clear = min_clear;
s.state = bp.state;

s.agents = struct('x', {}, 'y', {}, 'psi', {}, 'class', {}, 'id', {});
for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end
    s.agents(end+1) = struct('x', a.x, 'y', a.y, 'psi', a.psi, ...
                             'class', a.class, 'id', a.id);
end

s.tracks = struct('x', {}, 'y', {}, 'psi', {}, 'v', {}, 'class', {}, 'status', {});
for k = 1:numel(tracks)
    tr = tracks(k);
    if strcmp(tr.status, 'tentative')
        continue;
    end
    s.tracks(end+1) = struct('x', tr.x(1), 'y', tr.x(2), 'psi', tr.x(4), ...
                             'v', tr.x(3), 'class', tr.class, 'status', tr.status);
end

if isempty(traj)
    s.traj_x = [];
    s.traj_y = [];
else
    s.traj_x = traj.x;
    s.traj_y = traj.y;
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
