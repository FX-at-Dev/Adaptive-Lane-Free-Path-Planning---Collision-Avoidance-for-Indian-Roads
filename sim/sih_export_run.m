function path_out = sih_export_run(scn, cfg, result, log, out_dir)
%SIH_EXPORT_RUN Write a completed scenario run to JSON for external rendering.
%
%   path_out = SIH_EXPORT_RUN(scn, cfg, result, log, out_dir)
%
%   Produces <out_dir>/<scenario>.json containing the road geometry, the ego
%   time series, per-frame snapshots of the world as the vehicle saw it, and
%   the computed metrics.
%
%   The separation exists because rendering and simulation have different
%   dependencies. Under Octave on a headless machine the only available
%   graphics toolkit requires a display, so plotting from inside the simulation
%   is not possible at all; tools/render_run.py consumes this file instead. A
%   MATLAB-native visualiser reading the same JSON is Phase 3 work, once a
%   MATLAB licence is available to test it against.
%
%   Exporting rather than plotting in place also means the replay and the
%   metrics come from one simulation run, so the animation shows exactly the
%   run the numbers describe rather than a re-simulation that may differ.

if nargin < 5 || isempty(out_dir)
    out_dir = fullfile(pwd, 'results');
end
if exist(out_dir, 'dir') ~= 7
    mkdir(out_dir);
end

m = sih_metrics(result, log, cfg);

% ---- road geometry, decimated ------------------------------------------
% The reference path is sampled every 0.25 m, which is far finer than any
% renderer needs and would dominate the file size.
step = max(1, round(1.0 / scn.rp.ds));
idx  = 1:step:numel(scn.rp.s);

data.meta.name       = scn.name;
data.meta.desc       = scn.desc;
data.meta.dt         = cfg.sim.dt;
data.meta.replan_dt  = cfg.sim.replan_dt;
data.meta.goal       = scn.goal;
data.meta.v_max      = cfg.plan.v_max;
data.meta.ego_length = cfg.ego.length;
data.meta.ego_width  = cfg.ego.width;
data.meta.latency_budget = cfg.metric.latency_budget;

data.road.x  = scn.rp.x(idx).';
data.road.y  = scn.rp.y(idx).';
data.road.hw = scn.rp.halfwidth(idx).';

% ---- ego time series ----------------------------------------------------
data.series.t          = log.t.';
data.series.x          = log.x.';
data.series.y          = log.y.';
data.series.psi        = log.psi.';
data.series.v          = log.v.';
data.series.a          = log.a.';
data.series.delta      = log.delta.';
data.series.min_clear  = log.min_clear.';
data.series.v_cap      = log.v_cap.';
data.series.plan_risk  = log.plan_risk.';
data.series.feasible   = double(log.feasible).';
data.series.n_tracks   = log.n_confirmed.';
data.series.state      = log.state(:).';

% Latency is only defined on planner cycles; NaN elsewhere does not survive a
% JSON round trip cleanly, so the defined samples are exported as pairs.
li = find(isfinite(log.latency_ms));
data.series.latency_t  = log.t(li).';
data.series.latency_ms = log.latency_ms(li).';

% ---- per-frame world snapshots -----------------------------------------
frames = cell(1, numel(log.snaps));
for k = 1:numel(log.snaps)
    s = log.snaps{k};
    f.t     = s.t;
    f.state = s.state;
    f.ego   = [s.ego.x, s.ego.y, s.ego.psi, s.ego.v];

    na = numel(s.agents);
    f.agents = zeros(na, 3);
    f.agent_class = cell(1, na);
    for i = 1:na
        f.agents(i,:)    = [s.agents(i).x, s.agents(i).y, s.agents(i).psi];
        f.agent_class{i} = s.agents(i).class;
    end

    nt = numel(s.tracks);
    f.tracks = zeros(nt, 4);
    f.track_class = cell(1, nt);
    for i = 1:nt
        f.tracks(i,:)    = [s.tracks(i).x, s.tracks(i).y, s.tracks(i).psi, s.tracks(i).v];
        f.track_class{i} = s.tracks(i).class;
    end

    % The planned trajectory is short; decimate lightly.
    f.traj = [s.traj_x(:).'; s.traj_y(:).'];

    frames{k} = f;
end
data.frames = frames;

% ---- metrics ------------------------------------------------------------
data.metrics = m;
data.result.reached   = double(result.reached);
data.result.collided  = double(result.collided);
data.result.t_reached = result.t_reached;
data.result.t_end     = result.t_end;
data.result.min_clear = result.min_clear;

% ---- write --------------------------------------------------------------
path_out = fullfile(out_dir, [scn.name '.json']);

if exist('jsonencode', 'builtin') == 5 || exist('jsonencode', 'file') == 2
    txt = jsonencode(data);
else
    error('sih_export_run:noJson', ...
          ['jsonencode is unavailable on this interpreter. It ships with ' ...
           'MATLAB R2016b+ and Octave 7+; upgrade, or render from MATLAB ' ...
           'with sih_visualize instead.']);
end

fid = fopen(path_out, 'w');
if fid < 0
    error('sih_export_run:write', 'Could not open %s for writing.', path_out);
end
fwrite(fid, txt, 'char');
fclose(fid);

fprintf('  exported %s (%.1f KB, %d frames)\n', path_out, numel(txt)/1024, numel(frames));
end
