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

% ---- meta and road -----------------------------------------------------
% Footprint of every class present, so a viewer sizes agents from the same
% table the simulation used rather than from its own copy.
classes = {};
for k = 1:numel(log.snaps)
    classes = [classes, {log.snaps{k}.agents.class}]; %#ok<AGROW>
end
think = ~isempty(log.snaps) && isfield(log.snaps{1}, 'think');
data = sih_export_header(scn, cfg, classes, think);

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
    frames{k} = sih_export_frame(log.snaps{k});
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
