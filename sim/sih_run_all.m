function [summary, all_metrics] = sih_run_all(names, opts)
%SIH_RUN_ALL Run every scenario and report the aggregate metrics table.
%
%   [summary, all_metrics] = SIH_RUN_ALL()          runs all five scenarios
%   [...] = SIH_RUN_ALL({'market','cattle_crossing'})  runs a subset
%
%   opts (optional):
%       .export    write results/<name>.json for each run (default true)
%       .verbose   per-step console output from each run (default false)
%       .seed      override cfg.sim.seed for every scenario
%
%   This is the function that produces the scenario-completion rate the problem
%   statement asks for. Each scenario is run with its OWN configuration -- speed
%   limits, corridor width, prediction horizon and risk tolerances differ
%   substantially between a market street and a highway merge -- so the summary
%   compares behaviour under appropriate settings rather than forcing one
%   tuning onto five very different problems.

if nargin < 1 || isempty(names)
    names = {'village_road', 'urban_intersection', 'highway_merge', ...
             'market', 'cattle_crossing'};
end
if nargin < 2, opts = struct(); end
if ~isfield(opts, 'export'),  opts.export  = true;  end
if ~isfield(opts, 'verbose'), opts.verbose = false; end

n = numel(names);
all_metrics = cell(1, n);
rows = cell(1, n);

fprintf('\n=== SIH 26037 scenario suite ===\n');

for k = 1:n
    name = names{k};
    builder = ['sih_scn_' name];
    if exist(builder, 'file') ~= 2
        fprintf('  SKIP  %-20s (no %s.m)\n', name, builder);
        continue;
    end

    [scn, cfg] = feval(builder);
    cfg.verbose = opts.verbose;
    if isfield(opts, 'seed') && ~isempty(opts.seed)
        cfg.sim.seed = opts.seed;
    end

    fprintf('  run   %-20s ... ', name);
    t0 = tic;
    [result, log] = sih_run_scenario(scn, cfg, ...
        struct('verbose', opts.verbose, 'snapshots', opts.export));
    wall = toc(t0);

    m = sih_metrics(result, log, cfg);
    all_metrics{k} = m;
    rows{k} = local_row(name, result, m, wall);

    if result.collided
        verdict = 'COLLISION';
    elseif ~result.reached
        verdict = 'INCOMPLETE';
    else
        verdict = 'ok';
    end
    fprintf('%-10s  %5.1fs wall\n', verdict, wall);

    if opts.export
        sih_export_run(scn, cfg, result, log, fullfile(local_root(), 'results'));
    end
end

% ---- summary table -------------------------------------------------------
keep = ~cellfun(@isempty, rows);
rows = rows(keep);
kept_names = names(keep);
all_metrics = all_metrics(keep);

fprintf('\n');
fprintf('%-20s %6s %6s %8s %8s %9s %8s %8s %7s\n', ...
        'scenario', 'goal', 'coll', 'minclr', 'time', 'lat_p95', 'jerkRMS', 'curvmax', 'vmean');
fprintf('%s\n', repmat('-', 1, 92));
for k = 1:numel(rows)
    r = rows{k};
    fprintf('%-20s %6s %6s %8.2f %8.1f %9.1f %8.2f %8.3f %7.2f\n', ...
            kept_names{k}, r.goal, r.coll, r.minclr, r.time, ...
            r.lat95, r.jerk, r.curv, r.vmean);
end
fprintf('%s\n', repmat('-', 1, 92));

n_done = sum(cellfun(@(r) strcmp(r.goal, 'yes'), rows));
n_coll = sum(cellfun(@(r) strcmp(r.coll, 'YES'), rows));

summary.n_scenarios = numel(rows);
summary.n_completed = n_done;
summary.n_collided  = n_coll;
summary.completion_rate = n_done / max(numel(rows), 1);
summary.names = kept_names;

fprintf('completion rate %d/%d (%.0f%%)   collisions %d\n\n', ...
        n_done, numel(rows), 100*summary.completion_rate, n_coll);
end

% -------------------------------------------------------------------------
function r = local_row(~, result, m, ~)
if result.reached, r.goal = 'yes'; else, r.goal = 'NO'; end
if result.collided, r.coll = 'YES'; else, r.coll = 'no'; end
r.minclr = m.min_clear;
if isfinite(m.t_to_goal), r.time = m.t_to_goal; else, r.time = m.t_end; end
r.lat95  = m.latency_p95_ms;
r.jerk   = m.jerk_lon_rms;
r.curv   = m.curv_max;
r.vmean  = m.v_mean;
end

% -------------------------------------------------------------------------
function root = local_root()
%LOCAL_ROOT Repository root, derived from this file's location.
root = fileparts(fileparts(mfilename('fullpath')));
end
