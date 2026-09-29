function test_scenario_closed_loop()
%TEST_SCENARIO_CLOSED_LOOP End-to-end assertions on the village road scenario.
%
%   Runs the full pipeline -- world, sensors, fusion, prediction, risk,
%   behaviour, planning, control, vehicle dynamics -- and asserts the
%   properties the problem statement asks to be demonstrated: collision-free
%   completion, real-time replanning, and motion inside the comfort envelope.
%
%   This is the slowest test in the suite by a wide margin (roughly a minute),
%   because it is a full closed-loop simulation rather than a unit check. It
%   earns that cost: every bug found during development was an INTERACTION
%   between components that individually passed their own tests -- a planner
%   that rejected every candidate because of an initial condition the
%   controller had set, a behaviour state whose risk tolerance made its own
%   exit condition unreachable, a curvature test that divided by a speed the
%   vehicle did not yet have. None of those are visible from a unit test.
%
%   Thresholds are set to catch regression, not to certify performance: they
%   sit far enough from the measured values that ordinary run-to-run variation
%   does not trip them, and close enough that a genuine behavioural regression
%   does.

[scn, cfg] = sih_scn_village_road();
cfg.verbose = false;

[result, log] = sih_run_scenario(scn, cfg, ...
                                 struct('verbose', false, 'snapshots', false));
m = sih_metrics(result, log, cfg);

% ---- safety: the property that matters most ----------------------------
sih_assert_true(~result.collided, ...
    'the vehicle collided at t = %.2f s (min clearance %.3f m)', ...
    result.collide_t, result.min_clear);

sih_assert_true(m.min_clear > 0.15, ...
    'minimum clearance %.3f m is too tight', m.min_clear);

sih_assert_true(m.t_near_miss < 1.0, ...
    'spent %.2f s inside the near-miss threshold', m.t_near_miss);

% ---- completion --------------------------------------------------------
sih_assert_true(result.reached, ...
    'did not reach the goal within %.0f s (%.0f m travelled, %.1f m short)', ...
    cfg.sim.t_end, m.dist_travel, log.d_goal(end));

sih_assert_true(m.t_to_goal < 0.9 * cfg.sim.t_end, ...
    'reached the goal at %.1f s, uncomfortably close to the %.0f s limit', ...
    m.t_to_goal, cfg.sim.t_end);

sih_assert_true(m.dist_travel > 150, ...
    'only travelled %.0f m; the route is about 190 m', m.dist_travel);

% The vehicle must actually make progress rather than crawling the whole way.
% An over-conservative planner that inches to the goal would satisfy every
% safety assertion above while being useless, and that failure mode occurred
% repeatedly during development.
sih_assert_true(m.v_mean > 2.5, ...
    'mean speed %.2f m/s is too timid for an 8 m/s limit', m.v_mean);

% ---- real-time replanning ----------------------------------------------
sih_assert_true(m.latency_p95_ms < 150, ...
    'p95 replanning latency %.1f ms', m.latency_p95_ms);

sih_assert_true(m.latency_mean_ms < cfg.metric.latency_budget, ...
    'mean replanning latency %.1f ms exceeds the %.0f ms budget', ...
    m.latency_mean_ms, cfg.metric.latency_budget);

sih_assert_true(m.n_replans > 300, ...
    'only %d planner cycles ran; the loop is not replanning as expected', ...
    m.n_replans);

% ---- motion quality ----------------------------------------------------
sih_assert_true(m.curv_max <= cfg.metric.curv_limit * 1.2, ...
    'peak path curvature %.3f 1/m exceeds the steering envelope', m.curv_max);

sih_assert_true(m.frac_jerk_ok > 0.75, ...
    'only %.0f%% of moving samples are within the jerk comfort limit', ...
    100 * m.frac_jerk_ok);

% ---- the scenario must exercise the behaviour layer --------------------
% If the vehicle simply cruised the whole way, the scenario is not testing
% anything and the assertions above would pass vacuously.
i_cruise = find(strcmp(m.state_names, 'CRUISE'));
sih_assert_true(m.state_frac(i_cruise) < 0.9, ...
    'spent %.0f%% of the run in CRUISE; the scenario is not challenging the planner', ...
    100 * m.state_frac(i_cruise));

sih_assert_true(sum(m.state_frac) > 0.99, ...
    'behaviour states account for only %.2f of the run', sum(m.state_frac));

% ---- perception must actually be tracking something --------------------
sih_assert_true(m.tracks_mean > 0.5, ...
    'mean confirmed track count %.2f -- perception is not seeing the traffic', ...
    m.tracks_mean);
end
