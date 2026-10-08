function st = sih_stack_init(world, cfg, think)
%SIH_STACK_INIT Fresh state for the autonomy stack, before the first tick.
%
%   st = SIH_STACK_INIT(world, cfg, think)
%
%   Holds everything the stack carries from one step to the next -- tracks,
%   the behaviour FSM, the last plan and its diagnostics -- so the offline
%   runner (sim/sih_run_scenario.m) and the Unity co-simulation
%   (cosim/sih_cosim_serve.m) step the exact same stack with
%   sih_stack_tick. think records what the stack believed at each replan for
%   the 3D visualiser; it is observation only.

if nargin < 3, think = false; end

st.tracks  = [];
st.next_id = 1;
st.ctrl    = struct();
st.fsm     = [];
st.traj    = [];
st.info    = struct('feasible', true);
st.bp      = struct('v_cap', cfg.plan.v_max, 'd_max', 2.5, ...
                    'risk_tol', cfg.plan.risk_threshold, 'state', 'CRUISE');
st.ra      = sih_risk_assess(world.ego, [], world.rp, cfg);
st.pred    = [];
st.n_hyp   = 0;
st.think   = think;
st.replan_every = max(1, round(cfg.sim.replan_dt / cfg.sim.dt));
st.cfg     = cfg;     % for the snapshot's footprints
st.rev     = [];      % backing out of a box (sih_reverse)
[st.rev]   = sih_reverse([], world.ego, st, cfg, 0, world.rp);
end
