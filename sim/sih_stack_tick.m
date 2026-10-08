function [st, a_cmd, delta_cmd, tk] = sih_stack_tick(st, world, k, cfg, dets)
%SIH_STACK_TICK One step of the autonomy stack: sense, track, decide, plan, control.
%
%   [st, a_cmd, delta_cmd, tk] = SIH_STACK_TICK(st, world, k, cfg)
%
%   world is the ground truth after it has advanced to step k (the stack
%   senses it; it never reads it otherwise). Perception and control run every
%   step; risk, behaviour, prediction and planning run every
%   st.replan_every steps, starting with step 1. Between planner cycles the
%   controller keeps tracking the trajectory it already has.
%
%   Returns the actuator command for the ego and tk, what this step produced
%   for logging: detections, tracker diagnostics, whether it replanned, the
%   planning latency (NaN when it did not), the number of prediction
%   hypotheses and, with st.think, the controller's lookahead point.
%
%   dets, when given, replaces the stack's own sensor models (sih_sense):
%   the co-simulation passes detections made from Unity's raw sensor data by
%   core/perception, merged with any sensors sih_sense still simulates.
%
%   The ego itself is advanced by the caller -- sih_bicycle_step offline, the
%   Unity plant in co-simulation -- which is the one thing the two runners do
%   differently.

rp = world.rp;

% ---- perception -----------------------------------------------------------
if nargin < 5
    dets = sih_sense(world, cfg);
end
[st.tracks, st.next_id, tdiag] = sih_tracker_step(st.tracks, dets, cfg.sim.dt, st.next_id, cfg);

% ---- backing out of a box (sih_reverse) -----------------------------------
% Outside the planner, which plans forward only: boxed in close behind
% something, the vehicle backs up a few metres and plans again from there.
was_reversing = ~strcmp(st.rev.phase, 'idle');
[st.rev, a_rev, d_rev, reversing, why_rev] = sih_reverse(st.rev, world.ego, st, cfg, cfg.sim.dt, rp);
if reversing
    st.bp.state  = 'REVERSE';
    st.bp.reason = why_rev;
    st.bp.v_cap  = 0;
    tk.dets = dets; tk.tdiag = tdiag; tk.did_replan = false; tk.latency_ms = NaN;
    tk.n_hyp = st.n_hyp; tk.look = []; tk.gear = -1;
    a_cmd = a_rev; delta_cmd = d_rev;
    return;
end

% ---- decision and planning, at the replan rate ----------------------------
did_replan = (mod(k - 1, st.replan_every) == 0) || was_reversing;
if did_replan
    st.ra             = sih_risk_assess(world.ego, st.tracks, rp, cfg);
    [st.fsm, st.bp]   = sih_behavior_fsm(st.fsm, world.ego, st.ra, st.info, cfg);
    st.bp             = sih_arrive(st.bp, world.ego, world.goal, rp, cfg);
    st.pred           = sih_predict_intent(st.tracks, cfg);
    [st.traj, st.info] = sih_lattice_plan(world.ego, rp, st.pred, st.bp, cfg);
    latency = st.info.latency_ms;
    st.n_hyp = 0;
    if ~isempty(st.pred.hyp)
        st.n_hyp = numel(unique(st.pred.hyp));
    end
else
    latency = NaN;
end

% ---- control --------------------------------------------------------------
% The controller reads the planned speed AND acceleration profile itself, so
% no separate speed setpoint is passed in.
[a_cmd, delta_cmd, st.ctrl] = sih_controller(world.ego, st.traj, st.ctrl, cfg);

tk.dets       = dets;
tk.tdiag      = tdiag;
tk.did_replan = did_replan;
tk.latency_ms = latency;
tk.n_hyp      = st.n_hyp;
tk.look       = [];
tk.gear       = 1;
if world.ego.v < 0
    tk.gear = -1;          % still rolling back: the brake works in reverse gear
end
if st.think
    % Pure-pursuit target, taken now because the ego moves next.
    tk.look = [world.ego.x + st.ctrl.Ld * cos(world.ego.psi + st.ctrl.alpha), ...
               world.ego.y + st.ctrl.Ld * sin(world.ego.psi + st.ctrl.alpha)];
end
end
