function [fsm, bp] = sih_behavior_fsm(fsm, ego, ra, plan_info, cfg)
%SIH_BEHAVIOR_FSM Behaviour state machine for unstructured mixed traffic.
%
%   [fsm, bp] = SIH_BEHAVIOR_FSM(fsm, ego, ra, plan_info, cfg)
%
%   States, grouped by the mode they belong to:
%       NORMAL    CRUISE  free running at the scenario speed cap
%                 FOLLOW  matched to a slower lead vehicle
%       CAUTION   NUDGE   easing laterally around an obstacle
%                 YIELD   giving way to a crossing agent
%                 CREEP   low-speed negotiation in dense traffic
%       EMERGENCY STOP    come to rest, plan again from there
%
%   The FSM does NOT command steering or acceleration. It emits planner
%   parameters:
%       bp.v_cap     speed ceiling
%       bp.d_max     lateral freedom either side of the corridor centreline
%       bp.risk_tol  acceptable probability mass of predicted conflict
%
%   Keeping the decision layer to three numbers is what makes it tractable:
%   the planner stays a single optimiser rather than a pile of special cases,
%   and this function stays small enough to transcribe into a legible
%   Stateflow chart without carrying any geometry across.
%
%   Call this BEFORE the planner each cycle, passing the previous cycle's
%   plan_info. A plan that came back infeasible is the strongest evidence
%   available that the situation has deteriorated, and acting on it one cycle
%   later is what lets the machine escalate to STOP rather than repeatedly
%   asking the planner for something it has already said does not exist.

if isempty(fsm) || ~isfield(fsm, 'state')
    fsm.state    = 'CRUISE';
    fsm.hold     = 0;        % cycles the current state must persist
    fsm.prev     = 'CRUISE';
    fsm.t_in     = 0;
    fsm.stuck    = 0;        % consecutive cycles at a standstill
end

prev_state = fsm.state;

% ---- evaluate transition conditions, highest severity first -------------
infeasible = ~isempty(plan_info) && isfield(plan_info, 'feasible') && ~plan_info.feasible;

% Planner infeasibility justifies an emergency stop only while the vehicle is
% still moving, where shedding speed is the useful response. Once stopped it
% must not: STOP would then be self-sustaining, because standing still does
% not improve whatever made planning infeasible, and the vehicle would never
% move again. A stopped vehicle with room around it escalates to CREEP instead
% and lets the planner try to inch out.
stopped = ego.v < 0.3;

if stopped
    fsm.stuck = fsm.stuck + 1;
else
    fsm.stuck = 0;
end

% UNSTICK. A vehicle that has been stationary for several seconds with real
% room around it is not being careful, it is stuck -- and the situation will
% not improve by waiting, because nothing about it is changing. This is the
% failure mode that dominated the dense scenarios: the vehicle halts beside a
% parked cart, forward motion conflicts with the predicted footprint of
% something it is already alongside, lateral correction is impossible at zero
% speed, and it waits forever.
%
% Edging forward is what a driver does here, and it is what breaks the
% deadlock: a little forward travel restores the vehicle's ability to steer,
% which restores the lateral options, which resolves the conflict. The raised
% risk tolerance below is deliberate and bounded -- it applies only at creep
% speed, only when there is more than a metre of clearance, and it is dropped
% as soon as the vehicle is moving again.
unstick = (fsm.stuck > cfg.dec.stuck_cycles) && (ra.min_clear > cfg.dec.stuck_clear);

% Only a genuine proximity or time-to-collision breach forces an immediate
% stop. Planner infeasibility does not: see the escalation ladder below.
critical = ra.ttc < cfg.dec.ttc_emergency || ...
           ra.min_clear < cfg.dec.clear_stop;

% Recovery: stopped, no acceptable plan, but real clearance all round. Creeping
% is what a human driver does here -- edge forward until the situation clears.
recovering = stopped && infeasible && ra.min_clear > cfg.dec.clear_caution;

yielding  = ra.cross_id > 0 && ra.cross_ttc < cfg.dec.yield_gap;

cautious  = ra.min_clear < cfg.dec.clear_caution || ra.ttc < cfg.dec.ttc_caution;

% Room to ease around rather than stop behind. A nudge needs meaningfully more
% lateral space than the safety margin, otherwise it is just a slow squeeze.
room = max(ra.free_left, ra.free_right);
can_nudge = room > cfg.plan.safety_margin + 0.5;

% FOLLOW requires a lead that is actually MOVING in roughly our direction.
% Matching the speed of a stationary object is not following it, it is
% stopping behind it -- and on a market street, where parked carts sit within
% the lateral band the lead test uses, that pinned the speed cap at zero and
% left the vehicle stationary for the whole run with a passable gap beside it.
% A stopped obstacle instead falls through to the caution branch, which is
% what NUDGE exists for: go around.
following = ra.lead_id > 0 && isfinite(ra.lead_gap) && ...
            ra.lead_speed > 0.8 && ...
            ra.lead_gap < max(12.0, 2.0 * ego.v);

if critical && ~unstick
    state = 'STOP';
elseif unstick
    state = 'CREEP';
elseif infeasible && ~stopped
    % GRADUATED ESCALATION. "No plan met the risk tolerance" usually means
    % "not at this speed", not "not at all": the risk a candidate carries
    % scales with how far it sweeps over the prediction horizon. Jumping
    % straight to STOP was measured producing a limit cycle -- accelerate to
    % the speed cap, find nothing acceptable, brake to rest, recover, repeat --
    % with jerk an order of magnitude past the comfort limit and no progress
    % made. Climbing one rung at a time instead lets the vehicle try a lower
    % speed cap first, which shortens the swept path and usually restores
    % feasibility without ever coming to a halt.
    state = local_escalate(prev_state);
elseif recovering
    state = 'CREEP';
elseif yielding
    state = 'YIELD';
elseif cautious
    if can_nudge && ra.n_near <= 4
        state = 'NUDGE';
    else
        state = 'CREEP';
    end
elseif following
    state = 'FOLLOW';
else
    state = 'CRUISE';
end

% ---- hysteresis ---------------------------------------------------------
% Without this the machine chatters between CRUISE and NUDGE as a track's
% estimated clearance jitters by a few centimetres, and the resulting steering
% command oscillates. Escalation is immediate; relaxation has to wait.
sev_new = local_severity(state);
sev_old = local_severity(prev_state);

if sev_new >= sev_old
    fsm.hold = local_hold_cycles(state);
else
    if fsm.hold > 0
        fsm.hold = fsm.hold - 1;
        state    = prev_state;      % not yet allowed to relax
    else
        fsm.hold = local_hold_cycles(state);
    end
end

if strcmp(state, prev_state)
    fsm.t_in = fsm.t_in + 1;
else
    fsm.t_in = 0;
end

fsm.prev  = prev_state;
fsm.state = state;

% ---- emit planner parameters -------------------------------------------
% Lateral freedom is expressed as a FRACTION of the corridor's half-width
% rather than as an absolute distance. A fixed 2.5 m allowance is reasonable on
% a 3.2 m half-width village road, needlessly restrictive on an 8 m wide urban
% carriageway, and wider than the road on a market street. On the urban
% intersection it was the binding constraint: a bus parked at the kerb needed
% about 4.3 m of separation, the corridor had room for it, and the planner was
% not allowed to use it -- so the vehicle stopped six metres short of a bus it
% could comfortably have driven around.
hw = cfg.plan.corridor_halfwidth;

switch state
    case 'CRUISE'
        bp.v_cap    = cfg.plan.v_max;
        bp.d_max    = 0.80 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    case 'FOLLOW'
        % Match the lead, with a gap-dependent trim so the vehicle closes a
        % large gap and eases off a small one.
        desired = max(0, ra.lead_speed);
        if isfinite(ra.lead_gap)
            desired = desired + 0.35 * (ra.lead_gap - max(6.0, 1.5 * ego.v));
        end
        bp.v_cap    = min(cfg.plan.v_max, max(0, desired));
        bp.d_max    = 0.80 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    case 'NUDGE'
        % Widen the lattice and slow down: this is the state that drives
        % around a pushcart or an oncoming rickshaw on an unmarked road.
        bp.v_cap    = cfg.dec.caution_scale * cfg.plan.v_max;
        bp.d_max    = 1.00 * hw;
        bp.risk_tol = 0.8 * cfg.plan.risk_threshold;

    case 'YIELD'
        % Slow enough to let the crossing agent clear, without stopping dead
        % in the road unless it becomes necessary.
        bp.v_cap    = min(cfg.dec.creep_speed * 1.4, cfg.plan.v_max);
        bp.d_max    = 0.70 * hw;
        bp.risk_tol = 0.5 * cfg.plan.risk_threshold;

    case 'CREEP'
        bp.v_cap    = cfg.dec.creep_speed;
        bp.d_max    = 0.95 * hw;
        if unstick
            % Bounded relaxation while edging out of a standstill.
            bp.risk_tol = cfg.dec.stuck_risk_scale * cfg.plan.risk_threshold;
        else
            bp.risk_tol = 0.6 * cfg.plan.risk_threshold;
        end

    case 'STOP'
        % Speed is capped at zero, but the risk tolerance is deliberately NOT
        % tightened. Tightening it here caused a deadlock: a stopped vehicle
        % whose stay-put candidate exceeded the stricter tolerance had no
        % feasible plan at all, which forced STOP, which kept the tolerance
        % strict. The vehicle is already taking the most conservative action
        % available; the tolerance's remaining job is to let it find a way out.
        bp.v_cap    = 0;
        bp.d_max    = 0.60 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    otherwise
        error('sih_behavior_fsm:state', 'Unhandled state "%s".', state);
end

bp.state   = state;
bp.unstick = unstick;
end

% -------------------------------------------------------------------------
function next = local_escalate(state)
%LOCAL_ESCALATE One rung up the caution ladder.
%   CRUISE and FOLLOW both fall back to NUDGE, which keeps the lateral freedom
%   needed to go around whatever is causing the trouble while halving the speed
%   cap. YIELD and NUDGE fall back to CREEP. Only CREEP escalates to STOP, so
%   the vehicle has tried both a reduced speed and a walking pace before it
%   gives up and halts.
switch state
    case {'CRUISE', 'FOLLOW'}, next = 'NUDGE';
    case {'NUDGE', 'YIELD'},   next = 'CREEP';
    case 'CREEP',              next = 'STOP';
    case 'STOP',               next = 'STOP';
    otherwise,                 next = 'CREEP';
end
end

% -------------------------------------------------------------------------
function s = local_severity(state)
%LOCAL_SEVERITY Ordering used to decide whether a transition is an escalation.
switch state
    case 'CRUISE', s = 0;
    case 'FOLLOW', s = 1;
    case 'NUDGE',  s = 2;
    case 'YIELD',  s = 3;
    case 'CREEP',  s = 3;
    case 'STOP',   s = 4;
    otherwise,     s = 0;
end
end

% -------------------------------------------------------------------------
function n = local_hold_cycles(state)
%LOCAL_HOLD_CYCLES Minimum planner cycles to remain in a state before relaxing.
%   At 10 Hz these are fractions of a second: long enough to suppress jitter,
%   short enough that the vehicle does not stay timid after a hazard clears.
switch state
    case 'STOP',   n = 8;
    case 'YIELD',  n = 6;
    case 'CREEP',  n = 6;
    case 'NUDGE',  n = 5;
    case 'FOLLOW', n = 3;
    otherwise,     n = 0;
end
end
