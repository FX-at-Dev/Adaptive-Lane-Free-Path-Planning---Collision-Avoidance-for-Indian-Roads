function [traj, info] = sih_lattice_plan(ego, rp, pred, bp, cfg)
%SIH_LATTICE_PLAN Frenet-frame lattice planner for unstructured roads.
%
%   [traj, info] = SIH_LATTICE_PLAN(ego, rp, pred, bp, cfg)
%
%   Samples a lattice of terminal states around a drivable-corridor reference
%   path, connects the current state to each with polynomial primitives, scores
%   the survivors against the predicted risk field, and returns the cheapest.
%
%   bp is the behaviour parameter set produced by sih_behavior_fsm:
%       bp.v_cap     [m/s]  speed ceiling for the current behaviour
%       bp.d_max     [m]    how far laterally the planner may leave the corridor
%       bp.risk_tol  [0..1] probability mass of conflict that is acceptable
%
%   traj has fields t, x, y, psi, v, a, s, d and diagnostics (cost, risk,
%   min_clear, emergency, feasible).
%
%   THE REFERENCE PATH IS A CORRIDOR CENTRELINE, NOT A LANE. On a road with no
%   markings there is no lane to hold, so the lattice deliberately samples wide
%   lateral offsets and the cost function only mildly prefers the centre. That
%   single choice is what lets the vehicle drift around a pushcart or take the
%   wrong side of a village road to pass an oncoming rickshaw -- normal and
%   necessary driving here rather than a violation.
%
%   Longitudinal motion uses a quartic (terminal position free, terminal speed
%   specified) because the goal is to travel at a sensible speed, not to arrive
%   at a particular station. Lateral motion uses a quintic, because where the
%   vehicle ends up across the road is exactly what is being chosen.
%
%   STRUCTURE. Candidates are generated and filtered in BATCHES rather than one
%   at a time. Every candidate sharing a terminal time shares a time grid, so
%   their polynomials evaluate as a single matrix product, their Cartesian
%   conversion is one call, and their curvature is one differencing pass. Under
%   an interpreter the arrays involved are small enough that per-call overhead
%   rather than arithmetic sets the cycle time, and batching cut measured
%   planner latency by close to an order of magnitude -- which is why this file
%   is written the way it is. Only the collision check stays per-candidate, and
%   only for candidates that survived the kinematic filters: it shares
%   sih_collision_check with the rest of the system so that there is exactly
%   one implementation of the safety test rather than a fast copy and a slow
%   one that can drift apart.

t_start = tic;

% ---- current state in Frenet coordinates ---------------------------------
[s0, d0, k_ref, psi_ref] = sih_cart2frenet(rp, ego.x, ego.y);

dpsi   = sih_wrap_pi(ego.psi - psi_ref);
one_kd = max(1 - k_ref * d0, 0.1);      % guard against the inside of a bend

s0_dot  = ego.v * cos(dpsi) / one_kd;
d0_dot  = ego.v * sin(dpsi);
% Longitudinal planning starts from ZERO acceleration, not from the measured
% one, and this is deliberate.
%
% The measured acceleration is a boundary condition carried into every
% candidate polynomial. Feeding a large negative value in poisons the entire
% lattice: a quartic that must start at -4.5 m/s^2 and still reach the target
% speed within the horizon has to overshoot hard the other way, which then
% violates the acceleration limit, so every candidate is rejected. That is
% self-reinforcing -- no feasible plan produces an emergency stop, the
% emergency stop drives the acceleration further negative, and the vehicle is
% stranded because it once braked hard. It was observed sitting at rest for
% thirty seconds with the nearest obstacle thirty metres away.
%
% Planning from a neutral longitudinal state breaks that loop. Continuity of
% the executed motion is enforced where it belongs, in the controller's
% feed-forward and jerk limiter, rather than by constraining the plan.
s0_ddot = 0;
d0_ddot = 0;

% ---- terminal state lattice ---------------------------------------------
halfwidth = interp1(rp.s, rp.halfwidth, min(max(s0, 0), rp.length), 'linear');
d_limit   = min(bp.d_max, halfwidth);

% Lateral offsets are generated to span the freedom actually allowed, at a
% fixed spatial resolution. A fixed grid was silently capping the planner:
% however much lateral room the behaviour layer granted, the sampled offsets
% never exceeded the grid's own extent, so on a wide carriageway most of the
% road was simply never considered.
n_off = max(3, round(2 * d_limit / cfg.plan.d_step) + 1);
d_set = linspace(-d_limit, d_limit, n_off);

% ALWAYS offer "hold the current line". The offsets above come from a fixed
% grid that has no reason to contain the vehicle's actual lateral position, so
% without this every single candidate demands a sideways shift -- and at low
% speed a sideways shift is exactly what the non-holonomic and curvature
% filters reject. The lattice could therefore be empty for a stopped vehicle
% with a completely clear road ahead, which is how three scenarios ended up
% stuck: no feasible plan, emergency stop, escalate, repeat.
%
% Keeping d0 in the set guarantees at least one candidate whose lateral
% displacement is zero, so "carry straight on" is always available to be
% costed and is never discarded on kinematic grounds.
% The current offset is added UNCLAMPED. Clamping it into the nominal band
% defeats the purpose: a vehicle that has drifted to the edge of the corridor
% while squeezing past an obstacle would find that even "carry straight on"
% demanded a sideways correction, which at low speed is precisely what cannot
% be done. It would then have no feasible candidate at all and sit there
% permanently. Holding the line the vehicle is already on is always physically
% possible, so it is always offered.
d_set = unique([d_set, d0]);

% The lane to keep. India drives on the left: on a road wide enough for
% two lanes the vehicle holds the centre of the left one (sih_lane_centre),
% leaving the right one to oncoming traffic and to overtaking, and when it
% stops -- for a crossing cow, in a queue -- it stops in its own lane
% instead of across the road. A single-lane road is driven down the middle.
d_lane = sih_lane_centre(halfwidth, cfg);
lane_w = 1;
if isfield(bp, 'lane_w'), lane_w = bp.lane_w; end
if abs(d_lane) <= d_limit
    d_set = unique([d_set, d_lane]);
end

% Terminal speeds are sampled over what is REACHABLE from the current speed
% within the horizon, not over a fixed band below the speed cap.
%
% Sampling relative to the cap is wrong whenever the vehicle is far below it.
% A quartic that changes speed by dv over T peaks at 1.5*dv/T, so the largest
% comfortable change is dv = (2/3)*a_max*T. Starting from rest under an 18 m/s
% cap, a band of [15, 18] asks for an acceleration of 7 m/s^2 and every single
% candidate fails the comfort filter -- the planner reports nothing feasible,
% the behaviour layer emergency-stops, escalates to CREEP, creeps forward,
% relaxes to CRUISE, and the whole cycle repeats indefinitely. That limit cycle
% left three of the five scenarios stationary.
v_cap = max(bp.v_cap, 0);

T_max = max(cfg.plan.horizon_T);
reach = (2/3) * T_max;
v_hi  = min(v_cap, ego.v + reach * cfg.veh.a_max);
v_lo  = max(0, ego.v + reach * cfg.veh.a_min);     % a_min is negative
v_lo  = min(v_lo, v_hi);

v_set = unique(max(linspace(v_lo, v_hi, cfg.plan.v_samples), 0));

nD = numel(d_set);
nV = numel(v_set);

best      = [];
best_cost = Inf;

% Least-bad fallback. A candidate that clears every KINEMATIC filter but
% carries more predicted risk than the behaviour layer currently tolerates is
% still far better than nothing: it is drivable, it was collision checked, and
% its risk is known. Keeping the lowest-risk such candidate means the planner
% can always answer, so "no acceptable plan" degrades into "here is the safest
% available plan, flagged as unacceptable" rather than into an emergency stop
% that the vehicle then cannot plan its way out of.
relax       = [];
relax_score = Inf;

n_eval = 0;
n_feas = 0;

% Rejection tally. Which filter is discarding candidates is the single most
% useful thing to know when a scenario stalls, and it is invisible from the
% outside: "no feasible plan" looks identical whether the cause is traffic, a
% speed target the vehicle cannot reach, or a curvature limit. These counters
% are returned in info and cost nothing to maintain.
n_rej = struct('lon', 0, 'slope', 0, 'curv', 0, 'speed', 0, 'alat', 0, 'risk', 0, 'road', 0, 'hold', 0);

% Candidate record for the 3D visualiser (cfg.debug.think). Observation only:
% it is filled alongside the real filters and never read by them. Status codes
% are 0 accepted, 1 over the risk tolerance, 2 geometric conflict, 3 slope
% (non-holonomic), 4 curvature, 5 speed, 6 lateral acceleration.
think = isfield(cfg, 'debug') && isfield(cfg.debug, 'think') && cfg.debug.think;
if think
    cand = struct('x', zeros(cfg.debug.cand_points, 0), ...
                  'y', zeros(cfg.debug.cand_points, 0), ...
                  'cost', zeros(1, 0), 'risk', zeros(1, 0), 'code', zeros(1, 0));
end

for T = cfg.plan.horizon_T
    tv = (0:cfg.pred.dt:T)';
    nT = numel(tv);
    if nT < 3
        continue;
    end

    % ---- batch polynomial evaluation ------------------------------------
    % Power bases shared by every candidate at this terminal time. Columns are
    % ascending powers, matching the coefficient order of sih_quintic and
    % sih_quartic.
    z  = zeros(nT, 1);
    o  = ones(nT, 1);
    P0 = [o, tv, tv.^2, tv.^3,   tv.^4,    tv.^5];
    P1 = [z, o,  2*tv,  3*tv.^2, 4*tv.^3,  5*tv.^4];
    P2 = [z, z,  2*o,   6*tv,    12*tv.^2, 20*tv.^3];
    P3 = [z, z,  z,     6*o,     24*tv,    60*tv.^2];

    LonC = zeros(nV, 5);
    for i = 1:nV
        LonC(i,:) = sih_quartic(s0, s0_dot, s0_ddot, v_set(i), 0, T);
    end

    S   = P0(:,1:5) * LonC.';     % [nT x nV]
    Sd  = P1(:,1:5) * LonC.';
    Sdd = P2(:,1:5) * LonC.';
    Sj  = P3(:,1:5) * LonC.';

    % ---- longitudinal feasibility depends only on the speed sample -------
    % Reversing is outside the model's validity: the bicycle model and the
    % pure-pursuit controller both assume forward motion.
    lon_ok = all(Sd  >= -0.05, 1) & ...
             all(Sdd <=  cfg.veh.a_max * 1.05, 1) & ...
             all(Sdd >=  cfg.veh.a_min * 1.05, 1);

    vi_ok  = find(lon_ok);
    n_eval = n_eval + nD * nV;
    n_rej.lon = n_rej.lon + nD * sum(~lon_ok);
    if isempty(vi_ok)
        continue;
    end

    % ---- expand to every (lateral, longitudinal) pair --------------------
    nP = nD * numel(vi_ok);
    DI = repmat(1:nD, 1, numel(vi_ok));                  % lateral index
    VI = reshape(repmat(vi_ok(:).', nD, 1), 1, nP);      % speed index

    Sp  = S(:, VI);
    Sdp = Sd(:, VI);
    Sjp = Sj(:, VI);

    % ---- lateral motion, parameterised by ARC LENGTH ---------------------
    % d(s), not d(t). This is the single most important detail in the planner.
    %
    % A time-parameterised lateral polynomial asks the vehicle to be a certain
    % distance across the road after a certain TIME, regardless of how far it
    % has travelled. From low speed that means moving sideways before moving
    % forward, and the resulting path curvature is enormous: shifting half a
    % metre while creeping two metres forward needs a 0.9 m turn radius, when
    % the steering lock allows 4.4 m. Every lateral escape was therefore
    % rejected as infeasible -- correctly -- and a vehicle boxed in at the kerb
    % had no way out at all. It is the reason the market street was impassable.
    %
    % Tying lateral displacement to longitudinal progress instead makes the
    % geometry self-consistent: the further the candidate travels, the more it
    % may move across, and curvature stays inside what the steering can do.
    DS    = max(Sp - s0, 0);                 % [nT x nP] distance travelled
    Strav = DS(end, :);                      % [1 x nP] total forward travel

    % dd/ds at the current state. The speed floor keeps this finite at rest.
    dprime0 = d0_dot / max(s0_dot, 0.5);

    LatC = zeros(6, nP);
    for q = 1:nP
        d1q = d_set(DI(q));
        if Strav(q) < 0.5
            % Too little forward travel to move across at all. Holding the
            % current offset is the only honest option, and it is exactly what
            % a real vehicle can do.
            LatC(:, q) = [d0; 0; 0; 0; 0; 0];
        else
            LatC(:, q) = sih_quintic(d0, dprime0, 0, d1q, 0, 0, Strav(q)).';
        end
    end

    % Evaluate d(s) and dd/ds for every pair in six vectorised passes.
    Dp  = zeros(nT, nP);
    DPr = zeros(nT, nP);
    for k = 0:5
        Ck = ones(nT, 1) * LatC(k+1, :);
        Dp = Dp + Ck .* DS.^k;
        if k >= 1
            DPr = DPr + k * Ck .* DS.^(k-1);
        end
    end

    % Lateral velocity and jerk in time, for the comfort terms of the cost.
    Ddp = DPr .* Sdp;
    Djp = sih_gradient_cols(sih_gradient_cols( ...
              sih_gradient_cols(Dp, cfg.pred.dt), cfg.pred.dt), cfg.pred.dt);

    % NON-HOLONOMIC FEASIBILITY. A car cannot translate sideways, but a
    % sideways translation is a straight line in Cartesian space, so it sails
    % through a curvature test unchallenged. From a standstill the lattice was
    % otherwise full of "feasible" candidates that slid the vehicle bodily
    % across the road at zero forward speed; the planner picked one, the
    % controller could not execute it, and nothing moved.
    %
    % The test is on total displacement rather than on pointwise dd/ds. The
    % pointwise form divides by the longitudinal rate, which is near zero at
    % the start of any trajectory that begins from rest, so it reported
    % enormous slopes for perfectly ordinary manoeuvres and rejected them. The
    % displacement form says the same thing where it matters -- you may only
    % move sideways in proportion to how far you travel forward -- and stays
    % well defined at zero speed.
    % With d parameterised by arc length, dd/ds IS the path's slope relative to
    % the corridor, so the non-holonomic bound is now a direct pointwise test
    % rather than the displacement approximation it had to be before.
    holo_ok = all(abs(DPr) <= cfg.plan.max_path_slope, 1);

    % ON THE ROAD, ALL THE WAY. Only the end of a candidate was held to the
    % corridor; the curve on the way there was not. Turned 24 degrees off the
    % road after easing round a cow, every candidate set off along that
    % heading and bowed outward before coming back, and at 12 m/s, replanned
    % from ever further out, the vehicle drove 8 m off the road "on plan".
    % Every sample must keep the vehicle's side inside the road -- or, where
    % it already is outside, no further out than it is now.
    hw_s = interp1(rp.s, rp.halfwidth, min(max(Sp, 0), rp.length), 'linear');   % [nT x nP]
    road_lim = max(hw_s - 0.5 * cfg.ego.width + cfg.plan.road_overhang, abs(d0) + 1e-3);
    road_ok = all(abs(Dp) <= road_lim, 1);
    holo_ok = holo_ok & road_ok;
    n_rej.road = n_rej.road + sum(~road_ok);

    % ---- one Cartesian conversion for every pair ------------------------
    [cxv, cyv, cpv] = sih_frenet2cart(rp, Sp(:), Dp(:), DPr(:));
    CX   = reshape(cxv, nT, nP);
    CY   = reshape(cyv, nT, nP);
    CPSI = reshape(cpv, nT, nP);

    % Speed along the actual Cartesian path.
    V = sqrt((Sdp .* one_kd).^2 + Ddp.^2);

    % ---- one differencing pass for every pair ---------------------------
    % Differentiated with respect to TIME, so dx and dy are velocity components
    % and (dx^2 + dy^2) is the squared speed. That matters for the mask below.
    dx  = sih_gradient_cols(CX, cfg.pred.dt);
    dy  = sih_gradient_cols(CY, cfg.pred.dt);
    ddx = sih_gradient_cols(dx, cfg.pred.dt);
    ddy = sih_gradient_cols(dy, cfg.pred.dt);

    v2  = dx.^2 + dy.^2;
    den = v2.^1.5;
    den(den < 1e-9) = 1e-9;
    KAP = (dx .* ddy - dy .* ddx) ./ den;

    % Curvature is undefined where the vehicle is barely moving, and the
    % formula divides by speed cubed. On any trajectory starting from rest the
    % first samples are nearly coincident, the denominator collapses onto its
    % floor, and the reported curvature reaches the hundreds -- which rejected
    % essentially every candidate that pulled away from a standstill and left
    % the vehicle permanently stationary. A vehicle that is not moving is not
    % turning either, so those samples are masked out rather than clamped.
    KAP(v2 < 0.30^2) = 0;

    max_k  = max(abs(KAP), [], 1);
    max_v  = max(V, [], 1);
    max_al = max(V.^2 .* abs(KAP), [], 1);

    ok_curv  = max_k  <= cfg.metric.curv_limit * 1.5;
    % The cap limits where a candidate may GO, not where the vehicle already
    % is. Every candidate starts at the current speed, so when the behaviour
    % layer lowered the cap below it (easing round a cyclist at 8 m/s under a
    % new 4 m/s cap) a test against the cap alone rejected all of them --
    % including every one that brakes down to it -- and "no plan" became an
    % emergency stop on an empty road. A candidate may not speed up past the
    % cap; slowing towards it is exactly what is wanted.
    ok_speed = max_v  <= max(v_cap, ego.v) + 1.5;
    ok_alat  = max_al <= cfg.veh.a_lat_max * 1.4;

    n_rej.slope = n_rej.slope + sum(~holo_ok);
    n_rej.curv  = n_rej.curv  + sum(holo_ok & ~ok_curv);
    n_rej.speed = n_rej.speed + sum(holo_ok & ok_curv & ~ok_speed);
    n_rej.alat  = n_rej.alat  + sum(holo_ok & ok_curv & ok_speed & ~ok_alat);

    % ROOM TO PULL OUT. No path may end closer than cfg.dec.standoff behind
    % something standing in the way while still in line with it (bp.hold):
    % from a metre behind a cart a car that cannot reverse has no way round
    % it. Pulling out past it, or staying put, is always allowed.
    hold_ok = true(1, nP);
    if isfield(bp, 'hold') && ~isempty(bp.hold)
        nose = cfg.ego.length - 0.5 * (cfg.ego.length - cfg.ego.wheelbase);
        band = 0.5 * cfg.ego.width + 0.3;
        for h = 1:size(bp.hold, 1)
            ends_close = Sp(end, :) + nose > bp.hold(h, 1) & Sp(end, :) < bp.hold(h, 1) + cfg.dec.standoff & Strav > 0.05;
            in_line = Dp(end, :) > bp.hold(h, 2) - band & Dp(end, :) < bp.hold(h, 3) + band;
            hold_ok = hold_ok & ~(ends_close & in_line);
        end
    end
    n_rej.hold = n_rej.hold + sum(holo_ok & ~hold_ok);

    kin_ok = holo_ok & ok_curv & ok_speed & ok_alat & hold_ok;

    if think
        code = zeros(1, nP);
        code(~ok_alat)  = 6;
        code(~ok_speed) = 5;
        code(~ok_curv)  = 4;
        code(~holo_ok)  = 3;
        rej = find(~kin_ok);
        ks  = round(linspace(1, nT, cfg.debug.cand_points));
        cand.x    = [cand.x,    CX(ks, rej)];
        cand.y    = [cand.y,    CY(ks, rej)];
        cand.cost = [cand.cost, NaN(1, numel(rej))];
        cand.risk = [cand.risk, NaN(1, numel(rej))];
        cand.code = [cand.code, code(rej)];
    end

    % ---- collision check, batched over everything that survived ---------
    keep = find(kin_ok);
    if isempty(keep)
        continue;
    end
    [COLL, MINCLR, RISK] = sih_collision_check( ...
        CX(:,keep), CY(:,keep), CPSI(:,keep), tv, pred, cfg);

    for q = 1:numel(keep)
        p         = keep(q);
        collides  = COLL(q);
        min_clear = MINCLR(q);
        risk      = RISK(q);

        d1 = d_set(DI(p));
        v1 = v_set(VI(p));

        cost = local_cost(Sp(:,p), Sdp(:,p), Sjp(:,p), Djp(:,p), d1 - d_lane, v1, T, ...
                          max_k(p), min_clear, risk, s0, v_cap, cfg, lane_w);

        % Building the result struct here, for every surviving candidate, was
        % measured to cost more than the collision check itself: it copies ten
        % array fields per candidate, tens of times per cycle, to throw all but
        % one away. Only the scalars needed for ranking are kept in the loop,
        % and the struct is assembled once at the end for the winner.
        score = risk - 0.01 * min(min_clear, 5);
        if score < relax_score
            relax_score = score;
            relax       = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                                     cost, risk, min_clear, T, d1, v1, cfg, false);
        end

        if think
            ccode = 0;
            if risk > bp.risk_tol
                ccode = 1;
            elseif collides && min_clear < -0.05 && risk > 0.5 * bp.risk_tol
                ccode = 2;
            end
            cand.x    = [cand.x,    CX(ks, p)];
            cand.y    = [cand.y,    CY(ks, p)];
            cand.cost = [cand.cost, cost];
            cand.risk = [cand.risk, risk];
            cand.code = [cand.code, ccode];
        end

        if risk > bp.risk_tol
            n_rej.risk = n_rej.risk + 1;
            continue;
        end
        % Never accept geometric penetration of a likely hypothesis, even when
        % the weighted risk happens to sit under the tolerance.
        if collides && min_clear < -0.05 && risk > 0.5 * bp.risk_tol
            continue;
        end

        n_feas = n_feas + 1;
        if cost < best_cost
            best_cost = cost;
            best      = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                                   cost, risk, min_clear, T, d1, v1, cfg, true);
        end
    end
end

% ---- fallback ladder -----------------------------------------------------
% Three rungs, tried in order of how much they give up:
%   1. a lattice candidate over the risk tolerance but still drivable
%   2. a Hybrid A* search, which can compose manoeuvres the lattice cannot
%   3. emergency braking along the current line
info.hybrid_used     = false;
info.hybrid_expanded = 0;
info.stagnant        = false;

% STAGNATION, not just infeasibility, triggers the search.
%
% An empty lattice is the obvious failure, but it is not the common one. Far
% more often the lattice returns something perfectly feasible whose terminal
% speed is zero -- "stay exactly where you are" is always collision free, so it
% survives every filter and wins on cost once everything that moves has been
% rejected. The vehicle then sits still with a valid plan, indefinitely, which
% no amount of replanning improves.
%
% That is the state the search exists for: boxed in at the kerb, unable to move
% forward because of what is alongside, and unable to move sideways because
% that needs forward travel it cannot afford. Composing several small steering
% actions is the way out, and only a search can express it.
stagnant = ~isempty(best) && best.v_target < 0.30 && ego.v < 0.50 && ...
           bp.v_cap > 0.30;

if isempty(best) || stagnant
    info.stagnant = stagnant;

    % The search is reserved for the LOW-SPEED regime. It is where the lattice
    % genuinely cannot express the answer, and the only regime where spending
    % tens of milliseconds on a search is affordable: at speed, the right
    % response to an infeasible plan is to shed speed, not to go looking for a
    % clever path through.
    % Gated on the behaviour layer's own stuck detection. Running the search
    % on every stagnant cycle tripled the planner's p95 latency and, planning
    % to a thinner margin than the lattice, produced a contact on a scenario
    % that had been clean. Tying it to bp.unstick means it runs only after the
    % vehicle has genuinely been stationary for a few seconds -- rarely, and
    % exactly when nothing else has worked.
    may_search = cfg.plan.hybrid.enable && ...
                 ego.v < cfg.plan.hybrid.max_entry_speed && ...
                 (isempty(best) || (isfield(bp, 'unstick') && bp.unstick));

    if may_search
        [hyb, hinfo] = sih_hybrid_astar(ego, rp, pred, bp, cfg);
        info.hybrid_expanded = hinfo.expanded;
        % Only prefer the search result if it actually makes progress; a
        % stationary answer from it is no better than the one already held.
        % The acceptance threshold is slightly negative, and deliberately so.
        % min_clear is measured against INFLATED footprints -- each carries the
        % safety margin plus the prediction inflation -- so a value of -0.3 m
        % still leaves real physical separation. Demanding a positive value
        % made the escape unreachable in exactly the situation it is for: a
        % vehicle already marginally inside an inflated footprint cannot
        % produce any path with positive clearance, so every result the search
        % found was thrown away and it stayed stuck.
        if ~isempty(hyb) && hyb.min_clear > cfg.plan.hybrid.accept_clear && ...
           (hyb.s(end) - hyb.s(1)) > 1.0
            best = hyb;
            info.hybrid_used = true;
        end
    end

    if isempty(best)
        if ~isempty(relax) && relax.min_clear > 0
            % Drivable and collision-checked, just over the risk tolerance.
            % Flag it and let the behaviour layer decide what that means.
            best = relax;
        else
            best = local_emergency_stop(ego, rp, s0, d0, cfg);
        end
    end
end

traj = best;

info.feasible    = traj.feasible;
info.n_evaluated = n_eval;
info.n_feasible  = n_feas;
info.n_rejected  = n_rej;
info.latency_ms  = toc(t_start) * 1000;
info.s0          = s0;
info.d0          = d0;
info.d_limit     = d_limit;

if think
    info.cand = local_thin_candidates(cand, cfg.debug.max_candidates);
end
end

% -------------------------------------------------------------------------
function c = local_thin_candidates(c, cap)
%LOCAL_THIN_CANDIDATES Keep the cheapest accepted candidates plus a spread of
%   rejected ones, so the picture shows both what was chosen between and what
%   was ruled out, without shipping hundreds of near-duplicate curves.
n = numel(c.code);
if n <= cap
    return;
end
acc = find(c.code == 0);
[~, o] = sort(c.cost(acc));
acc = acc(o);
n_acc = min(numel(acc), round(0.6 * cap));
keep = acc(1:n_acc);
rej = find(c.code ~= 0);
n_rej = min(numel(rej), cap - n_acc);
if n_rej > 0
    keep = [keep, rej(unique(round(linspace(1, numel(rej), n_rej))))];
end
c.x = c.x(:, keep);  c.y = c.y(:, keep);
c.cost = c.cost(keep);  c.risk = c.risk(keep);  c.code = c.code(keep);
end

% -------------------------------------------------------------------------
function c = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                        cost, risk, min_clear, T, d1, v1, cfg, feasible)
%LOCAL_PACK Assemble one candidate column into the trajectory struct.
%   Called only when a candidate becomes the new best or the new fallback, not
%   for every candidate examined.
c = struct('t', tv, 'x', CX(:,p), 'y', CY(:,p), 'psi', CPSI(:,p), ...
           'v', V(:,p), 'a', sih_gradient(V(:,p), cfg.pred.dt), ...
           's', Sp(:,p), 'd', Dp(:,p), 'kappa', KAP(:,p), ...
           'cost', cost, 'risk', risk, 'min_clear', min_clear, ...
           'T', T, 'd_target', d1, 'v_target', v1, ...
           'emergency', false, 'feasible', feasible);
end

% -------------------------------------------------------------------------
function J = local_cost(s_t, sd_t, js_t, jd_t, d1, v1, T, max_k, ...
                        min_clear, risk, s0, v_cap, cfg, lane_w)
%LOCAL_COST Scalar score for one candidate. Lower is better.
%
%   Terms are normalised to roughly comparable magnitudes so the weights in
%   sih_config read as relative priorities rather than as unit conversions.

% Risk dominates by construction.
J = cfg.plan.w_risk * risk;

% Comfort: mean squared jerk in both axes, scaled so a typical 10 m/s^3
% manoeuvre contributes order one.
J = J + cfg.plan.w_jerk * (sih_mean(js_t.^2) + sih_mean(jd_t.^2)) / 100;

% Path curvature.
J = J + cfg.plan.w_curv * max_k^2;

% Progress: penalise falling short of what the speed cap would have achieved.
shortfall = max(0, (s0 + v_cap * T) - s_t(end)) / max(T, 0.1);
J = J + cfg.plan.w_progress * shortfall;

% Preference for the lane centre (d1 is measured from it, sih_lane_centre):
% strong enough to keep to the left lane, weak enough that risk and
% clearance still move the vehicle out of it to pass a cart or a parked car.
J = J + lane_w * cfg.plan.w_offset * d1^2;

% Speed tracking.
J = J + cfg.plan.w_speed_dev * (v1 - v_cap)^2;

% Clearance: reward keeping a buffer, but stop rewarding beyond a metre and
% a half, so the planner does not hug the far edge of an empty road -- and
% does not find passing a parked stall a metre off on a village road about
% as bad as not moving at all, which left it standing behind the stall.
CLEAR_REF = 1.5;
if isfinite(min_clear)
    J = J + cfg.plan.w_clearance * max(0, CLEAR_REF - min_clear)^2;
end

% Discourage dawdling: a candidate that barely moves scores badly even when it
% is perfectly safe, otherwise standing still is an attractive local optimum.
if sih_mean(sd_t) < 0.3
    J = J + 25;
end
end

% -------------------------------------------------------------------------
function traj = local_emergency_stop(ego, rp, s0, d0, cfg)
%LOCAL_EMERGENCY_STOP Straight-line maximum-braking profile along the corridor.
%   Holding the current lateral offset is deliberate: when no plan is feasible
%   the safest action is to shed speed on the path already being followed
%   rather than to swerve into space that was never collision checked.
tv = (0:cfg.pred.dt:max(cfg.plan.horizon_T))';

a  = cfg.veh.a_emergency;
v  = max(ego.v + a * tv, 0);
ds = ego.v * tv + 0.5 * a * tv.^2;

% Freeze the distance once stopped, otherwise the parabola turns back on
% itself and the trajectory reverses.
t_stop = -ego.v / a;
ds(tv > t_stop) = ego.v * t_stop + 0.5 * a * t_stop^2;
ds = max(ds, 0);

s_t = s0 + ds;
d_t = d0 * ones(size(tv));

[cx, cy, cpsi] = sih_frenet2cart(rp, s_t, d_t, zeros(size(tv)));

traj = struct('t', tv, 'x', cx, 'y', cy, 'psi', cpsi, 'v', v, ...
              'a', a * ones(size(tv)), ...
              's', s_t, 'd', d_t, 'kappa', zeros(size(tv)), ...
              'cost', Inf, 'risk', 1, 'min_clear', -Inf, ...
              'T', tv(end), 'd_target', d0, 'v_target', 0, ...
              'emergency', true, 'feasible', false);
end
