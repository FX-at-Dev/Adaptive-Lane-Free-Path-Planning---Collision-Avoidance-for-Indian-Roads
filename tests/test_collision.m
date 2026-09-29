function test_collision()
%TEST_COLLISION Swept-disc collision checking and footprint covering.
%
%   The collision checker is the last line of defence in the planner: if it
%   reports clear when it is not, nothing downstream catches the error. Its
%   geometry is therefore checked against distances computed by hand rather
%   than against another implementation of the same idea.

cfg = sih_config();

% ---- ego footprint covering ---------------------------------------------
[cx, cy, r] = sih_ego_discs(0, 0, 0, cfg);
sih_assert_true(numel(cx) == cfg.ego.n_discs, 'one centre per disc');
sih_assert_close(cy, zeros(1, cfg.ego.n_discs), 1e-12, 'centres on the axis');

% Covering radius is the half-diagonal of an (L/n) x W slice.
seg = cfg.ego.length / cfg.ego.n_discs;
sih_assert_close(r, sqrt((seg/2)^2 + (cfg.ego.width/2)^2), 1e-12, 'covering radius');

% The discs must actually span the footprint: front disc plus radius reaches
% at least the front bumper, rear disc minus radius reaches the rear.
rear_overhang = 0.5 * (cfg.ego.length - cfg.ego.wheelbase);
sih_assert_true(max(cx) + r >= cfg.ego.length - rear_overhang - 1e-9, ...
    'discs must cover the front bumper');
sih_assert_true(min(cx) - r <= -rear_overhang + 1e-9, ...
    'discs must cover the rear bumper');

% Rotating the ego must rotate the disc centres.
[cx90, cy90] = sih_ego_discs(0, 0, pi/2, cfg);
sih_assert_close(cx90, zeros(1, cfg.ego.n_discs), 1e-12, 'rotated centres x');
sih_assert_close(cy90, cx, 1e-12, 'rotated centres y');

% ---- a hand-computed clearance ------------------------------------------
% Ego at the origin facing +x; a single 0.5 m disc parked 10 m ahead. The
% nearest ego disc is the front one, so the clearance follows directly.
pred = local_pred(0.2, 21, 10.0, 0.0, 0.5, 1.0);
te   = (0:0.2:2)';
n    = numel(te);

[collides, min_clear, risk] = sih_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);

expect = 10.0 - max(cx) - (r + cfg.plan.safety_margin + 0.5);
sih_assert_true(~collides, 'a target 10 m ahead must not collide');
sih_assert_close(min_clear, expect, 1e-9, 'hand-computed clearance');
sih_assert_close(risk, 0, 1e-12, 'no risk when clear');

% ---- an unambiguous collision -------------------------------------------
pred = local_pred(0.2, 21, 2.5, 0.0, 0.5, 1.0);
[collides, min_clear, risk] = sih_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
sih_assert_true(collides, 'a target inside the footprint must collide');
sih_assert_true(min_clear < 0, 'penetration must report negative clearance');
sih_assert_close(risk, 1.0, 1e-12, 'certain hypothesis gives full risk');

% ---- risk is weighted by hypothesis probability -------------------------
% Two hypotheses for one agent: an unlikely one in the way, a likely one clear.
pred = local_pred(0.2, 21, 2.5, 0.0, 0.5, 0.3);
far  = local_pred(0.2, 21, 40.0, 0.0, 0.5, 0.7);
pred = local_join(pred, far);

[collides, ~, risk] = sih_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
sih_assert_true(collides, 'conflict with the unlikely hypothesis still counts');
sih_assert_close(risk, 0.3, 1e-12, 'risk equals the conflicting weight only');

% ---- a multi-disc agent must not multiply-count its own probability -----
% A bus modelled as four discs, all in conflict, is still one hypothesis and
% must contribute its weight exactly once.
props = sih_agent_props('bus');
sih_assert_true(props.n_discs >= 3, 'a bus should need several discs');
[bx, by] = sih_agent_discs(3.0, 0.0, 0.0, props);

A = props.n_discs;
K = 21;
pred = struct();
pred.t   = (0:0.2:4)';
pred.x   = ones(K,1) * bx;
pred.y   = ones(K,1) * by;
pred.r   = props.radius * ones(1, A);
pred.w   = 0.4 * ones(1, A);
pred.hyp = ones(1, A);            % all four discs are one hypothesis

[collides, ~, risk] = sih_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
sih_assert_true(collides, 'bus across the bonnet must collide');
sih_assert_close(risk, 0.4, 1e-12, 'multi-disc agent counted once');

% ---- timing: a target that only enters the path later -------------------
% Stationary ego; the agent starts 30 m away and arrives at t = 2 s. A checker
% that ignored time would call this clear.
K = 21;
tt = (0:0.2:4)';
ax = 30 - 13.5 * tt;              % reaches x = 3 at t = 2 s
pred = struct();
pred.t   = tt;
pred.x   = ax;
pred.y   = zeros(K,1);
pred.r   = 0.5;
pred.w   = 1.0;
pred.hyp = 1;

te_short = (0:0.2:1.0)';          % ego trajectory ends before the agent arrives
[c_short, ~, ~] = sih_collision_check(zeros(numel(te_short),1), ...
    zeros(numel(te_short),1), zeros(numel(te_short),1), te_short, pred, cfg);
sih_assert_true(~c_short, 'short horizon must not see the later conflict');

te_long = (0:0.2:3.0)';
[c_long, ~, ~] = sih_collision_check(zeros(numel(te_long),1), ...
    zeros(numel(te_long),1), zeros(numel(te_long),1), te_long, pred, cfg);
sih_assert_true(c_long, 'longer horizon must catch the conflict at t = 2 s');

% ---- empty prediction is safe, not an error -----------------------------
empty_pred = struct('t', (0:0.2:4)', 'x', zeros(21,0), 'y', zeros(21,0), ...
                    'r', zeros(1,0), 'w', zeros(1,0), 'hyp', zeros(1,0));
[c, mc, rk] = sih_collision_check(zeros(n,1), zeros(n,1), zeros(n,1), te, empty_pred, cfg);
sih_assert_true(~c && isinf(mc) && rk == 0, 'empty world is collision free');
end

% -------------------------------------------------------------------------
function pred = local_pred(dt, K, x0, y0, radius, weight)
%LOCAL_PRED One stationary occupancy disc held over the prediction horizon.
pred.t   = (0:dt:(K-1)*dt)';
pred.x   = x0 * ones(K, 1);
pred.y   = y0 * ones(K, 1);
pred.r   = radius;
pred.w   = weight;
pred.hyp = 1;
end

% -------------------------------------------------------------------------
function p = local_join(a, b)
%LOCAL_JOIN Concatenate two prediction sets, renumbering hypothesis ids.
p     = a;
p.x   = [a.x,   b.x];
p.y   = [a.y,   b.y];
p.r   = [a.r,   b.r];
p.w   = [a.w,   b.w];
p.hyp = [a.hyp, b.hyp + max(a.hyp)];
end
