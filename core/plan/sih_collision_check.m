function [collides, min_clear, risk] = sih_collision_check(ex, ey, epsi, te, pred, cfg)
%SIH_COLLISION_CHECK Swept-disc check of ego trajectories against predictions.
%
%   [collides, min_clear, risk] = SIH_COLLISION_CHECK(ex, ey, epsi, te, pred, cfg)
%
%   ex, ey, epsi are [nT x nC]: nC candidate trajectories sharing the time grid
%   te. Column vectors (nC = 1) are the ordinary single-trajectory case. All
%   three outputs are [1 x nC].
%
%   pred describes predicted occupancy as a flat list of A discs. One agent
%   contributes (number of hypotheses) x (discs covering its footprint)
%   columns, so a bus under two motion hypotheses occupies eight columns:
%       pred.t    [K x 1]  uniform prediction times starting at 0
%       pred.x    [K x A]  predicted x of each occupancy disc
%       pred.y    [K x A]  predicted y
%       pred.r    [1 x A]  disc radius (already inflated)
%       pred.w    [1 x A]  probability weight of the owning hypothesis
%       pred.hyp  [1 x A]  hypothesis id, shared by the discs of one hypothesis
%
%   Returns, per candidate, whether it intersects any hypothesis, the minimum
%   clearance in metres over the trajectory (negative on penetration), and a
%   probability-weighted risk in [0,1]: the weight of each conflicting
%   hypothesis, scaled by how imminent the conflict is (see cfg.plan.risk_tau).
%
%   A "hypothesis" rather than an "agent" is the unit of risk because
%   prediction is multi-modal -- one rickshaw contributes a straight-on and a
%   cutting-in hypothesis with different weights, and a candidate conflicting
%   only with the unlikely one should be penalised, not rejected.
%
%   WHY IT TAKES A BATCH. This is the planner's innermost operation, and
%   profiling put it at over ninety per cent of a planning cycle when it was
%   called once per candidate: the arrays are small, so interpreter overhead
%   per call dominated the arithmetic inside it. Accepting every candidate at
%   once turns roughly two hundred small calls into a single pass whose inner
%   loop runs over predicted discs instead. The three ego discs are folded into
%   the candidate dimension, so the loop body touches one
%   [nT x nC*n_discs] array per predicted disc.

nT = size(ex, 1);
nC = size(ex, 2);

collides  = false(1, nC);
min_clear = Inf(1, nC);
risk      = zeros(1, nC);

if isempty(pred) || ~isfield(pred, 'x') || isempty(pred.x)
    return;
end

A = size(pred.x, 2);
if A == 0
    return;
end

K  = numel(pred.t);
nd = cfg.ego.n_discs;

% Ego footprint discs for every candidate at every time, computed in one call.
[dcx, dcy, rego] = sih_ego_discs(ex(:), ey(:), epsi(:), cfg);   % [nT*nC x nd]
margin = rego + cfg.plan.safety_margin;

% Fold the disc index into the column dimension: column (i-1)*nC + c is disc i
% of candidate c. One array, one loop level fewer.
EGX = zeros(nT, nC * nd);
EGY = zeros(nT, nC * nd);
for i = 1:nd
    EGX(:, (i-1)*nC + (1:nC)) = reshape(dcx(:, i), nT, nC);
    EGY(:, (i-1)*nC + (1:nC)) = reshape(dcy(:, i), nT, nC);
end

% The prediction grid is uniform from zero, so the time index is arithmetic
% rather than a search. Clamping instead of extrapolating is deliberate: past
% the horizon the final frame is the best estimate available.
ki = round(te(:) / cfg.pred.dt) + 1;
ki = min(max(ki, 1), K);

AX = pred.x(ki, :);            % [nT x A]
AY = pred.y(ki, :);

ones_cols = ones(1, nC * nd);

% Time discount on conflict severity. A conflict at the far end of the horizon
% is weighted far less than an imminent one, because the plan will be revised
% many times before that moment arrives and the prediction that far ahead is
% mostly accumulated uncertainty. severity is in (0,1]: 1 at t = 0, decaying
% with cfg.plan.risk_tau.
if isfield(cfg.plan, 'risk_tau') && cfg.plan.risk_tau > 0
    disc = exp(-te(:) / cfg.plan.risk_tau);
else
    disc = ones(nT, 1);
end
sev = zeros(A, nC);          % discounted conflict severity per hypothesis

for a = 1:A
    dx = EGX - AX(:, a) * ones_cols;
    dy = EGY - AY(:, a) * ones_cols;
    d2 = dx.^2 + dy.^2;

    thresh  = margin + pred.r(a);
    thresh2 = thresh^2;

    % Closest approach of each ego disc column over the whole trajectory.
    mind2 = min(d2, [], 1);                              % [1 x nC*nd]
    clr   = sqrt(mind2) - thresh;

    % Fold the disc columns back down to candidates: a candidate's clearance
    % is the smallest over its discs.
    clr_c = min(reshape(clr, nC, nd), [], 2).';          % [1 x nC]
    min_clear = min(min_clear, clr_c);

    % Worst discounted violation over time, then over the candidate's discs.
    % The discount decreases monotonically with time, so the worst-discounted
    % violation is always the EARLIEST one. max() on a logical array returns
    % both whether any element is true and the index of the first true one, so
    % the severity follows from a single reduction -- there is no need to
    % materialise a discounted copy of the whole distance array, which was
    % measured to triple the cost of this loop.
    [hasv, firstrow] = max(d2 < thresh2, [], 1);         % [1 x nC*nd]
    sev_dc = double(hasv) .* disc(firstrow).';
    sev(a, :) = max(reshape(sev_dc, nC, nd), [], 2).';
end

if ~any(sev(:) > 0)
    return;
end

collides = any(sev > 0, 1);

% ---- weighted risk, summed over distinct hypotheses ---------------------
% Without the de-duplication a four-disc bus would contribute its probability
% four times and saturate the risk term on a single contact.
if isfield(pred, 'hyp') && ~isempty(pred.hyp)
    [hyp_ids, ia] = unique(pred.hyp(:).');
    w_hyp = pred.w(ia);                                  % one weight per hypothesis

    nH      = numel(hyp_ids);
    hyp_sev = zeros(nH, nC);
    for h = 1:nH
        cols = (pred.hyp(:).' == hyp_ids(h));
        hyp_sev(h, :) = max(sev(cols, :), [], 1);        % worst disc of this hypothesis
    end

    risk = w_hyp(:).' * hyp_sev;
else
    risk = pred.w(:).' * sev;
end

% Weights are probabilities over a shared set of agents, so the sum can exceed
% one when several distinct agents are all in conflict. Report the clipped
% value: the planner treats risk as "probability this candidate is unsafe".
risk = min(risk, 1);
end
