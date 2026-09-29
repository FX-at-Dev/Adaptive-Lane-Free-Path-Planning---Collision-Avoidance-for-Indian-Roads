function [assign, total] = sih_hungarian(C)
%SIH_HUNGARIAN Optimal rectangular assignment minimising total cost.
%
%   [assign, total] = SIH_HUNGARIAN(C) for an n x m cost matrix returns
%   assign(i) = column matched to row i (0 when row i is left unmatched, which
%   only happens if n > m), and the total cost of the matching.
%
%   This is the shortest-augmenting-path form of the Hungarian algorithm with
%   dual potentials -- O(n^2 m) and exact. It replaces MATLAB's
%   assignmunkres/matchpairs so that track-to-detection association needs no
%   toolbox and runs identically under Octave.
%
%   Gating is the caller's job: forbidden pairings should be passed in as a
%   large finite cost (see sih_assoc_gnn), not as Inf, because infinities
%   would poison the dual updates.

[n, m] = size(C);
if n == 0 || m == 0
    assign = zeros(n, 1);
    total  = 0;
    return;
end

% The algorithm requires at least as many columns as rows; solve the
% transposed problem and map the result back when that is not the case.
transposed = false;
if n > m
    C = C.';
    [n, m] = size(C);
    transposed = true;
end

BIG = 1e18;

u   = zeros(n+1, 1);   % row potentials;    index r+1, index 1 is the dummy row
v   = zeros(m+1, 1);   % column potentials; index c+1, index 1 is the dummy col
p   = zeros(m+1, 1);   % p(c+1) = row currently matched to column c (0 = none)
way = zeros(m+1, 1);   % predecessor column on the augmenting path

for i = 1:n
    p(1) = i;
    j0   = 0;                      % start from the dummy column
    minv = BIG * ones(m+1, 1);
    used = false(m+1, 1);

    % --- grow a shortest augmenting path until it reaches a free column ---
    while true
        used(j0+1) = true;
        i0 = p(j0+1);

        idx = find(~used(2:m+1));               % candidate columns, 1..m
        cur = C(i0, idx).' - u(i0+1) - v(idx+1);

        better = cur < minv(idx+1);
        minv(idx(better)+1) = cur(better);
        way(idx(better)+1)  = j0;

        [delta, k] = min(minv(idx+1));
        j1 = idx(k);

        % --- shift the potentials so the path stays tight -----------------
        % Rows appearing in p(used) are distinct by construction, so this
        % indexed update cannot silently drop a duplicate.
        uidx = find(used);
        u(p(uidx)+1) = u(p(uidx)+1) + delta;
        v(uidx)      = v(uidx) - delta;

        nidx = find(~used);
        minv(nidx) = minv(nidx) - delta;

        j0 = j1;
        if p(j0+1) == 0
            break;                 % reached a free column: path complete
        end
    end

    % --- walk the path backwards, flipping the matching ------------------
    while true
        j1       = way(j0+1);
        p(j0+1)  = p(j1+1);
        j0       = j1;
        if j0 == 0
            break;
        end
    end
end

assign = zeros(n, 1);
for j = 1:m
    if p(j+1) > 0
        assign(p(j+1)) = j;
    end
end

if transposed
    % assign maps (former) columns to (former) rows; invert it.
    inv_assign = zeros(m, 1);
    for r = 1:n
        if assign(r) > 0
            inv_assign(assign(r)) = r;
        end
    end
    assign = inv_assign;
end

total = 0;
if transposed
    C = C.';
end
for r = 1:numel(assign)
    if assign(r) > 0
        total = total + C(r, assign(r));
    end
end
end
