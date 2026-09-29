function test_hungarian()
%TEST_HUNGARIAN Assignment solver matched against brute-force optimum.
%
%   Track-to-detection association is where a subtle optimality bug would be
%   least visible -- a slightly wrong matching still produces plausible-looking
%   tracks -- so the solver is checked exhaustively against every permutation
%   on small problems, including rectangular ones in both orientations.

sih_rng(12345);

% ---- square problems, exhaustive check ----------------------------------
for n = 1:6
    for trial = 1:12
        C = round(rand(n, n) * 20);
        [assign, total] = sih_hungarian(C);

        sih_assert_true(numel(assign) == n, 'assign length for %dx%d', n, n);
        sih_assert_true(numel(unique(assign)) == n, 'assignment must be a permutation');
        sih_assert_true(all(assign >= 1 & assign <= n), 'assignment in range');

        best = local_brute_force(C);
        sih_assert_close(total, best, 1e-9, sprintf('optimal cost n=%d', n));

        % Reported total must actually match the returned assignment.
        chk = 0;
        for r = 1:n
            chk = chk + C(r, assign(r));
        end
        sih_assert_close(total, chk, 1e-9, 'total matches assignment');
    end
end

% ---- wide problems (more detections than tracks): every row matched -----
for trial = 1:12
    n = 3; m = 6;
    C = round(rand(n, m) * 20);
    [assign, total] = sih_hungarian(C);
    sih_assert_true(numel(assign) == n, 'wide: one entry per row');
    sih_assert_true(all(assign >= 1), 'wide: every row matched');
    sih_assert_true(numel(unique(assign)) == n, 'wide: columns used at most once');
    sih_assert_close(total, local_brute_force(C), 1e-9, 'wide optimal cost');
end

% ---- tall problems (more tracks than detections): surplus rows unmatched
for trial = 1:12
    n = 6; m = 3;
    C = round(rand(n, m) * 20);
    [assign, total] = sih_hungarian(C);
    sih_assert_true(numel(assign) == n, 'tall: one entry per row');
    sih_assert_true(sum(assign > 0) == m, 'tall: exactly m rows matched');
    matched = assign(assign > 0);
    sih_assert_true(numel(unique(matched)) == m, 'tall: columns used once');
    sih_assert_close(total, local_brute_force(C), 1e-9, 'tall optimal cost');
end

% ---- degenerate inputs --------------------------------------------------
[a, t] = sih_hungarian(zeros(0, 4));
sih_assert_true(isempty(a) && t == 0, 'empty rows handled');
[a, t] = sih_hungarian(zeros(3, 0));
sih_assert_true(numel(a) == 3 && all(a == 0) && t == 0, 'empty columns handled');

% ---- a known optimum that greedy matching gets wrong --------------------
% Greedy takes the global minimum 1 at (1,1) first, which strands row 2 with
% the 100 at (2,2) for a total of 101. The optimal pairing avoids the tempting
% cell entirely: 2 at (1,2) plus 3 at (2,1) for 5. This is the failure mode
% that matters in tracking -- a cheap wrong association early in the frame
% forces an absurd one later -- so it is asserted explicitly.
C = [1 2; 3 100];
[assign, total] = sih_hungarian(C);
sih_assert_close(total, 5, 1e-12, 'greedy trap total');
sih_assert_true(assign(1) == 2 && assign(2) == 1, 'greedy trap assignment');
end

% -------------------------------------------------------------------------
function best = local_brute_force(C)
%LOCAL_BRUTE_FORCE Minimum-cost matching by enumerating every injective map.
%   Solves the transposed problem when there are more rows than columns, so a
%   single enumeration covers square, wide and tall cases alike.
[n, m] = size(C);
if n > m
    C = C.';
    [n, m] = size(C);
end
cols = perms(1:m);
best = Inf;
for r = 1:size(cols, 1)
    pick = cols(r, 1:n);
    c    = sum(C(sub2ind(size(C), (1:n).', pick(:))));
    best = min(best, c);
end
end
