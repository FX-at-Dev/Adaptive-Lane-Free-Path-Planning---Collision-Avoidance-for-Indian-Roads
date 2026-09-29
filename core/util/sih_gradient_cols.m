function G = sih_gradient_cols(M, h)
%SIH_GRADIENT_COLS Central-difference derivative down every column at once.
%
%   G = SIH_GRADIENT_COLS(M, h) applies the same stencil as sih_gradient to
%   each column of M independently: central differences in the interior,
%   one-sided at the first and last row.
%
%   The planner uses this to differentiate every candidate trajectory in a
%   single call. Operating on one [nT x nCandidates] matrix instead of looping
%   over hundreds of 21-element vectors is what keeps the cycle inside its
%   latency budget under an interpreter, where per-call overhead rather than
%   arithmetic dominates.

if nargin < 2 || isempty(h)
    h = 1;
end

[n, ~] = size(M);
G = zeros(size(M));

if n == 1
    return;
end
if n == 2
    d = (M(2,:) - M(1,:)) / h;
    G(1,:) = d;
    G(2,:) = d;
    return;
end

G(1,:)     = (M(2,:) - M(1,:)) / h;
G(2:n-1,:) = (M(3:n,:) - M(1:n-2,:)) / (2*h);
G(n,:)     = (M(n,:) - M(n-1,:)) / h;
end
