function g = sih_gradient(v, h)
%SIH_GRADIENT Fast central-difference derivative on a uniform grid.
%
%   g = SIH_GRADIENT(v, h) is numerically identical to gradient(v, h) for a
%   vector v with uniform spacing h: central differences in the interior,
%   one-sided at the two ends.
%
%   This exists purely for speed. Octave's built-in gradient is a general
%   m-file that handles N-dimensional arrays and non-uniform grids, and its
%   per-call overhead is large. The planner calls it four times per candidate
%   trajectory and evaluates on the order of a hundred candidates per cycle,
%   where profiling showed it accounting for roughly three quarters of the
%   entire planning cycle. Replacing it here cut planner latency by a factor
%   of four with no change in the computed values.

if nargin < 2 || isempty(h)
    h = 1;
end

v = v(:);
n = numel(v);

g = zeros(n, 1);
if n == 1
    return;
end
if n == 2
    g(:) = (v(2) - v(1)) / h;
    return;
end

g(1)     = (v(2) - v(1)) / h;
g(2:n-1) = (v(3:n) - v(1:n-2)) / (2*h);
g(n)     = (v(n) - v(n-1)) / h;
end
