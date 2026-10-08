function m = sih_mean(x, dim)
%SIH_MEAN Arithmetic mean, bit-identical to mean() and without its overhead.
%
%   m = SIH_MEAN(x)       mean along the first non-singleton dimension
%   m = SIH_MEAN(x, dim)  mean along dim
%
%   Octave's mean is an m-file that validates its arguments on every call and
%   then computes sum(x, dim, "extra") / n. The planner calls it tens of
%   thousands of times per run on short vectors, where the checks cost far
%   more than the arithmetic -- profiling put it above the planner itself.
%   This calls the same compensated sum directly, so the result is
%   bit-identical. MATLAB's sum has no "extra" option and its mean is
%   built in, so there it is used as is.

persistent octave
if isempty(octave)
    octave = sih_is_octave();
end

if nargin < 2
    dim = find(size(x) ~= 1, 1);
    if isempty(dim)
        dim = 1;
    end
end

if octave
    if isempty(x) && ndims(x) == 2 && nargin < 2 && all(size(x) == 0)
        m = NaN;                % mean([]) is NaN, not empty
        return;
    end
    m = sum(x, dim, 'extra') ./ size(x, dim);
else
    m = mean(x, dim);
end
end
