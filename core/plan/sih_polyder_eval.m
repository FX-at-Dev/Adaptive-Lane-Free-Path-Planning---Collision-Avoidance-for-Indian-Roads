function [p, v, a, j] = sih_polyder_eval(c, t)
%SIH_POLYDER_EVAL Evaluate a polynomial and its first three derivatives.
%
%   [p,v,a,j] = SIH_POLYDER_EVAL(c, t) where c = [c0 c1 c2 ...] are ascending
%   power coefficients and t is a vector of times. Returns position, velocity,
%   acceleration and jerk, each the same shape as t.
%
%   Coefficients are ascending-power (c0 first) to match sih_quintic and
%   sih_quartic, which is the opposite of MATLAB's built-in polyval ordering,
%   so the evaluation is done explicitly here rather than via polyval.

t = t(:);
n = numel(c);

p = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));
j = zeros(size(t));

for k = 1:n
    e = k - 1;                      % power of this coefficient
    p = p + c(k) * t.^e;
    if e >= 1
        v = v + c(k) * e * t.^(e-1);
    end
    if e >= 2
        a = a + c(k) * e*(e-1) * t.^(e-2);
    end
    if e >= 3
        j = j + c(k) * e*(e-1)*(e-2) * t.^(e-3);
    end
end
end
