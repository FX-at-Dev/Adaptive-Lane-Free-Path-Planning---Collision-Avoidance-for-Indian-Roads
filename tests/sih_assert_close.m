function sih_assert_close(actual, expected, tol, what)
%SIH_ASSERT_CLOSE Assert two numeric arrays agree elementwise within tol.
%   Reports the worst offending element, which makes a failure in a vector
%   comparison (a whole trajectory, say) immediately diagnosable.
if nargin < 4, what = 'value'; end

actual   = double(actual(:));
expected = double(expected(:));

if numel(actual) ~= numel(expected)
    error('sih:assertClose', '%s: size mismatch (%d vs %d).', ...
          what, numel(actual), numel(expected));
end

err = abs(actual - expected);
[worst, i] = max(err);
if ~(worst <= tol)
    error('sih:assertClose', ...
          '%s: max |err| = %.3g > tol %.3g at index %d (got %.10g, want %.10g).', ...
          what, worst, tol, i, actual(i), expected(i));
end
end
