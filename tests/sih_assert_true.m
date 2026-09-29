function sih_assert_true(cond, msg, varargin)
%SIH_ASSERT_TRUE Assert a scalar condition, with a printf-style message.
if ~all(cond(:))
    error('sih:assertTrue', msg, varargin{:});
end
end
