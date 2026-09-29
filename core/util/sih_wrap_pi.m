function a = sih_wrap_pi(a)
%SIH_WRAP_PI Wrap angles to the interval (-pi, pi].
a = mod(a + pi, 2*pi) - pi;
% mod maps exactly -pi to -pi; fold it to +pi so the range is half-open the
% way downstream heading-error arithmetic assumes.
a(a == -pi) = pi;
end
