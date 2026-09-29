function c = sih_quartic(x0, v0, a0, v1, a1, T)
%SIH_QUARTIC Quartic polynomial with a free terminal position.
%
%   c = SIH_QUARTIC(x0,v0,a0, v1,a1, T) returns [c0 c1 c2 c3 c4] of
%       x(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4
%   satisfying x(0)=x0, x'(0)=v0, x''(0)=a0, x'(T)=v1, x''(T)=a1.
%
%   This is the longitudinal (s-axis) primitive for speed-keeping: how far the
%   vehicle travels is an outcome, not a constraint, so terminal position is
%   left free and only the terminal speed and acceleration are specified.

c0 = x0;
c1 = v0;
c2 = 0.5 * a0;

T2 = T*T;  T3 = T2*T;

A = [3*T2,   4*T3;
      6*T,  12*T2];

b = [v1 - (c1 + 2*c2*T);
     a1 - 2*c2];

c34 = A \ b;
c = [c0, c1, c2, c34(1), c34(2)];
end
