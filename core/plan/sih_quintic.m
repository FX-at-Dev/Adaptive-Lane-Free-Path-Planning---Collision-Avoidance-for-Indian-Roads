function c = sih_quintic(x0, v0, a0, x1, v1, a1, T)
%SIH_QUINTIC Quintic polynomial connecting two full boundary states.
%
%   c = SIH_QUINTIC(x0,v0,a0, x1,v1,a1, T) returns coefficients
%   c = [c0 c1 c2 c3 c4 c5] of
%       x(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5
%   satisfying x(0)=x0, x'(0)=v0, x''(0)=a0 and the same at t=T.
%
%   This is the lateral (d-axis) primitive of the Frenet lattice: lateral
%   motion needs a specified terminal offset, so all six boundary conditions
%   are constrained.

c0 = x0;
c1 = v0;
c2 = 0.5 * a0;

% Remaining three coefficients solve a 3x3 system in the terminal conditions.
T2 = T*T;  T3 = T2*T;  T4 = T3*T;  T5 = T4*T;

A = [    T3,     T4,      T5;
     3*T2,   4*T3,    5*T4;
      6*T,  12*T2,   20*T3];

b = [x1 - (c0 + c1*T + c2*T2);
     v1 - (c1 + 2*c2*T);
     a1 - 2*c2];

c345 = A \ b;
c = [c0, c1, c2, c345(1), c345(2), c345(3)];
end
