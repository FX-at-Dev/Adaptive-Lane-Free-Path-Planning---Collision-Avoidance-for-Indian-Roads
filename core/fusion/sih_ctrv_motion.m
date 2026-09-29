function [xp, F] = sih_ctrv_motion(x, dt)
%SIH_CTRV_MOTION Constant turn-rate and velocity propagation with Jacobian.
%
%   [xp, F] = SIH_CTRV_MOTION(x, dt) advances the state
%       x = [px; py; v; psi; omega]
%   and returns the Jacobian F = d(xp)/d(x).
%
%   CTRV is the right base model for Indian mixed traffic: unlike a constant
%   velocity model it represents the tight, sustained turns that rickshaws and
%   two-wheelers actually execute when threading through a gap. The straight
%   line case is a removable singularity at omega = 0 and is handled by the
%   small-angle branch below, which keeps both the state and the Jacobian
%   continuous through omega = 0.

px = x(1); py = x(2); v = x(3); psi = x(4); w = x(5);

c0 = cos(psi);
s0 = sin(psi);

OMEGA_EPS = 1e-4;

if abs(w) > OMEGA_EPS
    psi1 = psi + w*dt;
    c1 = cos(psi1);
    s1 = sin(psi1);

    xp = [px + (v/w) * (s1 - s0);
          py + (v/w) * (c0 - c1);
          v;
          psi1;
          w];

    F = eye(5);
    F(1,3) = (s1 - s0) / w;
    F(1,4) = (v/w) * (c1 - c0);
    F(1,5) = (v*dt*c1)/w - v*(s1 - s0)/w^2;

    F(2,3) = (c0 - c1) / w;
    F(2,4) = (v/w) * (s1 - s0);
    F(2,5) = (v*dt*s1)/w - v*(c0 - c1)/w^2;

    F(4,5) = dt;
else
    % Near-straight limit. Taking the w -> 0 limit of the exact formulae and
    % keeping terms through second order in dt gives
    %     px+ = px + v cos(psi) dt - (1/2) v sin(psi) w dt^2
    %     py+ = py + v sin(psi) dt + (1/2) v cos(psi) w dt^2
    % which is continuous with the exact branch at the switch-over and, more
    % importantly, still depends on w. A pure straight-line propagation would
    % not, and its Jacobian would drop the heading/turn-rate coupling exactly
    % when a track is drifting through zero yaw rate -- precisely the moment a
    % rickshaw straightens up before cutting across.
    dt2 = dt^2;

    xp = [px + v*c0*dt - 0.5*v*s0*w*dt2;
          py + v*s0*dt + 0.5*v*c0*w*dt2;
          v;
          psi + w*dt;
          w];

    F = eye(5);
    F(1,3) =  c0*dt - 0.5*s0*w*dt2;
    F(1,4) = -v*s0*dt - 0.5*v*c0*w*dt2;
    F(1,5) = -0.5*v*s0*dt2;

    F(2,3) =  s0*dt + 0.5*c0*w*dt2;
    F(2,4) =  v*c0*dt - 0.5*v*s0*w*dt2;
    F(2,5) =  0.5*v*c0*dt2;

    F(4,5) = dt;
end

xp(4) = sih_wrap_pi(xp(4));
end
