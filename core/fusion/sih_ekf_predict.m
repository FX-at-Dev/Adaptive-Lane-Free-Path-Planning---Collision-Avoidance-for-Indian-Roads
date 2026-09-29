function [x, P] = sih_ekf_predict(x, P, dt, cfg)
%SIH_EKF_PREDICT Propagate a CTRV track state and covariance.
%
%   [x,P] = SIH_EKF_PREDICT(x, P, dt, cfg) uses cfg.fuse.q_accel and
%   cfg.fuse.q_yawrate as the standard deviations of the unmodelled
%   longitudinal acceleration and yaw acceleration respectively.
%
%   The process noise is built from the two physical noise sources rather than
%   being a hand-tuned diagonal, so that raising q_accel for erratic classes
%   (cattle, pedestrians) correctly inflates position and velocity uncertainty
%   together instead of independently.

[x, F] = sih_ctrv_motion(x, dt);

psi = x(4);

% Noise entering through longitudinal acceleration ...
Ga = [0.5 * dt^2 * cos(psi);
      0.5 * dt^2 * sin(psi);
      dt;
      0;
      0];

% ... and through yaw acceleration.
Gw = [0;
      0;
      0;
      0.5 * dt^2;
      dt];

Q = Ga * (cfg.fuse.q_accel^2)   * Ga.' + ...
    Gw * (cfg.fuse.q_yawrate^2) * Gw.';

P = F * P * F.' + Q;
P = 0.5 * (P + P.');    % keep it symmetric against round-off drift
end
