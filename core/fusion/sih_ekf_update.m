function [x, P, nis, valid] = sih_ekf_update(x, P, z, R, gate_chi2)
%SIH_EKF_UPDATE Position measurement update for a CTRV track.
%
%   [x,P,nis,valid] = SIH_EKF_UPDATE(x, P, z, R, gate_chi2) folds in a
%   Cartesian position measurement z = [px; py] with covariance R.
%
%   Every sensor in this stack reports position in the ego frame -- the radar's
%   range/bearing ellipse and the camera's shallow-depth ellipse are rotated
%   into Cartesian covariances by the sensor models -- so the measurement
%   function is linear and only the prediction step needs the EKF Jacobian.
%
%   nis is the normalised innovation squared, which doubles as the association
%   gate statistic. valid is false when nis exceeds gate_chi2, in which case
%   the state is returned unchanged.

if nargin < 5 || isempty(gate_chi2)
    gate_chi2 = Inf;
end

H = [1 0 0 0 0;
     0 1 0 0 0];

y = z(:) - H * x;              % innovation
S = H * P * H.' + R;           % innovation covariance
S = 0.5 * (S + S.');

nis = y.' * (S \ y);

if nis > gate_chi2
    valid = false;
    return;
end
valid = true;

K = (P * H.') / S;

x = x + K * y;
x(4) = sih_wrap_pi(x(4));

% Joseph form: stays positive definite even when K is computed from a
% marginally conditioned S, which happens on tracks that have coasted through
% a long occlusion and have large covariance.
I  = eye(5);
IKH = I - K * H;
P = IKH * P * IKH.' + K * R * K.';
P = 0.5 * (P + P.');
end
