function [cx, cy] = sih_agent_discs(x, y, psi, props)
%SIH_AGENT_DISCS Disc centres covering an agent footprint.
%
%   [cx, cy] = SIH_AGENT_DISCS(x, y, psi, props) places props.n_discs disc
%   centres along the agent's longitudinal axis. x, y, psi may be column
%   vectors; cx and cy are then [numel(x) x props.n_discs].
%
%   The agent pose is its geometric CENTRE (unlike the ego, whose pose is the
%   rear axle), because tracked agents are estimated as point targets at their
%   centroid and there is no axle to speak of for a pedestrian or a cow.

n   = props.n_discs;
seg = props.length / n;

% Offsets measured from the centre of the footprint.
offs = -props.length/2 + seg * ((1:n) - 0.5);

x = x(:); y = y(:); psi = psi(:);
cx = x * ones(1, n) + cos(psi) * offs;
cy = y * ones(1, n) + sin(psi) * offs;
end
