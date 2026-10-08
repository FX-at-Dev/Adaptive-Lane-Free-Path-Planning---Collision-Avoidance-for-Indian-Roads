function [s_start, t_wait] = sih_upstream(rp, s0, v, t_due, dirn)
%SIH_UPSTREAM Where a vehicle due at station s0 at time t_due starts at t = 0.
%
%   [s_start, t_wait] = SIH_UPSTREAM(rp, s0, v, t_due, dirn)
%
%   A vehicle that a scenario wants at s0 at t_due, travelling at v in the
%   direction dirn (+1 with the road, -1 against it), is already driving
%   towards s0 from the start, v * t_due further back along its way. Where
%   that is beyond the end of the road it waits at the end, out of the
%   way, for t_wait and then drives. Nothing appears out of thin air.
s_start = s0 - dirn * v * t_due;
s_clamped = min(max(s_start, 0), rp.length);
t_wait = abs(s_clamped - s_start) / max(v, 0.1);
s_start = s_clamped;
end
