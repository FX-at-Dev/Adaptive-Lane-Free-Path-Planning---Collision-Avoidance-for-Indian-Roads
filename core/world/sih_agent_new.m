function a = sih_agent_new(id, class_name, x, y, psi, v_des, mode, wp, t_spawn)
%SIH_AGENT_NEW Construct a ground-truth road user.
%
%   a = SIH_AGENT_NEW(id, class_name, x, y, psi, v_des, mode, wp, t_spawn)
%
%   mode is one of:
%       'path'   follow the waypoint polyline wp at v_des
%       'static' never move (parked vehicle, debris, pushcart at a stall)
%       'cross'  travel in a straight line along psi at v_des, ignoring wp
%
%   t_spawn is when it sets off. Until then it stands where it starts --
%   a cow waiting at the verge, a pedestrian at the kerb -- in the world and
%   in view of the sensors from the first step. Nothing appears out of thin
%   air in front of the vehicle.

if nargin < 8, wp = []; end
if nargin < 9 || isempty(t_spawn), t_spawn = 0; end

a.id      = id;
a.class   = lower(class_name);
a.props   = sih_agent_props(class_name);
a.x       = x;
a.y       = y;
a.psi     = psi;
a.v       = v_des;
a.v_des   = v_des;
a.mode    = mode;
a.wp      = wp;
a.wp_i    = 1;
a.t_spawn = t_spawn;
a.active  = true;            % false only once removed from the scene
a.done    = false;
a.pass_psi = NaN;            % going round a stopped ego (sih_world_step)
a.pass_dir = NaN;
a.pass_t  = 0;
end
