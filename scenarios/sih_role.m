function a = sih_role(kind, id, cls, rp, varargin)
%SIH_ROLE A road user playing one of the scenario roles, in road coordinates.
%
%   a = SIH_ROLE('parked',  id, cls, rp, s, d)
%       stands still at station s, offset d, along the road
%   a = SIH_ROLE('mover',   id, cls, rp, s0, d0, v, s_list, d_list, t_spawn)
%       drives through the stations s_list at offsets d_list (decreasing s
%       is oncoming traffic), heading for the first of them
%   a = SIH_ROLE('crosser', id, cls, rp, s, d_from, d_to, v, t_spawn)
%       crosses the road at station s with a slight diagonal, as cattle do
%   a = SIH_ROLE('walker',  id, cls, rp, s, d_from, d_to, v, t_spawn)
%       walks straight across the road at station s
%   a = SIH_ROLE('cross_traffic', id, cls, rp, s, d_from, d_to, v)
%       drives across the road along a cross street at station s
%
%   Shared by the scripted scenarios and their random variants (each
%   scenario's seed argument), so both build road users the same way.

switch kind
    case 'parked'
        [s, d] = deal(varargin{1:2});
        [x0, y0] = sih_wp_at(rp, s, d);
        a = sih_agent_new(id, cls, x0, y0, sih_wp_heading(rp, s), 0.0, 'static', [], 0);

    case 'mover'
        [s0, d0, v, s_list, d_list, t_spawn] = deal(varargin{1:6});
        % A vehicle due at s0 at t_spawn is already on the road before
        % that, further back along its way -- it does not appear at s0.
        if t_spawn > 0 && ~isempty(s_list)
            dirn = sign(s_list(end) - s0);
            [s0, t_spawn] = sih_upstream(rp, s0, v, t_spawn, dirn);
        end
        [x0, y0] = sih_wp_at(rp, s0, d0);
        wp = sih_wp_path(rp, s_list, d_list);
        psi0 = sih_wp_heading(rp, s0);
        if size(wp, 1) >= 1
            k = 1;
            if hypot(wp(1,1) - x0, wp(1,2) - y0) < 1e-6 && size(wp, 1) >= 2, k = 2; end
            if hypot(wp(k,1) - x0, wp(k,2) - y0) > 1e-6
                psi0 = atan2(wp(k,2) - y0, wp(k,1) - x0);
            end
        end
        a = sih_agent_new(id, cls, x0, y0, psi0, v, 'path', wp, t_spawn);

    case 'crosser'
        [s, d_from, d_to, v, t_spawn] = deal(varargin{1:5});
        [x0, y0] = sih_wp_at(rp, s, d_from);
        wp = sih_wp_path(rp, [s, s + 2, s + 1, s + 3], [d_from, 0.5 * d_from, 0.4 * d_to, d_to]);
        psi0 = sih_wp_heading(rp, s) + sign(d_to - d_from) * pi / 2;
        a = sih_agent_new(id, cls, x0, y0, psi0, v, 'path', wp, t_spawn);

    case 'walker'
        [s, d_from, d_to, v, t_spawn] = deal(varargin{1:5});
        [x0, y0] = sih_wp_at(rp, s, d_from);
        wp = sih_wp_path(rp, [s, s + 1, s + 2], [d_from, 0, d_to]);
        psi0 = sih_wp_heading(rp, s) + sign(d_to - d_from) * pi / 2;
        a = sih_agent_new(id, cls, x0, y0, psi0, v, 'path', wp, t_spawn);

    case 'cross_traffic'
        [s, d_from, d_to, v] = deal(varargin{1:4});
        [x0, y0] = sih_wp_at(rp, s, d_from);
        wp = sih_wp_path(rp, [s, s, s], [d_from, 0, d_to]);
        psi0 = sih_wp_heading(rp, s) + sign(d_to - d_from) * pi / 2;
        a = sih_agent_new(id, cls, x0, y0, psi0, abs(v), 'path', wp, 0);

    otherwise
        error('sih_role:kind', 'unknown role %s', kind);
end
end
