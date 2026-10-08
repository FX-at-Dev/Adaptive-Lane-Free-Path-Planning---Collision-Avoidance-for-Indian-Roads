function [ok, why] = sih_scn_validate(scn, cfg)
%SIH_SCN_VALIDATE Can this scenario happen on this road?
%
%   [ok, why] = SIH_SCN_VALIDATE(scn, cfg) checks a scenario -- usually a
%   random draw -- against the rules of the road and of the test:
%     * no two road users overlap where they start
%     * the ego's start is clear: nothing within 8 m of it at t = 0
%     * vehicles start on the road (within the corridor), walkers and
%       animals anywhere
%     * every parked obstacle, alone or with another one beside it, leaves
%       a gap the ego fits through: its width plus 1.2 m
%   why names the first rule broken, '' when ok.

ok = false;
rp = scn.rp;
hw = cfg.plan.corridor_halfwidth;
need = cfg.ego.width + 1.2;
A = scn.agents;
n = numel(A);
walkers = {'pedestrian', 'cattle'};

% ---- where each starts, in road coordinates ---------------------------------
s0 = zeros(1, n); d0 = zeros(1, n);
for i = 1:n
    [s0(i), d0(i)] = sih_cart2frenet(rp, A(i).x, A(i).y);
end

for i = 1:n
    if ~any(strcmp(A(i).class, walkers)) && ~strcmp(A(i).mode, 'path') && abs(d0(i)) > hw + 0.5
        why = sprintf('%s #%d parked off the road', A(i).class, A(i).id);
        return;
    end
    if ~any(strcmp(A(i).class, walkers)) && strcmp(A(i).mode, 'path') && abs(d0(i)) > hw + 1.0 && ...
            ~(isfield(scn, 'cross_streets') && any(abs(s0(i) - scn.cross_streets(:, 1)) < 15))
        why = sprintf('%s #%d starts off the road', A(i).class, A(i).id);
        return;
    end
    if A(i).t_spawn <= 0 && hypot(A(i).x - scn.ego.x, A(i).y - scn.ego.y) < 8
        why = sprintf('%s #%d on the ego start', A(i).class, A(i).id);
        return;
    end
end

% ---- no overlaps at the start ---------------------------------------------------
for i = 1:n
    [ax, ay] = sih_agent_discs(A(i).x, A(i).y, A(i).psi, A(i).props);
    for j = i+1:n
        if A(i).t_spawn ~= A(j).t_spawn && (A(i).t_spawn > 0 || A(j).t_spawn > 0)
            continue;                % never there at the same time at the start
        end
        [bx, by] = sih_agent_discs(A(j).x, A(j).y, A(j).psi, A(j).props);
        gap = min(min(hypot(ax(:) - bx(:).', ay(:) - by(:).'))) - A(i).props.radius - A(j).props.radius;
        if gap < 0.5
            why = sprintf('%s #%d overlaps %s #%d', A(i).class, A(i).id, A(j).class, A(j).id);
            return;
        end
    end
end

% ---- a passable gap past parked obstacles ------------------------------------------
st = find(strcmp({A.mode}, 'static'));
for i = st
    lo = d0(i) - A(i).props.width / 2;      % its extent across the road
    hi = d0(i) + A(i).props.width / 2;
    for j = st
        if j ~= i && abs(s0(j) - s0(i)) < (A(i).props.length + A(j).props.length) / 2 + 4
            lo = min(lo, d0(j) - A(j).props.width / 2);   % two side by side block together
            hi = max(hi, d0(j) + A(j).props.width / 2);
        end
    end
    room = max(hw - hi, lo + hw);        % the wider side
    if room < need
        why = sprintf('%s #%d leaves %.1f m, the ego needs %.1f m', A(i).class, A(i).id, room, need);
        return;
    end
end

ok = true;
why = '';
end
