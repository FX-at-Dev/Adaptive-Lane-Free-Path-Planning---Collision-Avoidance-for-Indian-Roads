function [min_clear, collided, worst_id] = sih_clearance_truth(world, cfg)
%SIH_CLEARANCE_TRUTH Ground-truth clearance between the ego and every agent.
%
%   [min_clear, collided, worst_id] = SIH_CLEARANCE_TRUTH(world, cfg)
%
%   Measured against the true agent states, not the tracked estimates. Safety
%   claims made from the vehicle's own perception would be circular: a tracker
%   that lost an agent would report a comfortable clearance right up to the
%   impact. This is the number the metrics report quotes.

min_clear = Inf;
collided  = false;
worst_id  = 0;

[ecx, ecy, erad] = sih_ego_discs(world.ego.x, world.ego.y, world.ego.psi, cfg);

for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end

    [acx, acy] = sih_agent_discs(a.x, a.y, a.psi, a.props);

    d = Inf;
    for i = 1:numel(ecx)
        d = min(d, min(sqrt((ecx(i) - acx).^2 + (ecy(i) - acy).^2)));
    end

    clear_k = d - erad - a.props.radius;
    if clear_k < min_clear
        min_clear = clear_k;
        worst_id  = a.id;
    end
end

collided = (min_clear < 0);
end
