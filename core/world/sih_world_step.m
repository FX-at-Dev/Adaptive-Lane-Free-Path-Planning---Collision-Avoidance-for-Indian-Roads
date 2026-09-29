function world = sih_world_step(world, dt, cfg)
%SIH_WORLD_STEP Advance every ground-truth agent by one simulation step.
%
%   Agents steer toward their next waypoint with a class-dependent yaw rate
%   limit, and erratic classes accumulate heading noise. That noise is the
%   mechanism by which cattle and pedestrians become genuinely unpredictable
%   rather than merely unknown: the predictor cannot recover a motion that the
%   world itself generates stochastically, which is the honest way to test
%   whether the planner's uncertainty handling actually works.

t = world.t + dt;

for k = 1:numel(world.agents)
    a = world.agents(k);

    if ~a.active
        if t >= a.t_spawn
            a.active = true;       % event-triggered entry (e.g. cattle)
        else
            world.agents(k) = a;
            continue;
        end
    end

    % Only agents explicitly declared 'static' hold station. An agent that has
    % merely run out of waypoints keeps driving in a straight line and leaves
    % the scene -- this guard used to zero its speed and skip the rest of the
    % step, which quietly defeated that and left road users frozen exactly
    % where their path happened to end. A cyclist stopped dead five metres
    % short of the goal was the last thing blocking the market street.
    if strcmp(a.mode, 'static')
        a.v = 0;
        world.agents(k) = a;
        continue;
    end

    p = a.props;

    % ---- react to the ego vehicle ---------------------------------------
    % Other road users are not oblivious: they brake for a vehicle in their
    % way. Without this an agent following fixed waypoints drives straight
    % through a stopped ego and books a "collision" that says nothing about
    % the planner. How readily an agent yields is class dependent, so a bus
    % concedes far less than a cyclist.
    ex = world.ego.x - a.x;
    ey = world.ego.y - a.y;
    fwd = cos(a.psi) * ex + sin(a.psi) * ey;      % ego ahead of this agent
    lat = -sin(a.psi) * ex + cos(a.psi) * ey;

    % How readily an agent yields sets how EARLY it reacts, not whether it
    % reacts at all. An earlier version blended the braking factor towards one
    % for low-yield classes, which left an auto-rickshaw still doing 65% of its
    % cruising speed as it reached a stationary vehicle and drove straight into
    % it. Nobody does that: stopping for an obstacle in your path is
    % self-preservation, not courtesy.
    % Vehicles give way to the ego; pedestrians and animals do not.
    %
    % A crossing agent that slows for the ego while the ego is waiting for it
    % to cross produces a deadlock in which both creep at a fraction of walking
    % pace and neither clears. Real pedestrians and cattle commit to a crossing
    % and complete it, and modelling them that way keeps the scenario testing
    % the vehicle's behaviour rather than the traffic model's.
    is_crosser = strcmp(p.class, 'pedestrian') || strcmp(p.class, 'cattle');

    d_react = 6.0 + 10.0 * p.yields;
    react   = 1.0;
    avoid   = 0;
    % A crossing agent will not stop several metres short of a waiting vehicle
    % -- that was the deadlock -- but it will not walk through one either. At
    % very close range it halts, which prevents a pedestrian from registering a
    % collision against a stationary ego while leaving the committed-crossing
    % behaviour intact everywhere else.
    if is_crosser && fwd > 0 && fwd < 2.2 && abs(lat) < 1.6
        react = 0;
    end

    if ~is_crosser && fwd > 0 && fwd < d_react && abs(lat) < 3.2
        strength = 1 - fwd / d_react;

        % Slow down, but not to a standstill. Braking alone produced a mutual
        % deadlock: the ego stopped for the oncoming rickshaw, the rickshaw
        % stopped for the ego, and neither ever moved again. That is an
        % artefact of modelling other road users as though braking were their
        % only option.
        % A vehicle closing on a stopped obstacle comes to a halt. The floor
        % used to sit at 20% of cruising speed to avoid a mutual deadlock, but
        % that meant an oncoming rickshaw crept into a stationary ego and
        % registered as a collision the planner could do nothing about. The
        % deadlock it was guarding against is gone now that crossing agents are
        % exempt from reacting at all, so vehicles are allowed to stop.
        react = (fwd - 3.0) / max(d_react - 3.0, 1.0);
        react = min(max(react, 0), 1);

        % And ease sideways, away from the ego. This is the characteristic
        % behaviour of the traffic being modelled: on an unmarked road, two
        % vehicles meeting head-on both drift aside and pass rather than
        % negotiating right of way. Without it the surrounding traffic is
        % unrealistically rigid and the scenario tests deadlock handling
        % instead of path planning.
        % The gain is set so that oncoming traffic clears a STATIONARY ego with
        % real room. At a weaker setting a rickshaw passing a stopped car on a
        % 6.4 m carriageway grazed it by a few centimetres, which scored as a
        % collision while telling us nothing about the planner.
        if lat >= 0
            avoid = -1.1 * strength;    % ego is to the left, so steer right
        else
            avoid =  1.1 * strength;
        end
    end

    % ---- desired heading -------------------------------------------------
    if strcmp(a.mode, 'cross')
        psi_des = a.psi;           % straight line, heading fixed at spawn
    else
        % An agent that runs out of waypoints CARRIES ON in a straight line
        % rather than stopping where it stands.
        %
        % Stopping was a persistent source of false failures: a pedestrian
        % whose crossing ended a little short froze mid-carriageway and became
        % a permanent obstacle, and the vehicle -- entirely correctly -- refused
        % to drive through it. The scenario then looked like a planner failure
        % when it was an artefact of the traffic model. Road users leave the
        % scene; only agents explicitly declared 'static' stay put.
        if isempty(a.wp) || a.wp_i > size(a.wp, 1)
            a.done  = true;
            psi_des = a.psi;
        else
            tgt = a.wp(a.wp_i, :);
            if hypot(tgt(1) - a.x, tgt(2) - a.y) < max(1.5, 0.6 * a.v)
                a.wp_i = a.wp_i + 1;
            end
            if a.wp_i > size(a.wp, 1)
                a.done  = true;
                psi_des = a.psi;
            else
                tgt = a.wp(a.wp_i, :);
                psi_des = atan2(tgt(2) - a.y, tgt(1) - a.x);
            end
        end
    end

    % ---- yaw rate limit, tighter for long vehicles -----------------------
    yaw_max = 2.4 / max(1.0, p.length / 2);
    dpsi    = sih_wrap_pi(psi_des + avoid - a.psi);
    a.psi   = a.psi + min(max(dpsi, -yaw_max*dt), yaw_max*dt);

    % ---- class-dependent erratic heading noise ---------------------------
    if p.erratic > 0
        a.psi = a.psi + p.heading_noise * p.erratic * sqrt(dt) * randn();
    end
    a.psi = sih_wrap_pi(a.psi);

    % ---- speed tracking with a little variability ------------------------
    v_target = react * a.v_des * (1 + 0.05 * p.erratic * randn());
    a.v = a.v + min(max(v_target - a.v, -p.a_brake*dt), p.a_brake*dt);
    a.v = min(max(a.v, 0), p.v_max);

    a.x = a.x + a.v * cos(a.psi) * dt;
    a.y = a.y + a.v * sin(a.psi) * dt;

    world.agents(k) = a;
end

world.t = t;
end
