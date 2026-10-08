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

% The ego's footprint, for agents deciding whether it is in their way.
[ecx, ecy, erad] = sih_ego_discs(world.ego.x, world.ego.y, world.ego.psi, cfg);

for k = 1:numel(world.agents)
    a = world.agents(k);

    if ~a.active
        continue;                  % removed from the scene
    end
    if t < a.t_spawn
        % Waiting for its moment -- at the verge, at the kerb -- in plain
        % view of the vehicle's sensors.
        a.v = 0;
        world.agents(k) = a;
        continue;
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
    %
    % "In its way" is judged against the vehicle's whole footprint, not its
    % reference point: the ego's pose is its rear axle, 3.35 m behind its
    % nose, and a pedestrian crossing diagonally in front of a stopped car
    % used to walk into the front corner because the axle looked clear.
    if is_crosser
        for q = 1:numel(ecx)        % not k: that is the agent being stepped
            qx = ecx(q) - a.x;
            qy = ecy(q) - a.y;
            qf = cos(a.psi) * qx + sin(a.psi) * qy;
            ql = -sin(a.psi) * qx + cos(a.psi) * qy;
            % Its own body counts: a cow's nose is 1.1 m ahead of its centre.
            if qf > 0 && qf < erad + p.length / 2 + 0.6 && abs(ql) < erad + p.width / 2 + 0.4
                react = 0;
            end
        end
    end
    if is_crosser && fwd > 0 && fwd < 2.2 && abs(lat) < 1.6
        react = 0;
    end
    % Halted in front of a car that is itself standing still, a walker or an
    % animal goes round it rather than waiting: both waiting for the other
    % kept a car and a pedestrian half a metre apart for twenty seconds. It
    % turns, away from the car first, to the nearest heading along which a
    % second's walk does not bring it closer; the halt above then releases
    % it once it faces that way.
    side_psi = NaN;
    if is_crosser && react == 0 && world.ego.v < 0.3
        c_now = local_clear_after(a, a.psi, 0, p, ecx, ecy, erad);
        need = max(0.3, min(c_now, 0.8));
        away = -1;
        if lat < 0, away = 1; end
        for turn = [0.5, 1.0, 1.5, 2.0]
            for sgn = [away, -away]
                h = a.psi + sgn * turn;
                if local_clear_after(a, h, min(a.v_des, 1.2), p, ecx, ecy, erad) >= need
                    side_psi = h;
                    break;
                end
            end
            if isfinite(side_psi), break; end
        end
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

    % ---- never drive into the ego --------------------------------------------
    % If a vehicle's path would actually hit the ego's footprint -- not merely
    % pass near it -- it is held to a speed it can still stop from before it
    % gets there: v <= sqrt(2 a d). Measured to the footprint, not the rear
    % axle (head-on, the ego's nose is 3.35 m nearer), and out to 40 m, since
    % a fast vehicle needs more room than its reaction distance. Only a true
    % collision course is affected, so a vehicle that can pass beside the
    % ego still does, and the ego -- the planner -- deals with one that stops.
    if ~is_crosser
        for q = 1:numel(ecx)
            qx = ecx(q) - a.x;
            qy = ecy(q) - a.y;
            qf = cos(a.psi) * qx + sin(a.psi) * qy;
            ql = -sin(a.psi) * qx + cos(a.psi) * qy;
            if qf > 0 && qf < 40 && abs(ql) < erad + p.width / 2 + 0.3
                gap = qf - erad - p.length / 2 - 1.0;
                v_safe = sqrt(2 * 0.8 * p.a_brake * max(gap, 0));
                react = min(react, v_safe / max(a.v_des, 0.1));
            end
        end
        % Halted by a stopped ego, it goes round: a driver turns the wheel
        % and creeps past rather than waiting for ever for a car that
        % cannot reverse. It commits to the smallest turn, away from the
        % ego first, along which a second's creep keeps it clear and no
        % closer than it already is, and holds that heading, creeping,
        % until the ego is behind it (or moves off, or ten seconds pass).
        c_now = local_clear_after(a, a.psi, 0, p, ecx, ecy, erad);
        if isfinite(a.pass_psi)
            a.pass_t = a.pass_t + dt;
            behind = cos(a.pass_dir) * ex + sin(a.pass_dir) * ey < -(p.length / 2 + 1.0);
            if world.ego.v > 0.5 || (behind && c_now > 0.8) || a.pass_t > 10
                a.pass_psi = NaN;
            end
        elseif world.ego.v < 0.2 && fwd > 0 && a.v < 0.05 && c_now < 3.0
            v_c = 0.8;
            need = max(0.4, min(c_now, 1.0) - 0.02);
            if c_now < 0.4, need = c_now; end      % already close: just not closer
            away = -1;
            if lat < 0, away = 1; end
            for turn = [0.3, 0.6, 0.9, 1.2, 1.5]
                for sgn = [away, -away]
                    h = a.psi + sgn * turn;
                    if local_clear_after(a, h, v_c, p, ecx, ecy, erad) >= need
                        a.pass_psi = h;
                        a.pass_dir = a.psi;
                        a.pass_t = 0;
                        break;
                    end
                end
                if isfinite(a.pass_psi), break; end
            end
        end
        if isfinite(a.pass_psi)
            side_psi = a.pass_psi;
            react = 0.8 / max(a.v_des, 0.1);
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

    if isfinite(side_psi)
        psi_des = side_psi;
        avoid = 0;
    end

    % A vehicle stays on the road. Swerving round the ego -- or round
    % anything -- may not carry it off the carriageway: with the ego paused
    % in mid-road, oncoming traffic used to drive onto the verge to get
    % past. If keeping to the road means it cannot get past, it waits
    % (the never-into-the-ego rule above stops it).
    if ~is_crosser && (avoid ~= 0 || isfinite(side_psi)) && a.v > 0.1
        h = psi_des + avoid;
        look = max(a.v, 1.0) * 1.0;
        [~, d1] = sih_cart2frenet(world.rp, a.x + look * cos(h), a.y + look * sin(h));
        [s0a, d0a] = sih_cart2frenet(world.rp, a.x, a.y);
        hw_a = interp1(world.rp.s, world.rp.halfwidth, min(max(s0a, 0), world.rp.length), 'linear');
        lim = hw_a - p.width / 2 + 0.1;
        if abs(d1) > lim && abs(d1) > abs(d0a)
            avoid = 0;
            if isfinite(side_psi), psi_des = a.psi; react = 0; end
        end
    end

    % ---- yaw rate limit, tighter for long vehicles -----------------------
    yaw_max = 2.4 / max(1.0, p.length / 2);
    dpsi    = sih_wrap_pi(psi_des + avoid - a.psi);
    a.psi   = a.psi + min(max(dpsi, -yaw_max*dt), yaw_max*dt);

    % ---- class-dependent erratic heading noise ---------------------------
    if p.erratic > 0
        psi_was = a.psi;
        a.psi = a.psi + p.heading_noise * p.erratic * sqrt(dt) * randn();
        % It may turn, but not into the ego: a 2.2 m cow turning on the spot
        % beside a stopped car swung its body into it. A turn that brings it
        % closer than a safe distance is refused; turning away -- which is
        % how a halted walker eventually goes round a waiting car -- is not.
        [ox, oy] = sih_agent_discs(a.x, a.y, psi_was, p);
        [nx, ny] = sih_agent_discs(a.x, a.y, a.psi, p);
        d_was = min(min(hypot(ox(:) - ecx(:).', oy(:) - ecy(:).')));
        d_now = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).')));
        if d_now < d_was && d_now < erad + p.radius + 0.3
            a.psi = psi_was;
        end
    end
    a.psi = sih_wrap_pi(a.psi);

    % ---- speed tracking with a little variability ------------------------
    v_target = react * a.v_des * (1 + 0.05 * p.erratic * randn());
    a.v = a.v + min(max(v_target - a.v, -p.a_brake*dt), p.a_brake*dt);
    a.v = min(max(a.v, 0), p.v_max);

    % Never into the ego. Whatever the steering above decided, a step that
    % would bring this road user within 0.4 m of the vehicle -- or closer
    % than it already is, if it is nearer than that -- is not taken: going
    % round a stopped car must not end in its side.
    nx = a.x + a.v * cos(a.psi) * dt;
    ny = a.y + a.v * sin(a.psi) * dt;
    if a.v > 0
        c_was = local_clear_at(a.x, a.y, a.psi, p, ecx, ecy, erad);
        c_new = local_clear_at(nx, ny, a.psi, p, ecx, ecy, erad);
        if c_new < c_was && c_new < 0.4
            nx = a.x; ny = a.y; a.v = 0;
        end
    end
    a.x = nx;
    a.y = ny;

    world.agents(k) = a;
end

world.t = t;
end

% -------------------------------------------------------------------------
function c = local_clear_after(a, h, v, p, ecx, ecy, erad)
%LOCAL_CLEAR_AFTER Clearance to the ego after one second at speed v on heading h.
[nx, ny] = sih_agent_discs(a.x + v * cos(h), a.y + v * sin(h), h, p);
c = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).'))) - erad - p.radius;
end

% -------------------------------------------------------------------------
function c = local_clear_at(x, y, psi, p, ecx, ecy, erad)
%LOCAL_CLEAR_AT Clearance to the ego of a road user at (x, y, psi).
[nx, ny] = sih_agent_discs(x, y, psi, p);
c = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).'))) - erad - p.radius;
end
