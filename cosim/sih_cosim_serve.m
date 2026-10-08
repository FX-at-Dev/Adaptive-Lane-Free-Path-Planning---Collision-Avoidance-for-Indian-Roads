function sih_cosim_serve(opts)
%SIH_COSIM_SERVE Serve the autonomy stack to the Unity scene (V2: live closed loop).
%
%   SIH_COSIM_SERVE()        connect to Unity on 127.0.0.1:47600 and serve
%   SIH_COSIM_SERVE(opts)    with options:
%       .host         Unity's address                       ('127.0.0.1')
%       .port         Unity's Live Link port                (47600)
%       .idle_exit_s  quit after this long with no Unity    (900; 0 = never)
%       .silence_s    drop a connection silent this long    (10)
%       .once         serve one connection, then return     (false)
%
%   Unity is the world, the sensors and the ego vehicle's plant; this side
%   is everything that decides: perception, tracking, prediction, risk,
%   behaviour, planning and control, plus the traffic model and the safety
%   measurement on ground truth. Each 50 ms step:
%
%       Unity  --TICK k (ego state, raw sensor data, judge events)-->  Octave
%              perception turns the raw data into detections (V3,
%              core/perception), the stack runs (sih_stack_tick), Octave's
%              own bicycle step gives the reference ego, clearance and goal
%              are measured on truth, and the world advances to step k+1
%       Unity  <--ACT k (command, reference ego, series, frame, where every
%                       road user is for the next sensing)--  Octave
%       Unity integrates its plant with the command, poses the road users
%       where ACT k put them, senses, and sends TICK k+1.
%
%   Each sensor comes from Octave's object-level model (sih_sense), from
%   Unity's raw data, or is off; START chooses. With all three from Octave
%   the run is the offline run, bit for bit.
%
%   The stack and the world are the same functions the offline runner uses
%   (sim/sih_run_scenario.m), so a live run with an exact plant reproduces
%   the offline run. The one input from Unity is the ego state, which is
%   what a real vehicle's state estimate would be.
%
%   Wire format: every message is an 8-byte header -- uint32 JSON length,
%   uint32 binary length, little endian -- then the JSON text, then the
%   binary part. Numbers the loop depends on (ego state, commands) travel
%   in the binary part as float64, because a JSON round trip through
%   jsonencode/jsondecode does not preserve every bit of a double; the
%   JSON carries everything that is only displayed. A TICK's binary part is
%   the ego state (6 float64), then the LiDAR returns (n_lidar x [x y z]),
%   the radar returns (n_radar x [range bearing rate]) and the camera boxes
%   (n_camera x [u0 v0 u1 v1 class score]), all float32.
%
%   Set the environment variable SIH_COSIM_RECORD to a folder before Octave
%   starts and every run is saved there as <scenario>_<seed>.mat: per step
%   the ego, the detections, the LiDAR returns left after the road and map
%   filters, and where every road user really was -- enough to replay
%   perception, tracking and decisions offline (sih_lidar_objects,
%   sih_stack_tick) without Unity.
%
%   Run from the repository root:
%       octave-cli --eval "startup; sih_cosim_serve"
%   or let the Live Link in the Unity scene start it on Play.

if nargin < 1, opts = struct(); end
opts = local_defaults(opts, struct('host', '127.0.0.1', 'port', 47600, ...
    'idle_exit_s', 900, 'silence_s', 10, 'once', false));

if sih_is_octave()
    pkg load instrument-control
end

fprintf('cosim: serving the stack to Unity at %s:%d\n', opts.host, opts.port);
idle = tic;
announced = false;
while true
    t = local_connect(opts);
    if isempty(t)
        if ~announced
            fprintf('cosim: waiting for Unity (press Play in the scene)\n');
            announced = true;
        end
        if opts.idle_exit_s > 0 && toc(idle) > opts.idle_exit_s
            fprintf('cosim: no Unity for %.0f s, exiting\n', opts.idle_exit_s);
            return;
        end
        pause(0.5);
        continue;
    end

    fprintf('cosim: connected\n');
    try
        local_session(t, opts);
        fprintf('cosim: Unity said goodbye\n');
    catch err
        fprintf('cosim: connection ended (%s)\n', err.message);
    end
    clear t;
    idle = tic;
    announced = false;
    if opts.once
        return;
    end
end
end

% =========================================================================
% session
% =========================================================================

function local_session(t, opts)
local_send(t, struct('type', 'ready', 'protocol', 1, ...
    'interpreter', local_interpreter()), []);
sim = [];
while true
    [m, blob] = local_recv(t, opts.silence_s);
    switch m.type
        case 'start'
            try
                [sim, hello, hb] = local_start(m);
                local_send(t, hello, hb);
            catch err
                local_send(t, local_error(err, 'start'), []);
            end
        case 'tick'
            if isempty(sim)
                local_send(t, struct('type', 'error', 'where', 'tick', ...
                    'message', 'tick before start'), []);
                continue;
            end
            try
                [sim, act, ab] = local_tick(sim, m, blob);
                local_send(t, act, ab);
            catch err
                local_send(t, local_error(err, 'tick'), []);
            end
        case 'ping'
            % Keeps the connection alive while Unity is paused.
        case 'bye'
            return;
        otherwise
            local_send(t, struct('type', 'error', 'where', 'protocol', ...
                'message', ['unknown message ' m.type]), []);
    end
end
end

% -------------------------------------------------------------------------
function [sim, hello, blob] = local_start(m)
%LOCAL_START Build the scenario and the stack; HELLO describes the run.
name = m.scenario;
builder = ['sih_scn_' name];
if exist(builder, 'file') ~= 2
    error('sih_cosim:scenario', 'no scenario %s (no %s.m)', name, builder);
end
% A random layout that keeps the scenario's story when Unity asks for one
% (its world seed), the scripted layout otherwise.
if isfield(m, 'layout_seed') && ~isempty(m.layout_seed)
    [scn, cfg] = feval(builder, [], m.layout_seed);
else
    [scn, cfg] = feval(builder);
end
cfg.verbose = false;
% The target the vehicle stops at, when Unity sets one (the Goal flag in the
% scene): its station along the road, in the vehicle's own lane, at least
% 20 m past the start and short of the road's end.
if isfield(m, 'goal') && numel(m.goal) >= 2 && all(isfinite(double(m.goal(1:2))))
    g = double(m.goal(:)).';
    s_start = sih_cart2frenet(scn.rp, scn.ego.x, scn.ego.y);
    s_g = min(max(sih_cart2frenet(scn.rp, g(1), g(2)), s_start + 20), scn.rp.length - 2);
    hw = interp1(scn.rp.s, scn.rp.halfwidth, s_g, 'linear');
    [gx, gy] = sih_frenet2cart(scn.rp, s_g, sih_lane_centre(hw, cfg), 0);
    scn.goal = [gx, gy];
end
think = true;
if isfield(m, 'think'), think = logical(m.think); end
cfg.debug.think = think;     % the planner keeps its candidates only when asked
if isfield(m, 'seed') && ~isempty(m.seed), cfg.sim.seed = m.seed; end

% Where each sensor comes from: 'octave' (sih_sense), 'unity' (raw data,
% core/perception) or 'off'.
names = {'camera', 'radar', 'lidar'};
src = {'octave', 'octave', 'octave'};
if isfield(m, 'sensors')
    for i = 1:3
        if isfield(m.sensors, names{i}), src{i} = m.sensors.(names{i}); end
    end
end
sim.octave_sensors = strcmp(src, 'octave');
sim.unity_sensors  = strcmp(src, 'unity');

world = sih_world_init(scn, cfg);
sim.scn   = scn;
sim.cfg   = cfg;
sim.st    = sih_stack_init(world, cfg, think);
sim.k     = 0;
sim.N     = round(cfg.sim.t_end / cfg.sim.dt);
sim.ctx   = [];
if any(sim.unity_sensors)
    % The static map Unity surveyed: [cx cy length width heading height] rows.
    smap = zeros(0, 6);
    if isfield(m, 'map') && isfield(m.map, 'static') && ~isempty(m.map.static)
        smap = reshape(double(m.map.static(:)), 6, []).';
    end
    sim.ctx = sih_perception_init(scn.rp, cfg, smap);
    fprintf('cosim: static map of %d pieces\n', size(smap, 1));
end
sim.log = local_log_init(sim.N);
% A recording of what the stack was given, for replaying perception and
% tracking offline: set SIH_COSIM_RECORD to a folder before starting Octave.
sim.rec_dir = getenv('SIH_COSIM_RECORD');
sim.rec = {};
sim.percep = struct('points', 0, 'kept', 0, 'clusters', 0, 'objects', 0, 'radar', 0, 'camera', 0);
sim.quality = struct('n', zeros(1, 3), 'true', zeros(1, 3), 'ghost', zeros(1, 3), 'ghost_xy', zeros(0, 3), ...
                     'seen', zeros(1, 3), 'present', zeros(1, 3));

% The world is always one step ahead of the stack: Unity senses it at step
% k before the stack runs step k, so it is advanced to step 1 now and to
% step k+1 at the end of step k. The calls happen in the same order as in
% the offline runner, so the random numbers -- and the run -- are the same.
world = sih_world_step(world, cfg.sim.dt, cfg);
sim.world = world;
sim.reached   = false;
sim.collided  = false;
sim.collide_t = NaN;
sim.t_reached = NaN;
sim.next_agent_id = max([0, world.agents.id]) + 1;
sim.max_plant_err = 0;

% Every class a judge can drop in, as well as those in the scenario, so the
% viewer can size whatever appears.
classes = [{world.agents.class}, {'car', 'bus', 'truck', 'auto', 'two_wheeler', ...
           'bicycle', 'pedestrian', 'cattle', 'pushcart', 'static'}];
hello.type = 'hello';
hello.scenario = name;
hello.run = sih_export_header(scn, cfg, classes, think);
hello.plant = struct('wheelbase', cfg.ego.wheelbase, 'a_max', cfg.veh.a_max, ...
    'a_emergency', cfg.veh.a_emergency, 'delta_max', cfg.veh.delta_max, ...
    'delta_rate', cfg.veh.delta_rate, 'v_reverse_max', cfg.veh.v_reverse_max);
hello.n_steps = sim.N;
hello.layout_seed = -1;
if isfield(scn, 'seed'), hello.layout_seed = scn.seed; end
hello.next_agent_id = sim.next_agent_id;
hello.sensors = struct('camera', src{1}, 'radar', src{2}, 'lidar', src{3});
hello.sensor_spec = local_sensor_spec(cfg);
hello.sense_agents = local_sense_agents(world);
blob = local_ego_vec(world.ego);
fprintf('cosim: start %s (%d steps, seed %d; camera %s, radar %s, lidar %s)\n', ...
        name, sim.N, cfg.sim.seed, src{1}, src{2}, src{3});
end

% -------------------------------------------------------------------------
function [sim, act, blob] = local_tick(sim, m, blob_in)
%LOCAL_TICK One step: judge events, world, stack, reference plant, truth.
t0  = tic;
cfg = sim.cfg;
dt  = cfg.sim.dt;
k   = sim.k + 1;

% ---- the ego state, from Unity's plant -------------------------------------
% Before step 1 Unity holds the HELLO state; after it, whatever its plant
% made of the last command. How far that is from this side's own bicycle
% step is reported back, so a plant that drifts is visible, not hidden.
if numel(blob_in) >= 48
    e = double(typecast(uint8(blob_in(1:48)), 'double'));
    w = sim.world.ego;     % this side's own reference for the same instant
    err = max(abs([w.x - e(1), w.y - e(2), sih_wrap_pi(w.psi - e(3)), ...
                   w.v - e(4), w.delta - e(5), w.a - e(6)]));
    sim.max_plant_err = max(sim.max_plant_err, err);
    sim.world.ego.x = e(1); sim.world.ego.y = e(2); sim.world.ego.psi = e(3);
    sim.world.ego.v = e(4); sim.world.ego.delta = e(5); sim.world.ego.a = e(6);
else
    err = NaN;
end

% ---- judge events ------------------------------------------------------------
spawned = zeros(1, 0);
if isfield(m, 'events') && ~isempty(m.events)
    ev = m.events;
    if ~iscell(ev), ev = num2cell(ev); end
    for i = 1:numel(ev)
        [sim, id] = local_event(sim, ev{i});
        spawned = [spawned, id]; %#ok<AGROW>
    end
end

% ---- perception of Unity's raw sensor data ------------------------------
tp = tic;
if any(sim.unity_sensors)
    off = 48;
    [P, off]  = local_floats(blob_in, off, local_count(m, 'n_lidar'), 3);
    [Rr, off] = local_floats(blob_in, off, local_count(m, 'n_radar'), 3);
    B         = local_floats(blob_in, off, local_count(m, 'n_camera'), 6);
    dets = sih_sense(sim.world, cfg, sim.octave_sensors);
    pi_ = struct('points', 0, 'kept', 0, 'clusters', 0, 'objects', 0);
    kept_pts = zeros(0, 4, 'single');
    if sim.unity_sensors(3)
        [d, pi_] = sih_lidar_detect(P, sim.world.ego, sim.ctx, cfg);
        kept_pts = pi_.kept_pts;
        pi_ = rmfield(pi_, 'kept_pts');       % recorded, never sent
        dets = sih_dets_cat(dets, d);
    end
    nr = 0; nc = 0;
    if sim.unity_sensors(2)
        d = sih_radar_detect(Rr, sim.world.ego, sim.ctx, cfg);
        nr = numel(d.x);
        dets = sih_dets_cat(dets, d);
    end
    if sim.unity_sensors(1)
        d = sih_camera_detect(B, sim.world.ego, sim.ctx, cfg);
        nc = numel(d.x);
        dets = sih_dets_cat(dets, d);
    end
    sim.percep = pi_;
    sim.percep.radar = nr;
    sim.percep.camera = nc;
    sim.quality = local_quality(sim.quality, dets, sim.world, cfg);
    if ~isempty(sim.rec_dir)
        w = sim.world;
        ag = w.agents([w.agents.active]);
        tr = zeros(numel(ag), 7);
        for i = 1:numel(ag)
            tr(i, :) = [ag(i).id, ag(i).x, ag(i).y, ag(i).psi, ag(i).v, ag(i).props.length, ag(i).props.width];
        end
        sim.rec{k} = struct('ego', [w.ego.x, w.ego.y, w.ego.psi, w.ego.v, w.ego.delta, w.ego.a], ...
                            'dets', dets, 'lidar_pts', kept_pts, 'truth', tr, 'truth_class', {{ag.class}});
    end
end
percep_ms = 1000 * toc(tp);

% ---- the same step as the offline runner -----------------------------------
ts = tic;
if any(sim.unity_sensors)
    [sim.st, a_cmd, delta_cmd, tk] = sih_stack_tick(sim.st, sim.world, k, cfg, dets);
else
    [sim.st, a_cmd, delta_cmd, tk] = sih_stack_tick(sim.st, sim.world, k, cfg);
end
stack_ms = 1000 * toc(ts);
sim.world.ego = sih_bicycle_step(sim.world.ego, a_cmd, delta_cmd, dt, cfg, tk.gear);

t = k * dt;
[min_clear, hit, worst] = sih_clearance_truth(sim.world, cfg);
if hit && ~sim.collided
    sim.collided  = true;
    sim.collide_t = t;
    w = sim.world.agents([sim.world.agents.id] == worst);
    if ~isempty(w)
        fprintf('cosim: COLLISION at t=%.1f with %s #%d (ego %.1f m/s, it %.1f m/s)%s', t, w(1).class, worst, ...
                sim.world.ego.v, w(1).v, char(10));
    end
end
if ~sim.reached && sih_goal_reached(sim.world.ego, sim.scn.goal, sim.scn.rp, cfg)
    sim.reached   = true;
    sim.t_reached = t;
end
sim.k = k;
if mod(k, round(5 / dt)) == 0
    % Every 5 s: what the stack is doing and why (the HUD's state panel).
    T = sim.st.tracks;
    trk = '';
    for i = 1:numel(T)
        if strcmp(T(i).status, 'tentative'), continue; end
        trk = [trk, sprintf(' %s#%d(%.0f,%.0f %.1fm/s shape %.1fx%.1f@%.0f)', T(i).class, T(i).id, T(i).x(1), T(i).x(2), ...
               T(i).x(3), T(i).shape(1), T(i).shape(2), T(i).shape(3) * 180 / pi)]; %#ok<AGROW>
    end
    fprintf('cosim: t=%5.1f ego (%.1f,%.1f) %.1f m/s %s: %s | tracks%s%s', t, sim.world.ego.x, ...
            sim.world.ego.y, sim.world.ego.v, sim.st.bp.state, sim.st.bp.reason, trk, char(10));
end
sim.log = local_log_step(sim.log, k, t, sim.world.ego, min_clear, tk, sim.st);

% ---- what Unity shows ---------------------------------------------------------
st = sim.st;
act.type = 'act';
act.k = k;
act.gear = tk.gear;      % +1 forward, -1 reverse (sih_reverse)
act.t = t;
s.t = t;
s.state = st.bp.state;
s.v_cap = st.bp.v_cap;
s.min_clear = local_fin(min_clear);
s.plan_risk = local_fin(st.traj.risk);
s.n_tracks = tk.tdiag.n_confirmed;
s.feasible = double(st.info.feasible);
s.latency_ms = local_fin(tk.latency_ms, -1);
act.series = s;
tf = tic;
if mod(k - 1, 2) == 0
    act.frame = sih_export_frame(sih_snapshot(sim.world, st, tk, t, min_clear));
end
frame_ms = 1000 * toc(tf);
done = sim.reached || k >= sim.N;

% ---- the world advances to the next step, for Unity to sense --------------------
tw = tic;
if ~done
    sim.world = sih_world_step(sim.world, dt, cfg);
end
world_ms = 1000 * toc(tw);
act.sense_agents = local_sense_agents(sim.world);
act.percep = sim.percep;

act.done = double(done);
act.reached = double(sim.reached);
act.collided = double(sim.collided);
act.collide_t = local_fin(sim.collide_t, -1);
act.plant_err = local_fin(err, -1);
act.max_plant_err = sim.max_plant_err;
act.spawned = spawned;
act.timing = [world_ms, percep_ms, stack_ms, frame_ms, 1000 * toc(t0)];
if done
    act.metrics = local_summary(sim);
    if ~isempty(sim.rec_dir)
        rec = sim.rec; scenario = sim.scn.name; layout_seed = -1;
        if isfield(sim.scn, 'seed') && ~isempty(sim.scn.seed), layout_seed = sim.scn.seed; end
        f = fullfile(sim.rec_dir, sprintf('%s_%d.mat', scenario, layout_seed));
        save('-v7', f, 'rec', 'scenario', 'layout_seed');
        fprintf('cosim: recorded %d steps to %s%s', numel(rec), f, char(10));
    end
end

% Command and Octave's own next ego state, exactly.
blob = [typecast([a_cmd, delta_cmd], 'uint8'), local_ego_vec(sim.world.ego)];
end

% -------------------------------------------------------------------------
function [sim, id] = local_event(sim, e)
%LOCAL_EVENT A judge drops a road user into the world.
%   It joins the ground truth like any other agent: the stack has to see it
%   with its sensors, track it, predict it and plan around it.
id = -1;
switch e.kind
    case 'spawn'
        id = sim.next_agent_id;
        sim.next_agent_id = id + 1;
        mode = 'static';
        if isfield(e, 'mode'), mode = e.mode; end
        a = sih_agent_new(id, e.class, e.x, e.y, e.psi, e.v, mode, [], 0);
        sim.world.agents(end+1) = a;
        fprintf('cosim: t=%.1f judge dropped %s #%d (%s)\n', sim.k * sim.cfg.sim.dt, e.class, id, mode);
    case 'remove'
        for i = 1:numel(sim.world.agents)
            if sim.world.agents(i).id == e.id
                sim.world.agents(i).active = false;
                sim.world.agents(i).t_spawn = Inf;   % and do not come back
            end
        end
    otherwise
        error('sih_cosim:event', 'unknown event %s', e.kind);
end
end

% =========================================================================
% wire
% =========================================================================

function t = local_connect(opts)
t = [];
try
    if sih_is_octave()
        t = tcpclient(opts.host, opts.port, 'Timeout', 0.25, 'EnableTransferDelay', false);
    else
        t = tcpclient(opts.host, opts.port, 'Timeout', 0.25, 'ConnectTimeout', 2);
    end
catch
    t = [];
end
end

% -------------------------------------------------------------------------
function local_send(t, msg, blob)
j = uint8(jsonencode(msg));
blob = uint8(blob(:)');
write(t, [typecast(uint32([numel(j), numel(blob)]), 'uint8'), j, blob]);
end

% -------------------------------------------------------------------------
function [msg, blob] = local_recv(t, silence_s)
h = local_read(t, 8, silence_s);
n = double(typecast(h, 'uint32'));
msg = jsondecode(char(local_read(t, n(1), silence_s)));
blob = local_read(t, n(2), silence_s);
end

% -------------------------------------------------------------------------
function b = local_read(t, n, silence_s)
%LOCAL_READ Exactly n bytes; a read times out every 0.25 s, so keep going
%   until they arrive or the peer has been silent too long.
b = zeros(1, 0, 'uint8');
if n == 0, return; end
quiet = tic;
while numel(b) < n
    d = read(t, n - numel(b));
    if isempty(d)
        if toc(quiet) > silence_s
            error('sih_cosim:silent', 'Unity silent for %.0f s', silence_s);
        end
        continue;
    end
    b = [b, uint8(d(:)')]; %#ok<AGROW>
    quiet = tic;
end
end

% =========================================================================
% sensing and logging
% =========================================================================

function a = local_sense_agents(world)
%LOCAL_SENSE_AGENTS Every active road user where Unity should sense it:
%   flat [id x y psi v] per agent, and the class names.
act = [world.agents.active];
ag = world.agents(act);
n = numel(ag);
a.pose = zeros(1, 5 * n);
for i = 1:n
    a.pose(5*i-4:5*i) = [ag(i).id, ag(i).x, ag(i).y, ag(i).psi, ag(i).v];
end
a.class = {ag.class};
end

function q = local_quality(q, dets, world, cfg)
%LOCAL_QUALITY Score detections against the truth (diagnostics only: the
%   stack never sees this). A detection within 2.5 m of an active road user
%   is a true one; anything else is a ghost.
nm = {'camera', 'radar', 'lidar'};
act = [world.agents.active];
ax = [world.agents(act).x];
ay = [world.agents(act).y];
% Recall: road users within each sensor's range and field of view with a
% detection from it within 2.5 m (occluded ones count as missed).
sp = {cfg.sensor.camera, cfg.sensor.radar, cfg.sensor.lidar};
ex = world.ego.x + cos(world.ego.psi) * cfg.ego.rear_axle_to_centre;
ey = world.ego.y + sin(world.ego.psi) * cfg.ego.rear_axle_to_centre;
for si = 1:3
    mine = strcmp(dets.sensor, nm{si});
    dx = dets.x(mine); dy = dets.y(mine);
    for k = 1:numel(ax)
        r = hypot(ax(k) - ex, ay(k) - ey);
        b = sih_wrap_pi(atan2(ay(k) - ey, ax(k) - ex) - world.ego.psi);
        if r > sp{si}.range || abs(b) > sp{si}.fov / 2, continue; end
        q.present(si) = q.present(si) + 1;
        if ~isempty(dx) && min(hypot(dx - ax(k), dy - ay(k))) < 2.5
            q.seen(si) = q.seen(si) + 1;
        end
    end
end
for i = 1:numel(dets.x)
    si = find(strcmp(dets.sensor{i}, nm));
    q.n(si) = q.n(si) + 1;
    if ~isempty(ax) && min(hypot(ax - dets.x(i), ay - dets.y(i))) < 2.5
        q.true(si) = q.true(si) + 1;
    else
        q.ghost(si) = q.ghost(si) + 1;
        q.ghost_xy(end+1, :) = [dets.x(i), dets.y(i), si];
    end
end
end

function s = local_sensor_spec(cfg)
%LOCAL_SENSOR_SPEC What Unity's sensor models need from the stack's config.
L = cfg.sensor.lidar; R = cfg.sensor.radar; C = cfg.sensor.camera;
s.lidar = struct('range', L.range, 'beams', L.beams, 'azimuth_bins', L.azimuth_bins, ...
    'v_fov', L.v_fov, 'mount_height', L.mount_height, 'dropout', L.dropout, ...
    'range_noise', L.range_noise);
s.radar = struct('range', R.range, 'fov', R.fov, 'az_res', R.az_res, 'v_fov', R.v_fov, ...
    'mount_height', R.mount_height, 'sigma_range', R.sigma_range, ...
    'sigma_bear', R.sigma_bear, 'sigma_rate', R.sigma_rate, 'pd', R.pd);
s.camera = struct('range', C.range, 'fov', C.fov, 'image_size', C.image_size, ...
    'mount_height', C.mount_height, 'sigma_px', C.sigma_px, 'pd', C.pd, ...
    'class_acc', C.class_acc, 'min_box_px', C.min_box_px, 'min_visible', C.min_visible);
s.classes = sih_class_names();
s.rear_axle_to_centre = cfg.ego.rear_axle_to_centre;
end

function n = local_count(m, f)
n = 0;
if isfield(m, f), n = double(m.(f)); end
end

function [A, off] = local_floats(b, off, n, w)
%LOCAL_FLOATS n rows of w float32 from byte offset off.
nb = 4 * n * w;
if n <= 0 || off + nb > numel(b)
    A = zeros(0, w, 'single');
    off = off + max(nb, 0);
    return;
end
A = reshape(typecast(uint8(b(off+1:off+nb)), 'single'), w, n).';
off = off + nb;
end

function log = local_log_init(N)
z = zeros(N, 1);
log.t = z; log.x = z; log.y = z; log.psi = z; log.v = z; log.a = z; log.delta = z;
log.min_clear = z; log.latency_ms = z; log.v_cap = z; log.n_confirmed = z;
log.n_det = z; log.n_hypotheses = z; log.feasible = true(N, 1); log.plan_risk = z;
log.state = cell(N, 1);
end

function log = local_log_step(log, k, t, ego, min_clear, tk, st)
log.t(k) = t; log.x(k) = ego.x; log.y(k) = ego.y; log.psi(k) = ego.psi;
log.v(k) = ego.v; log.a(k) = ego.a; log.delta(k) = ego.delta;
log.min_clear(k) = min_clear; log.latency_ms(k) = tk.latency_ms;
log.v_cap(k) = st.bp.v_cap; log.n_confirmed(k) = tk.tdiag.n_confirmed;
log.n_det(k) = tk.tdiag.n_det; log.n_hypotheses(k) = tk.n_hyp;
log.feasible(k) = st.info.feasible; log.plan_risk(k) = st.traj.risk;
log.state{k} = st.bp.state;
end

function s = local_summary(sim)
%LOCAL_SUMMARY The run's metrics, as sih_run_all reports them offline.
n = sim.k;
f = fieldnames(sim.log);
log = sim.log;
for i = 1:numel(f)
    v = log.(f{i});
    log.(f{i}) = v(1:n);
end
r.reached = sim.reached; r.collided = sim.collided;
r.t_reached = sim.t_reached; r.t_end = log.t(end);
m = sih_metrics(r, log, sim.cfg);
s.reached = double(r.reached);
s.collided = double(r.collided);
s.t_end = r.t_end;
s.t_to_goal = local_fin(m.t_to_goal, -1);
s.min_clear = local_fin(m.min_clear, -1);
s.latency_p95_ms = local_fin(m.latency_p95_ms, -1);
s.jerk_lon_rms = local_fin(m.jerk_lon_rms, -1);
s.curv_max = local_fin(m.curv_max, -1);
s.v_mean = local_fin(m.v_mean, -1);
q = sim.quality;
if sum(q.n) > 0
    nm = {'camera', 'radar', 'lidar'};
    for i = 1:3
        if q.n(i) > 0
            fprintf('cosim: %s found %.0f%% of the road users in its range; detections %d, on a road user %.0f%%, ghosts %.0f%%\n', nm{i}, 100 * q.seen(i) / max(q.present(i), 1), q.n(i), ...
                    100 * q.true(i) / q.n(i), 100 * q.ghost(i) / q.n(i));
        end
    end
    if ~isempty(q.ghost_xy)
        % Where the ghosts were, to the metre, most frequent first.
        [u, ~, j] = unique(round(q.ghost_xy), 'rows');
        cnt = accumarray(j, 1);
        [cnt, o] = sort(cnt, 'descend');
        for i = 1:min(12, numel(o))
            fprintf('cosim:   ghost x %4d y %4d sensor %d: %d steps\n', u(o(i), 1), u(o(i), 2), u(o(i), 3), cnt(i));
        end
    end
end
fprintf('cosim: done %s reached %d collided %d t %.1f minclr %.2f lat95 %.1f jerk %.2f curv %.3f vmean %.2f\n', ...
    sim.scn.name, s.reached, s.collided, s.t_end, s.min_clear, s.latency_p95_ms, ...
    s.jerk_lon_rms, s.curv_max, s.v_mean);
end

% =========================================================================
% helpers
% =========================================================================

function v = local_ego_vec(ego)
v = typecast([ego.x, ego.y, ego.psi, ego.v, ego.delta, ego.a], 'uint8');
end

function v = local_fin(v, alt)
%LOCAL_FIN JSON has no Inf or NaN; "nothing" is written as 999 (or alt).
if nargin < 2, alt = 999; end
if isempty(v) || ~isfinite(v)
    v = alt;
end
end

function e = local_error(err, where)
e.type = 'error';
e.where = where;
e.message = err.message;
if ~isempty(err.stack)
    e.at = sprintf('%s line %d', err.stack(1).name, err.stack(1).line);
end
fprintf('cosim: error in %s: %s\n', where, err.message);
end

function s = local_interpreter()
if sih_is_octave()
    s = ['GNU Octave ' OCTAVE_VERSION];
else
    s = ['MATLAB ' version('-release')];
end
end

function o = local_defaults(o, d)
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}) || isempty(o.(f{i}))
        o.(f{i}) = d.(f{i});
    end
end
end
