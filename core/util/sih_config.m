function cfg = sih_config()
%SIH_CONFIG Central tunable configuration for the SIH 26037 planning stack.
%
%   Every magic number in the system lives here so that scenarios can override
%   individual fields without editing algorithm code. Scenario files receive the
%   struct and mutate the fields they care about (e.g. a market scenario lowers
%   cfg.plan.v_max and raises cfg.plan.w_clearance).
%
%   Octave-compatible: plain struct, no classdef, no string arrays.

% ---------------------------------------------------------------- simulation
cfg.sim.dt          = 0.05;    % [s] integration step (20 Hz)
cfg.sim.t_end       = 40.0;    % [s] hard stop for a scenario run
cfg.sim.replan_dt   = 0.10;    % [s] planner cycle (10 Hz)
cfg.scn.traffic     = true;    % random layouts add everyday traffic (sih_scn_traffic)
cfg.sim.seed        = 20260909;

% ------------------------------------------------------------- ego geometry
cfg.ego.length      = 4.00;    % [m]
cfg.ego.width       = 1.80;    % [m]
cfg.ego.wheelbase   = 2.70;    % [m]
cfg.ego.rear_axle_to_centre = 1.35;  % [m] geometric centre ahead of rear axle

% Ego footprint is approximated by discs for fast collision checking. The
% covering radius is derived from the box dimensions inside sih_ego_discs
% (half-diagonal of an L/n by W slice), so only the disc count is tunable here.
% Three discs over a 4.0 x 1.8 m box gives a radius of about 1.12 m.
cfg.ego.n_discs     = 3;

% ------------------------------------------------------- vehicle constraints
cfg.veh.a_max       =  2.50;   % [m/s^2] comfortable acceleration
cfg.veh.a_min       = -4.50;   % [m/s^2] comfortable braking
cfg.veh.a_emergency = -7.00;   % [m/s^2] emergency braking
cfg.veh.jerk_max       =  4.00;   % [m/s^3] comfort limit on command rate
cfg.veh.jerk_emergency = 25.00;   % [m/s^3] allowed while building emergency braking
cfg.veh.delta_max   =  0.55;   % [rad] steering angle limit (~31 deg)
cfg.veh.delta_rate  =  0.60;   % [rad/s] steering rate limit
cfg.veh.v_reverse_max = 1.5;   % [m/s] fastest it backs up
cfg.veh.a_lat_max   =  3.00;   % [m/s^2] lateral acceleration comfort limit

% ------------------------------------------------------------------ control
cfg.ctrl.lookahead_min  = 3.0;   % [m] pure-pursuit lookahead floor
cfg.ctrl.lookahead_gain = 0.9;   % [s] lookahead grows with speed
% Where on the planned speed profile the controller takes its setpoint. This
% must be on the order of the speed loop's own time constant, not the sample
% time: the planner's quartic starts and ends at zero acceleration, so its
% first fraction of a second is nearly flat, and sampling there from a
% standstill asks for almost no acceleration and the vehicle never pulls away.
% Fraction of the trajectory's horizon at which the controller samples the
% planned speed. Expressed as a fraction rather than an absolute time because
% scenarios use horizons from 2 s to 5 s, and a fixed time reads a different
% part of the profile in each -- at the long end, the almost-flat beginning.
cfg.ctrl.speed_lookahead = 0.28; % [-] fraction of traj.T
cfg.ctrl.kp_speed       = 1.6;
cfg.ctrl.ki_speed       = 0.20;
cfg.ctrl.i_clamp        = 2.0;

% ------------------------------------------------------------------ sensors
% Each sensor is mounted at the ego geometric centre facing forward (yaw 0),
% except the LiDAR which is omnidirectional.
cfg.sensor.camera.range        = 60.0;   % [m]
cfg.sensor.camera.fov          = deg2rad(100);
cfg.sensor.camera.sigma_range  = 0.08;   % fraction of true range (poor depth)
cfg.sensor.camera.sigma_bear   = deg2rad(0.5);
cfg.sensor.camera.pd           = 0.92;   % detection probability at close range
cfg.sensor.camera.classifies   = true;
cfg.sensor.camera.class_acc    = 0.88;   % probability the class label is right

cfg.sensor.radar.range         = 90.0;
cfg.sensor.radar.fov           = deg2rad(60);
cfg.sensor.radar.sigma_range   = 0.25;   % [m] absolute, good depth
cfg.sensor.radar.sigma_bear    = deg2rad(2.0);   % poor angular resolution
cfg.sensor.radar.sigma_rate    = 0.10;   % [m/s] range-rate
cfg.sensor.radar.pd            = 0.90;
cfg.sensor.radar.classifies    = false;

cfg.sensor.lidar.range         = 40.0;
cfg.sensor.lidar.fov           = 2*pi;   % omnidirectional
cfg.sensor.lidar.sigma_pos     = 0.06;   % [m] both axes
cfg.sensor.lidar.pd            = 0.95;
cfg.sensor.lidar.classifies    = false;
% Scan pattern, used by the Unity LiDAR (sih_sense works at object level and
% ignores these). Roof mount, 32 channels, horizontal resolution ~0.35 deg.
cfg.sensor.lidar.beams         = 32;
cfg.sensor.lidar.azimuth_bins  = 1024;
cfg.sensor.lidar.v_fov         = deg2rad([-15, 15]);
cfg.sensor.lidar.mount_height  = 1.90;   % [m] above ground

cfg.sensor.clutter_rate        = 0.6;    % expected false alarms per scan
cfg.sensor.occlusion           = true;

% Raw-sensor models in the Unity co-simulation (V3). sih_sense works at object
% level and ignores these; Unity casts the rays and renders the image, and
% core/perception turns what comes back into detections. Every sensor sits at
% the ego's geometric centre, facing forward, at these heights.
cfg.sensor.lidar.dropout       = 0.05;   % fraction of returns lost
cfg.sensor.lidar.range_noise   = 0.02;   % [m] 1-sigma range noise per return
cfg.sensor.radar.mount_height  = 0.60;   % [m]
cfg.sensor.radar.az_res        = deg2rad(1.0);        % beam spacing
cfg.sensor.radar.v_fov         = deg2rad([-3, 3]);    % three rows
cfg.sensor.camera.mount_height = 1.40;   % [m]
cfg.sensor.camera.image_size   = [1280, 720];         % [px] width, height
cfg.sensor.camera.sigma_px     = 1.5;    % [px] box-edge noise of the detector
cfg.sensor.camera.min_box_px   = 8;      % [px] smaller boxes are not reported
cfg.sensor.camera.min_visible  = 0.2;    % fraction of an object that must be unoccluded

% ---------------------------------------------------------------- perception
% Raw Unity sensor data -> detections (core/perception, V3). LiDAR returns are
% kept inside the drivable corridor and a margin; further out, only clusters
% the size of a person or an animal survive, which removes the walls, trees
% and houses that line the road without a map of them.
cfg.percep.ground_height   = 0.25;   % [m] returns below are ground
cfg.percep.max_height      = 4.00;   % [m] returns above are overhead (wires, canopy)
cfg.percep.roi_near        = 1.2;    % [m] beyond the corridor edge: keep every cluster
cfg.percep.roi_wide        = 10.0;   % [m] beyond it, only small clusters ...
cfg.percep.wide_max_height = 2.2;    % [m] ... no taller than this
cfg.percep.wide_max_extent = 3.0;    % [m] ... and no longer than this
cfg.percep.roi_cell        = 0.5;    % [m] resolution of the corridor mask
cfg.percep.cell            = 0.3;    % [m] BEV grid for LiDAR clustering
cfg.percep.join            = 0.3;    % [m] returns this close belong to one object (more swallowed a
                                     %     cow stepping out beside a parked truck into the truck)
cfg.percep.min_points      = 4;      % returns needed for a LiDAR object
cfg.percep.fit_angles      = 30;     % orientations tried by the box fit (a quarter turn)
cfg.percep.fit_d0          = 0.05;   % [m] closeness floor of the box fit (about the range noise)
cfg.percep.fit_road_bias   = 0.15;   % preference for a box aligned with the road, on a near tie
cfg.percep.max_extent      = 14.0;   % [m] longer clusters are structure, not road users
% The static map: footprints of walls, houses, trees and poles, known in
% advance as a surveyed map would have them. Returns on them are background.
cfg.percep.map_cell        = 0.25;   % [m] resolution of the static-structure mask
cfg.percep.map_margin      = 0.3;    % [m] grown around each footprint (LiDAR)
cfg.percep.radar_map_margin = 2.5;   % [m] wider for radar, whose bearing is coarse ...
cfg.percep.radar_static_speed = 0.5; % [m/s] ... and applied only to returns this still
cfg.percep.camera_edge_px  = 3;      % [px] a box this close to the image edge is cut off

% -------------------------------------------------------------------- fusion
cfg.fuse.gate_chi2   = 9.21;   % 2-DOF chi-square at 99%
cfg.fuse.confirm_M   = 3;      % confirm after M hits ...
cfg.fuse.confirm_N   = 4;      % ... within the last N scans
cfg.fuse.max_misses  = 6;      % delete a coasting track after this many misses
cfg.fuse.max_misses_still = 20; % ... a still one after this many (1 s)
cfg.fuse.tentative_max_age = 20; % [scans] a track never corroborated in 1 s is dropped
cfg.fuse.q_accel     = 1.5;    % [m/s^2] process noise on longitudinal accel
cfg.fuse.q_yawrate   = 0.6;    % [rad/s^2] process noise on yaw acceleration
cfg.fuse.p0_pos      = 4.0;
cfg.fuse.p0_vel      = 9.0;
cfg.fuse.p0_yaw      = 1.0;
cfg.fuse.p0_omega    = 0.5;

% ---------------------------------------------------------------- prediction
cfg.pred.horizon     = 4.0;    % [s]
cfg.pred.dt          = 0.20;   % [s] prediction step
cfg.pred.n_hypo      = 4;      % hypotheses per track (class dependent)
cfg.pred.sigma_grow  = 0.35;   % [m/s] positional uncertainty growth rate

% Risk field: ego-centric occupancy raster consumed by the planner.
cfg.risk.res         = 0.50;   % [m] cell size
cfg.risk.ahead       = 50.0;   % [m] extent ahead of ego
cfg.risk.behind      = 10.0;   % [m]
cfg.risk.lateral     = 20.0;   % [m] each side
cfg.risk.inflate     = 0.40;   % [m] extra inflation on agent footprints

% ------------------------------------------------------------------ planning
cfg.plan.v_max       = 8.0;    % [m/s] scenario speed cap (village default)
cfg.plan.v_min       = 0.0;
% Lattice size is the direct latency lever: candidates = |horizon_T| x
% |offsets| x v_samples, and each costs a collision check against every
% predicted hypothesis. The offset count now follows the road width at a fixed
% 0.55 m resolution, which is what matters for threading past a pushcart --
% far more than extra speed samples do.
cfg.plan.horizon_T   = [2.5 4.0];            % [s] candidate terminal times
cfg.plan.d_step      = 0.55;                 % [m] lateral offset resolution
cfg.plan.v_samples   = 4;                    % speed samples per terminal time
cfg.plan.corridor_halfwidth = 3.0;           % [m] drivable corridor default
cfg.plan.safety_margin      = 0.45;          % [m] added to every clearance test

% Maximum |dd/ds|: how steeply a candidate may cut across the reference path.
% 0.8 is about 39 degrees. This is a non-holonomic feasibility bound, not a
% comfort one -- it exists to reject pure sideways translations, which are
% straight lines in Cartesian space and so pass a curvature test despite being
% impossible for a car to drive.
cfg.plan.max_path_slope     = 0.80;

% Trajectory cost weights. Tuned so that risk dominates, comfort matters, and
% progress breaks ties between equally safe candidates.
cfg.plan.w_risk      = 60.0;
cfg.plan.w_jerk      = 0.8;
cfg.plan.w_curv      = 12.0;
cfg.plan.w_progress  = 6.0;
cfg.plan.w_offset    = 3.0;    % prefer staying near the lane centre (sih_lane_centre)
cfg.plan.w_clearance = 10.0;   % reward distance from nearest agent
cfg.plan.w_speed_dev = 2.0;

% Lanes. India drives on the left: on a road at least two_lane_width wide
% the vehicle keeps to the centre of the left lane; a narrower road is one
% lane, driven down the middle.
cfg.plan.keep_left      = true;
cfg.plan.two_lane_width = 5.0;   % [m] narrower than this the road is one lane
cfg.plan.road_overhang  = 0.3;   % [m] how far the vehicle's side may stray past the road edge

cfg.plan.risk_threshold = 0.35;  % candidate rejected above this peak risk

% Risk is discounted by how far into the horizon the conflict lies. A conflict
% 4 s away will be replanned against roughly forty more times before it can
% happen, and the prediction that far out is mostly uncertainty growth; a
% conflict 0.5 s away is nearly committed. Without this discount, prediction
% cones for erratic classes span the whole carriageway at the far end of the
% horizon, no candidate at any speed clears the tolerance, and the planner
% reports infeasible continuously.
cfg.plan.risk_tau = 2.0;         % [s] risk discount time constant

% Hybrid A* fallback, used only when no lattice candidate is feasible. It
% composes a sequence of small steering actions, so it can express the
% several-metre shuffle out of a tight spot that no single polynomial can.
% DISABLED BY DEFAULT. The search is implemented and verified to work in
% isolation -- given a boxed-in state it finds escape paths of around five
% metres, and accepted roughly a third of the ones it returned. But enabling it
% end to end made results WORSE, not better: p95 planner latency rose from
% about 80 ms to between 250 and 400 ms, the urban intersection stopped
% completing, and planning to a thinner margin produced a contact on the
% village road that had previously been clean. Turning it on is a one-line
% change once the cost of a search cycle is brought down and its margin
% handling is reconciled with the lattice's; shipping it enabled in this state
% would trade two working scenarios for none.
cfg.plan.hybrid.enable      = false;
cfg.plan.hybrid.grid_res    = 0.60;  % [m] closed-set and occupancy resolution
cfg.plan.hybrid.slice_dt    = 0.60;  % [s] occupancy grid time slice
cfg.plan.hybrid.yaw_bins    = 24;
cfg.plan.hybrid.step        = 1.20;  % [m] motion primitive arc length
cfg.plan.hybrid.steer_set   = [-0.50 -0.28 -0.12 0 0.12 0.28 0.50];  % [rad]
cfg.plan.hybrid.max_expand  = 1400;  % bounded: this runs inside a 10 Hz loop
cfg.plan.hybrid.max_entry_speed = 2.5;  % [m/s] only invoked at low speed
cfg.plan.hybrid.goal_tol_xy = 2.5;   % [m]
cfg.plan.hybrid.lookahead   = 11.0;  % [m] how far up the corridor to aim
cfg.plan.hybrid.max_time    = 9.0;   % [s] search depth, past the horizon
cfg.plan.hybrid.speed       = 1.60;  % [m/s] the fallback creeps
cfg.plan.hybrid.min_weight  = 0.08;  % ignore very unlikely hypotheses
cfg.plan.hybrid.margin_scale = 0.80; % fraction of the safety margin it plans to
cfg.plan.hybrid.accept_clear = -0.12;% [m] acceptable inflated-footprint overlap

% Cost shaping. The heuristic weight above one makes the search greedy, which
% trades optimality for the speed this has to run at; the result only has to be
% drivable and safe, not minimal.
cfg.plan.hybrid.w_steer     = 0.30;
cfg.plan.hybrid.w_change    = 1.20;
cfg.plan.hybrid.w_offset    = 0.20;
cfg.plan.hybrid.w_heur      = 1.35;

% ------------------------------------------------------------------ decision
cfg.dec.ttc_caution    = 4.0;   % [s] enter CAUTION below this TTC
cfg.dec.ttc_emergency  = 1.8;   % [s] emergency braking below this TTC
cfg.dec.clear_caution  = 1.6;   % [m] lateral clearance triggering CAUTION
cfg.dec.clear_stop     = 0.7;   % [m]
cfg.dec.standoff       = 5.0;   % [m] stop this far behind something standing in the path ...
cfg.dec.pullout_speed  = 4.0;   % [m/s] ... and may plan pulling out round it at up to this
% Backing out of a box (sih_reverse).
cfg.dec.reverse_after    = 3.0;   % [s] stood still with no way forward this long ...
cfg.dec.reverse_clear    = 1.5;   % [m] ... with something this close
cfg.dec.reverse_dist     = 3.0;   % [m] backs up this far
cfg.dec.reverse_speed    = 0.8;   % [m/s] at this speed
cfg.dec.reverse_steer    = 0.35;  % [rad] most the wheel is turned while backing
cfg.dec.reverse_angle    = 0.20;  % [rad] heading it backs towards: the road's, angled to the free side
cfg.dec.reverse_cooldown = 6.0;   % [s] before it may back up again
cfg.dec.caution_decel  = 1.5;   % [m/s^2] gentle braking for an object not yet confirmed ...
cfg.dec.caution_margin = 2.0;   % [m] ... aimed this far short of it
cfg.dec.creep_speed    = 1.8;   % [m/s] speed cap while creeping
cfg.dec.caution_scale  = 0.55;  % speed cap multiplier in CAUTION
cfg.dec.yield_gap      = 3.5;   % [s] accepted time gap at an unsignalled cross
cfg.dec.goal_tol       = 3.0;   % [m] along the road: stopped this near the target is there
cfg.dec.arrive_v       = 0.3;   % [m/s] ... and this slow
cfg.dec.arrive_decel   = 0.8;   % [m/s^2] braking for the target (sih_arrive)
cfg.dec.arrive_lead    = 2.5;   % [m] aimed this far short: the planner reaches a speed over its horizon
cfg.dec.arrive_tol     = 0.3;   % [m] this near (or past): hold still

% Unstick: how long the vehicle may sit stationary before it starts edging
% forward, how much clearance it needs to be allowed to, and how far the risk
% tolerance is relaxed while it does. 25 cycles at 10 Hz is 2.5 seconds.
cfg.dec.stuck_cycles     = 18;
cfg.dec.stuck_clear      = 1.0;   % [m]
cfg.dec.stuck_risk_scale = 1.8;   % multiplier on cfg.plan.risk_threshold

% ------------------------------------------------------------- object shape
% A track carries the object's shape -- length, width and the orientation of
% its long axis -- separately from its motion. The footprint every decision
% uses (sih_footprint) is turned by the motion heading only while the object
% is actually moving; a parked truck's heading is unobservable noise, and
% turning its 8.5 m footprint by it swept the box across the road.
cfg.track.v_still        = 0.5;   % [m/s] slower than this ...
cfg.track.t_still        = 1.0;   % [s]   ... for this long: stationary
cfg.track.v_moving       = 1.0;   % [m/s] faster than this (and mature): moving
cfg.track.v_unstill      = 0.7;   % [m/s] faster than this ...
cfg.track.t_move         = 0.5;   % [s]   ... for this long: a still object has moved off
cfg.track.two_face_width = 0.5;   % [m] a box thinner than this shows one face only
cfg.track.extent_decay   = 0.98;  % per update, towards what is seen now
cfg.track.theta_gain     = 0.3;   % smoothing of the long-axis orientation
cfg.track.theta_still_gain = 0.3; % ... scaled by this while the object is still
cfg.track.shape_settled  = 3;     % box updates after which the axis is established
cfg.track.ghost_weight   = 0;     % planner weight of an object not yet established: none, it only slows the car
cfg.track.establish_hits = 12;    % scans tracked (0.6 s) before an object can stop the vehicle ...
cfg.track.lidar_recent   = 10;    % ... with the LiDAR seeing it in the last 0.5 s, within its reach
cfg.track.lidar_trust    = 0.85;  % within this share of the LiDAR's range only it can confirm an object
cfg.track.piece_frac     = 0.5;   % a box shorter than this share of the object is a piece of it
cfg.track.class_size_cap = 1.6;   % a classified object's extents stay under this many times its class's
cfg.track.max_discs      = 12;    % footprint discs along the long axis

% ------------------------------------------------------------------- metrics
cfg.metric.latency_budget = 100.0;  % [ms] per replan cycle
cfg.metric.jerk_limit     = 4.0;    % [m/s^3]
cfg.metric.curv_limit     = 0.30;   % [1/m]
cfg.metric.min_clearance  = 0.30;   % [m] below this counts as a near miss

% -------------------------------------------------------------------- debug
% Observation-only instrumentation for the 3D visualiser. With think on, the
% planner keeps a sample of the candidates it evaluated and the runner records
% detections, predictions and the decision context each cycle. Nothing here
% feeds back into a decision: switching it on must leave every metric except
% wall-clock latency unchanged, and the regression check holds it to that.
cfg.debug.think          = false;
cfg.debug.max_candidates = 60;    % kept per cycle: best feasible + a rejected sample
cfg.debug.cand_points    = 9;     % samples kept along each candidate

% ---------------------------------------------------------------------- misc
cfg.verbose = true;
end
