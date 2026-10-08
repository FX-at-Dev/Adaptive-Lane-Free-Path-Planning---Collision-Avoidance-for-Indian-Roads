function data = sih_export_header(scn, cfg, classes, think)
%SIH_EXPORT_HEADER The meta and road sections of a run file.
%
%   data = SIH_EXPORT_HEADER(scn, cfg, classes, think)
%
%   classes lists the road-user classes whose footprint the viewer needs;
%   think says whether the frames will carry the stack's beliefs. Shared by
%   sih_export_run (a finished run) and the co-simulation's HELLO (a run that
%   is about to happen), so a live run and a replay describe the world in
%   exactly the same terms.

data.meta.name       = scn.name;
data.meta.desc       = scn.desc;
data.meta.dt         = cfg.sim.dt;
data.meta.replan_dt  = cfg.sim.replan_dt;
data.meta.t_end      = cfg.sim.t_end;
data.meta.goal       = scn.goal;
data.meta.v_max      = cfg.plan.v_max;
data.meta.ego_length = cfg.ego.length;
data.meta.ego_width  = cfg.ego.width;
data.meta.latency_budget = cfg.metric.latency_budget;
data.meta.rear_axle_to_centre = cfg.ego.rear_axle_to_centre;
data.meta.wheelbase  = cfg.ego.wheelbase;
% Cross streets [station half-width] per row: scenery keeps them clear.
if isfield(scn, 'cross_streets')
    data.meta.cross_streets = reshape(scn.cross_streets.', 1, []);
else
    data.meta.cross_streets = zeros(1, 0);
end

% Footprint of every class present, so a viewer sizes agents from the same
% table the simulation used rather than from its own copy.
classes = unique(classes);
data.meta.class_size = struct();
for k = 1:numel(classes)
    pr = sih_agent_props(classes{k});
    data.meta.class_size.(classes{k}) = [pr.length, pr.width];
end

data.meta.think = double(think);
if think
    data.meta.sensors.camera = local_sensor(cfg.sensor.camera);
    data.meta.sensors.radar  = local_sensor(cfg.sensor.radar);
    data.meta.sensors.lidar  = local_sensor(cfg.sensor.lidar);
    data.meta.sensors.lidar.beams        = cfg.sensor.lidar.beams;
    data.meta.sensors.lidar.azimuth_bins = cfg.sensor.lidar.azimuth_bins;
    data.meta.sensors.lidar.v_fov        = cfg.sensor.lidar.v_fov;
    data.meta.sensors.lidar.mount_height = cfg.sensor.lidar.mount_height;
    data.meta.pred_mode_names = {'keep', 'brake', 'left', 'right'};
    data.meta.cand_codes = {'ok', 'risk', 'conflict', 'slope', 'curvature', ...
                            'speed', 'lateral_accel'};
end

% ---- road geometry, decimated ------------------------------------------
% The reference path is sampled every 0.25 m, which is far finer than any
% renderer needs and would dominate the file size.
step = max(1, round(1.0 / scn.rp.ds));
idx  = 1:step:numel(scn.rp.s);
data.road.x  = scn.rp.x(idx).';
data.road.y  = scn.rp.y(idx).';
data.road.hw = scn.rp.halfwidth(idx).';
data.road.psi = scn.rp.psi(idx).';
end

% -------------------------------------------------------------------------
function o = local_sensor(sp)
o.range = sp.range;
o.fov   = sp.fov;
o.pd    = sp.pd;
end
