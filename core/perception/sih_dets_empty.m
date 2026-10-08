function d = sih_dets_empty()
%SIH_DETS_EMPTY A detection set with no detections, in the form sih_sense returns.
%
%   Per detection: x, y (position), R (2x2 covariance), class ('' if the
%   sensor cannot classify), sensor, truth_id (diagnostics only), and the
%   object's extent where the sensor measures one:
%       box       [L W theta] of what was seen (NaN when no box)
%       box_full  true when that box is the whole object, false when it is
%                 only the faces turned towards the sensor
%       origin    [x y] of the sensor that saw it (NaN if unknown)
%       road_psi  direction of the road at the detection (NaN if unknown)
%       surface   true when the position is the near surface of the object
%                 (a radar's range), not its centre
d.x = zeros(0,1); d.y = zeros(0,1); d.R = zeros(2,2,0);
d.class = cell(0,1); d.sensor = cell(0,1); d.truth_id = zeros(0,1);
d.box = zeros(0,3); d.box_full = false(0,1); d.origin = zeros(0,2); d.road_psi = zeros(0,1);
d.surface = false(0,1);
end
