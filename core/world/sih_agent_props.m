function p = sih_agent_props(class_name)
%SIH_AGENT_PROPS Physical and behavioural properties of a road-user class.
%
%   p = SIH_AGENT_PROPS(class_name) returns a struct describing one of the
%   road-user classes that share Indian roads. Fields:
%       .length, .width   [m] footprint
%       .n_discs          discs used to cover that footprint
%       .radius           [m] covering radius of one disc
%       .v_typ            [m/s] typical cruising speed
%       .v_max            [m/s]
%       .a_brake          [m/s^2] magnitude of comfortable deceleration
%       .lat_agility      [m/s] how fast this class can translate sideways
%       .heading_noise    [rad/s] heading volatility used by the predictor
%       .erratic          0..1 tendency to move non-deterministically
%       .yields           0..1 tendency to give way to the ego vehicle
%
%   lat_agility and erratic are what make the prediction step class-aware. A
%   bus tracks its heading; an auto-rickshaw translates sideways almost as
%   fast as it moves forward; cattle do neither predictably. Collapsing these
%   into one motion model is exactly the assumption that fails on Indian
%   roads, so they are kept explicit and tunable here.

% The table never changes; the tracker asks for it tens of thousands of
% times a run, so each class is built once.
persistent cache
key = lower(class_name);
if ~isempty(cache) && isfield(cache, key)
    p = cache.(key);
    return;
end

switch key
    case 'car'
        p = local_make(4.20, 1.80,  8.0, 16.0, 3.5, 0.7, 0.10, 0.15, 0.60);
    case 'bus'
        p = local_make(11.00, 2.60, 7.0, 14.0, 2.5, 0.4, 0.05, 0.10, 0.30);
    case 'truck'
        p = local_make(8.50, 2.50,  6.5, 13.0, 2.5, 0.4, 0.05, 0.10, 0.30);
    case 'auto'
        % Auto-rickshaw: narrow, highly manoeuvrable, weak lane discipline.
        p = local_make(2.60, 1.40,  6.0, 12.0, 3.0, 1.6, 0.35, 0.60, 0.35);
    case 'two_wheeler'
        p = local_make(1.90, 0.70,  7.0, 18.0, 4.0, 2.2, 0.45, 0.70, 0.25);
    case 'bicycle'
        p = local_make(1.70, 0.60,  3.5,  7.0, 2.5, 1.2, 0.40, 0.45, 0.45);
    case 'pedestrian'
        p = local_make(0.60, 0.60,  1.3,  3.0, 2.0, 1.1, 1.20, 0.80, 0.50);
    case 'cattle'
        % Slow, and essentially unpredictable in direction: the defining
        % feature of the mandated cattle-crossing scenario.
        p = local_make(2.20, 0.90,  1.0,  3.0, 1.5, 0.9, 1.00, 0.95, 0.05);
    case 'pushcart'
        p = local_make(2.00, 1.20,  1.2,  2.5, 1.5, 0.5, 0.30, 0.35, 0.40);
    case 'static'
        % Parked vehicle, debris, pothole barrier: never moves.
        p = local_make(3.00, 1.60,  0.0,  0.0, 0.0, 0.0, 0.00, 0.00, 0.00);
    otherwise
        error('sih_agent_props:unknownClass', ...
              'Unknown road-user class "%s".', class_name);
end
p.class = key;
if isempty(cache), cache = struct(); end
cache.(key) = p;
end

% -------------------------------------------------------------------------
function p = local_make(len, wid, v_typ, v_max, a_brake, lat_agility, ...
                        heading_noise, erratic, yields)
p.length        = len;
p.width         = wid;
p.v_typ         = v_typ;
p.v_max         = v_max;
p.a_brake       = a_brake;
p.lat_agility   = lat_agility;
p.heading_noise = heading_noise;
p.erratic       = erratic;
p.yields        = yields;

% Cover the footprint with discs whose count follows the aspect ratio, so a
% bus is four discs and a pedestrian is one. A single disc per agent would
% inflate an 11 m bus into a 5.6 m radius blob and make most gaps look
% impassable.
p.n_discs = max(1, round(len / max(wid, 0.3)));
p.radius  = sqrt((len / p.n_discs / 2)^2 + (wid / 2)^2);
end
