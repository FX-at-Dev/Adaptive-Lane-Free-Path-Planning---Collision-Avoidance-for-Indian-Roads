function tf = sih_is_octave()
%SIH_IS_OCTAVE True when running under GNU Octave rather than MATLAB.
%   Used to guard the handful of places where the two interpreters differ
%   (random seeding, figure/video export, toolbox probing).
persistent cached
if isempty(cached)
    cached = (exist('OCTAVE_VERSION', 'builtin') ~= 0);
end
tf = cached;
end
