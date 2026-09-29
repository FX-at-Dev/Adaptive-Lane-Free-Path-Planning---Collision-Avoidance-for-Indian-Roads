function sih_rng(seed)
%SIH_RNG Seed the random number generators portably.
%   MATLAB has rng(); Octave does not, and uses the legacy rand('state',...)
%   interface instead. Every stochastic component (sensor noise, clutter,
%   agent jitter) draws from these generators, so seeding here makes a whole
%   scenario run bit-reproducible on either interpreter.
if sih_is_octave()
    rand('state', seed);
    randn('state', seed);
else
    rng(seed, 'twister');
end
end
