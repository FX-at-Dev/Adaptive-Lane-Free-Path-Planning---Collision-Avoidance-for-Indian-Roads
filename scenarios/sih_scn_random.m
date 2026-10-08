function [scn, cfg] = sih_scn_random(scn, cfg, seed, draw)
%SIH_SCN_RANDOM A random, valid variant of a scenario.
%
%   [scn, cfg] = SIH_SCN_RANDOM(scn, cfg, seed, draw)
%
%   draw(rp, cfg, R) returns the scenario's story -- its road users for the
%   road rp -- and sih_scn_traffic adds everyday traffic around it
%   (cfg.scn.traffic, default on), both taking their
%   random numbers from R (R.u(a, b) uniform, R.pick({...}) one of a list,
%   R.coin(p) true with probability p, R.int(a, b) an integer). Draws that
%   break the rules (sih_scn_validate) are thrown away and drawn again, up to
%   60 times. The world's own randomness (cfg.sim.seed: erratic motion,
%   sensor noise) is reseeded from the same seed, so one number reproduces
%   the whole run.

if isempty(seed)
    return;
end
cfg.sim.seed = mod(round(seed), 2^31 - 1);
for attempt = 0:59
    sih_rng(cfg.sim.seed + 7919 * attempt);
    R.u = @(a, b) a + (b - a) * rand();
    R.pick = @(c) c{1 + floor(rand() * numel(c) * 0.999999)};
    R.coin = @(p) rand() < p;
    R.int = @(a, b) a + floor(rand() * (b - a + 1) * 0.999999);
    A = draw(scn.rp, cfg, R);
    % Around the story, the everyday traffic of an Indian road.
    if ~isfield(cfg, 'scn') || ~isfield(cfg.scn, 'traffic') || cfg.scn.traffic
        A = [A, sih_scn_traffic(scn, cfg, R, max([0, A.id]) + 1)]; %#ok<AGROW>
    end
    scn.agents = A;
    [ok, why] = sih_scn_validate(scn, cfg);
    if ok
        scn.seed = seed;
        scn.desc = sprintf('%s (random layout, seed %d)', scn.desc, round(seed));
        return;
    end
    last = why;
end
error('sih_scn_random:invalid', 'no valid layout for %s with seed %d (%s)', scn.name, round(seed), last);
end
