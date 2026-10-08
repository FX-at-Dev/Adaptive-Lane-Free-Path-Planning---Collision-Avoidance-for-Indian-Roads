function test_scenario_random()
%TEST_SCENARIO_RANDOM Random scenario layouts are valid, varied and reproducible.
%
%   For every scenario: the scripted layout passes the rules of the road
%   (sih_scn_validate); 30 random seeds all give a valid layout; the same
%   seed gives the same layout; different seeds give different ones.

names = {'village_road', 'urban_intersection', 'highway_merge', 'market', 'cattle_crossing'};
for k = 1:numel(names)
    f = ['sih_scn_' names{k}];
    [scn, cfg] = feval(f);
    [ok, why] = sih_scn_validate(scn, cfg);
    sih_assert_true(ok, sprintf('%s scripted layout: %s', names{k}, why));

    sig = cell(1, 30);
    for seed = 1:30
        [scn, cfg] = feval(f, [], seed);
        [ok, why] = sih_scn_validate(scn, cfg);
        sih_assert_true(ok, sprintf('%s seed %d: %s', names{k}, seed, why));
        sih_assert_true(cfg.sim.seed == seed, 'the world is seeded from the layout seed');
        sig{seed} = local_signature(scn);
    end
    [scn2, ~] = feval(f, [], 7);
    sih_assert_true(isequal(local_signature(scn2), sig{7}), sprintf('%s: same seed, same layout', names{k}));
    sih_assert_true(numel(unique(sig)) >= 25, sprintf('%s: layouts vary (%d distinct of 30)', names{k}, numel(unique(sig))));
end
end

function s = local_signature(scn)
a = scn.agents;
s = sprintf('%s:%.3f,%.3f;', [{a.class}; num2cell([a.x]); num2cell([a.y])]{:});
end
