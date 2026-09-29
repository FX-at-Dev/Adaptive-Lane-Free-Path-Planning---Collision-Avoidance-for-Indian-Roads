function world = sih_world_init(scn, cfg)
%SIH_WORLD_INIT Build the ground-truth world from a scenario specification.
%
%   world = SIH_WORLD_INIT(scn, cfg) takes the scenario struct produced by a
%   scenarios/sih_scn_*.m file and returns the mutable simulation state.
%
%   The scenario struct is the single source of truth shared by the Octave
%   core, the Simulink model and the MATLAB drivingScenario builder, so this
%   function deliberately does no interpretation beyond copying and seeding.

sih_rng(cfg.sim.seed);

world.t      = 0;
world.agents = scn.agents;
world.rp     = scn.rp;
world.goal   = scn.goal;
world.name   = scn.name;

world.ego       = scn.ego;
world.ego.delta = 0;
world.ego.a     = 0;

world.collided = false;
world.reached  = false;
end
