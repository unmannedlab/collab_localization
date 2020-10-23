clf; clear; close all;

Sim = Simulation('Tunnel', true, [1, 0, 1, 1]);
Sim.run_to(20, 0.1);
