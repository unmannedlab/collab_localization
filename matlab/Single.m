clf; clear; close all;

Sim = Simulation('Tunnel', false, [1, 0, 1, 1]);
Sim.run_to(60, 0.1);
