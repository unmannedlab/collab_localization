clf; clear; close all;

Sim = Simulation('Tunnel',true, [1, 1, 1]);

Sim.run_to(60, 0.1);
