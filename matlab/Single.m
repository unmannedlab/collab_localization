clf; clear; close all;

Sim = Simulation('Street_Cross',true, [1, 1, 1, 1]);

Sim.run_to(20, 0.1);

disp(max(Sim.err_dcl{1}(:,1)));