addpath('input');
addpath('output');
addpath('scripts');

nSimulations = 10;
runtime = 30; 

par; 
geometry;
preprocessor;

for t=1:length(ticks)
    Truth;
    EKF;
    

end
clear t;