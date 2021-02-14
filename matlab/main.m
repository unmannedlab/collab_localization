addpath('algorithms');
addpath('functions');
addpath('input');
addpath('output');
addpath('scripts');

nSims = 10^4;
runtime = 10;

par; 
preprocessor;
tic 
for t=1:length(ticks)
    Truth;
    EKF;
    EKF_LMK;
end
toc

postprocessor;
