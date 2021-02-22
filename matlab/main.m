% clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^2;
runtime = 20;
SF = -0;
plt = false;
sve = false;

par; 
preprocessor;
tic 
for t=1:length(ticks)
    Truth;
    EKF;
    DCL;
    DC2;
    
%     EKF_LMK;
%     DCL_LMK;
end
toc

postprocessor;
