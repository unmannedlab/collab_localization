clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^4;
runtime = 10;
SF = -0;
sve = false;

sensors = [true, true, true, true];

par;
preprocessor;

tic 
for t=1:length(ticks)
    Truth;
%     EKF;
    DCL;
%     DC2;
    CKF;
    
%     EKF_LMK;
%     DCL_LMK;
end
toc

postprocessor;
% postplot;
