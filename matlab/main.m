clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^3;
runtime = 10;
SF = -0;
sve = false;

sensors = [true, true, true];

par;
preprocessor;

tic 
for t=1:length(ticks)
    Truth;
    EKF;
    CKF;
    DCL;
end
toc

postprocessor;
postplot;
