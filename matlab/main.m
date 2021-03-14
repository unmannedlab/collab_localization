clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^4;
runtime = 10;
SF = -0;
sve = false;

sensors = [true, true, true];

scs;
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
