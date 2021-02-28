% clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^3;
runtime = 5;
SF = -0;
plt = true;
sve = false;

lone; 
preprocessor;
tic 
for t=1:length(ticks)
    Truth;
    EKF;
%     DCL;
%     DC2;
    CKF;
    
%     EKF_LMK;
%     DCL_LMK;
end
toc

postprocessor;
 