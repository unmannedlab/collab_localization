clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^3;
EKF_rmse = zeros(3, nSims/10, 10);
DCL_rmse = zeros(3, nSims/10, 10);
% DC2_rmse = zeros(3, nSims/10, 10);
CKF_rmse = zeros(3, nSims/10, 10);

sensors = [true, true, true, true];

tic
parfor i = 1:10
    out = fmain(nSims/10, sensors);
    
    EKF_rmse(:,:,i) = out.EKF;
    DCL_rmse(:,:,i) = out.DCL;
%     DC2_rmse(:,:,i) = out.DC2;
    CKF_rmse(:,:,i) = out.CKF;
    
end
toc

plt = true;
postplot;