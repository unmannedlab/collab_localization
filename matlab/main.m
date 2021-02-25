clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^4;
runtime = 20;
SF = -0;
plt = false;
sve = false;

scs; 
preprocessor;
tic 
for t=1:length(ticks)
    Truth;
%     EKF;
%     DCL;
%     DC2;
    CCL;
    
%     EKF_LMK;
%     DCL_LMK;
end
toc
figure(1);
subplot(1,3,1);
plot(reshape(CCL_x(1,1,:),[nTicks,1,1]))

subplot(1,3,2);
plot(reshape(CCL_x(2,1,:),[nTicks,1,1]))

subplot(1,3,3);
plot(reshape(CCL_x(3,1,:),[nTicks,1,1]))

postprocessor;
 