nSims = 10^2;
EKF = zeros(3, nSims/10, 10);
DCL = zeros(3, nSims/10, 10);
DC2 = zeros(3, nSims/10, 10);
CCL = zeros(3, nSims/10, 10);

tic
for i = 1:10
    out = fmain(nSims/10);
    
    EKF(:,:,i) = out.EKF;
    DCL(:,:,i) = out.DCL;
    DC2(:,:,i) = out.DC2;
    CCL(:,:,i) = out.CCL;
    
end
toc

%% 
figure(1); clf; hold on;
histogram(DCL(1,:,:))
histogram(DC2(1,:,:))
histogram(EKF(1,:,:))
histogram(CCL(1,:,:))
hold off;


figure(2); clf; hold on;
histogram(DCL(2,:,:))
histogram(DC2(2,:,:))
histogram(EKF(2,:,:))
histogram(CCL(2,:,:))
hold off;

figure(3); clf; hold on;
histogram(DCL(3,:,:))
histogram(DC2(3,:,:))
histogram(EKF(3,:,:))
histogram(CCL(3,:,:))
hold off;
