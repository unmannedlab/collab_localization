nSims = 10^3;
EKF = zeros(3, nSims/10, 10);
% DCL = zeros(3, nSims/10, 10);
% DC2 = zeros(3, nSims/10, 10);
CKF = zeros(3, nSims/10, 10);

tic
for i = 1:10
    out = fmain(nSims/10);
    
    EKF(:,:,i) = out.EKF;
%     DCL(:,:,i) = out.DCL;
%     DC2(:,:,i) = out.DC2;
    CKF(:,:,i) = out.CKF;
    
end
toc

%% 
figure(1); clf;
subplot(1,3,1); hold on; xlabel('X Error'); legend;
% histogram(DCL(1,:,:))
% histogram(DC2(1,:,:))
histogram(EKF(1,:,:), 'DisplayName', 'EKF')
histogram(CKF(1,:,:), 'DisplayName', 'CKF')
hold off;


subplot(1,3,2); hold on; xlabel('Y Error'); legend;
% histogram(DCL(2,:,:))
% histogram(DC2(2,:,:))
histogram(EKF(2,:,:), 'DisplayName', 'EKF')
histogram(CKF(2,:,:), 'DisplayName', 'CKF')
hold off;

subplot(1,3,3); hold on; xlabel('\theta Error'); legend;
% histogram(DCL(3,:,:))
% histogram(DC2(3,:,:))
histogram(EKF(3,:,:), 'DisplayName', 'EKF')
histogram(CKF(3,:,:), 'DisplayName', 'CKF')
hold off;
