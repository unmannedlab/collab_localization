nSims = 10^4;
EKF = zeros(3, nSims/10, 10);
DCL = zeros(3, nSims/10, 10);
DC2 = zeros(3, nSims/10, 10);

tic
parfor i = 1:10
    out = fmain(nSims/10);
    
    EKF(:,:,i) = out.EKF;
    DCL(:,:,i) = out.DCL;
    DC2(:,:,i) = out.DC2;
    
end
toc