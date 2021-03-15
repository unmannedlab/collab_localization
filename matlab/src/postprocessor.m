
EKF_diff = EKF_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
EKF_err  = [sqrt(EKF_diff(1,:,:,:).^2 + EKF_diff(2,:,:,:).^2); EKF_diff(3,:,:,:,:)];
EKF_rmse = reshape(RMSE(EKF_err), [2, nSims]); 

DCL_diff = DCL_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
DCL_err  = [sqrt(DCL_diff(1,:,:,:).^2 + DCL_diff(2,:,:,:).^2); DCL_diff(3,:,:,:,:)];
DCL_rmse = reshape(RMSE(DCL_err), [2, nSims]); 

m = zeros(3*nCars,1);
for i = 1:nCars
    m(i*3-2) = i*6-5;
    m(i*3-1) = i*6-4;
    m(i*3-0) = i*6-3;
end
CKF_diff = CKF_x(m,:,:) - repmat(reshape(x_truth(1:3,:,:), [3*nCars, 1, nTicks]), [1, nSims, 1]);
CKF_diff = reshape(CKF_diff, [3, nCars, nSims, nTicks]);
CKF_err  = [sqrt(CKF_diff(1,:,:,:).^2 + CKF_diff(2,:,:,:).^2); CKF_diff(3,:,:,:,:)];
CKF_rmse = reshape(RMSE(reshape(CKF_err, [2,nCars,nSims,nTicks])), [2, nSims]);

clear EKF_diff EKF_err;
clear DCL_diff DCL_err;
clear CKF_diff CKF_err;

function err = RMSE(x)
    m = size(x,1);
    n = size(x,3);
    
    N = size(x,2) * size(x,4);
    err = zeros(m, n);
    
    for i = 1:m
        for j = 1:n
            err(i,j) = sqrt( sum(x(i,:,j,:).^2,[2,4]) / N);
        end
    end
end
