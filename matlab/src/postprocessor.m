
if exist('EKF_x','var')
    EKF_err = EKF_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    EKF_rmse = reshape(RMSE(EKF_err), [3, nSims]); clear EKF_err;
end

if exist('DCL_x','var')
    DCL_err = DCL_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DCL_rmse = reshape(RMSE(DCL_err), [3, nSims]); clear DCL_err;
end

if exist('DC2_x','var')
    DC2_err = DC2_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DC2_rmse = reshape(RMSE(DC2_err), [3, nSims]); clear DC2_err;
end

if exist('CKF_x','var')
    m = zeros(3*nCars,1);
    for i = 1:nCars
        m(i*3-2) = i*6-5;
        m(i*3-1) = i*6-4;
        m(i*3-0) = i*6-3;
    end
    CKF_err = CKF_x(m,:,:) - repmat(reshape(x_truth(1:3,:,:), [3*nCars, 1, nTicks]), [1, nSims, 1]);
    CKF_rmse = reshape(RMSE(reshape(CKF_err, [3,nCars,nSims,nTicks])), [3, nSims]);
end

if exist('EKF_LMK_x','var')
    EKF_LMK_err = EKF_LMK_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    EKF_LMK_rmse = reshape(mean(EKF_LMK_err,[2,4]), [3, nSims]);
end

if exist('DCL_LMK_x','var')
    DCL_LMK_err = DCL_LMK_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DCL_LMK_rmse = reshape(mean(DCL_LMK_err,0,[2,4]), [3, nSims]);
end




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
