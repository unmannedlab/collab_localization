clear accel accel_r F gyro h H K kf_vel mag Q R x0 z z_del z_t z_v z_vel z_x z_y;

if exist('EKF_x','var')
    EKF_err = EKF_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    EKF_m = reshape(RMSE(EKF_err), [3, nSims]);
end

if exist('DCL_x','var')
    DCL_err = DCL_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DCL_m = reshape(RMSE(DCL_err), [3, nSims]);
end

if exist('DC2_x','var')
    DC2_err = DC2_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DC2_m = reshape(RMSE(DC2_err), [3, nSims]);
end

if exist('CCL_x','var')
    m = zeros(3*nCars,1);
    for i = 1:nCars
        m(i*3-2) = i*6-5;
        m(i*3-1) = i*6-4;
        m(i*3-0) = i*6-3;
    end
    CCL_err = CCL_x(m,:,:) - repmat(reshape(x_truth(1:3,:,:), [3*nCars, 1, nTicks]), [1, nSims, 1]);
    CCL_m = reshape(RMSE(reshape(CCL_err, [3,nCars,nSims,nTicks])), [3, nSims]);
end

if exist('EKF_LMK_x','var')
    EKF_LMK_err = EKF_LMK_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    EKF_LMK_m = reshape(mean(EKF_LMK_err,[2,4]), [3, nSims]);
end

if exist('DCL_LMK_x','var')
    DCL_LMK_err = DCL_LMK_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
    DCL_LMK_m = reshape(mean(DCL_LMK_err,0,[2,4]), [3, nSims]);
end

nBins = 20;

if plt
    figure(1); clf; xlabel('X Error');
    figure(2); clf; xlabel('Y Error');
    figure(3); clf; xlabel('Theta Error');    
    
    if exist('EKF_x','var')
        figure(1); hold on; histogram(EKF_m(1,:)); hold off;
        figure(2); hold on; histogram(EKF_m(2,:)); hold off;
        figure(3); hold on; histogram(EKF_m(3,:)); hold off;
    end

    if exist('DCL_x','var')
        figure(1); hold on; histogram(DCL_m(1,:)); hold off;
        figure(2); hold on; histogram(DCL_m(2,:)); hold off;
        figure(3); hold on; histogram(DCL_m(3,:)); hold off;
    end

    if exist('DC2_x','var')
        figure(1); hold on; histogram(DC2_m(1,:)); hold off;
        figure(2); hold on; histogram(DC2_m(2,:)); hold off;
        figure(3); hold on; histogram(DC2_m(3,:)); hold off;
    end

    if exist('CCL_x','var')
        figure(1); hold on; histogram(CCL_m(1,:)); hold off;
        figure(2); hold on; histogram(CCL_m(2,:)); hold off;
        figure(3); hold on; histogram(CCL_m(3,:)); hold off;
    end

    if exist('EKF_LMK_x','var')
        figure(1); hold on; histogram(EKF_LMK_m(1,:)); hold off;
        figure(2); hold on; histogram(EKF_LMK_m(2,:)); hold off;
        figure(3); hold on; histogram(EKF_LMK_m(3,:)); hold off;
    end

    if exist('DCL_LMK_x','var')
        figure(1); hold on; histogram(DCL_LMK_m(1,:)); hold off;
        figure(2); hold on; histogram(DCL_LMK_m(2,:)); hold off;
        figure(3); hold on; histogram(DCL_LMK_m(3,:)); hold off;
    end

end

if sve 
    fn = [datestr(now,'yyyy_mm_dd-HH_MM-'),in_name,'-',num2str(nSims),'.mat'];
    ff = fullfile('output',fn);

    save(ff, 'std_err');
end