clear accel accel_r F gyro h H K kf_vel mag Q R x0 z z_del z_t z_v z_vel z_x z_y;

EKF_err = EKF_x(1:3,:,:,:) ...
    - repmat(reshape(x_truth(1:3,:,:,:),[3, nCars, 1, nTicks]), [1, 1, nSims, 1]);

std_err = reshape(std(EKF_err,0,[2,4]), [3, nSims]);

figure(1);
histogram(std_err(1,:));

figure(2);
histogram(std_err(2,:));

figure(3);
histogram(std_err(3,:));

fn = [datestr(now,'yyyy_mm_dd-HH_MM-'),in_name,'-',num2str(nSims),'.mat'];
ff = fullfile('output',fn);

% save(ff, 'std_err');