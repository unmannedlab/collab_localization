clear accel accel_r F gyro h H K kf_vel mag Q R x0 z z_del z_t z_v z_vel z_x z_y;

EKF_err = EKF_x(1:3,:,:,:) ...
    - repmat(reshape(x_truth(1:3,:,:,:),[3, nCars, 1, nTicks]), [1, 1, nSims, 1]);

EKF_LMK_err = EKF_LMK_x(1:3,:,:,:) ...
    - repmat(reshape(x_truth(1:3,:,:,:),[3, nCars, 1, nTicks]), [1, 1, nSims, 1]);

EKF_std = reshape(std(EKF_err,0,[2,4]), [3, nSims]);
EKF_LMK_std = reshape(std(EKF_LMK_err,0,[2,4]), [3, nSims]);

figure(1); clf; hold on; 
histogram(EKF_std(1,:));
histogram(EKF_LMK_std(1,:));
xlabel('X Error');
hold off;

figure(2); clf; hold on;
histogram(EKF_std(2,:));
histogram(EKF_LMK_std(2,:));
xlabel('Y Error');
hold off;

figure(3); clf; hold on;
histogram(EKF_std(3,:));
histogram(EKF_LMK_std(3,:));
xlabel('Theta Error');
hold off; 

fn = [datestr(now,'yyyy_mm_dd-HH_MM-'),in_name,'-',num2str(nSims),'.mat'];
ff = fullfile('output',fn);

% save(ff, 'std_err');