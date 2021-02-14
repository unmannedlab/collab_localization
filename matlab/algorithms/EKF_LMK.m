if t == 1
    EKF_LMK_x = zeros(6, nCars, nSims, nTicks);
    EKF_LMK_P = zeros(6, 6, nCars, nSims);
    
    EKF_LMK_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    
    EKF_LMK_x(4,:,:,t) = ...
        repmat(-sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    EKF_LMK_x(5,:,:,t) = ...
        repmat( cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    
    EKF_LMK_P(:,:,:,:) = repmat(... 
            diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu        ;...
                    imu_acc_err / rate_imu        ;...
                    imu_acc_err / rate_imu        ;...
                    imu_gyr_err]), [1,1,nCars,nSims]);
                
else
    
    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) == 0
        
        accel = [acc(t,:); x_truth(4,:,t).^2 / wb.* tan(del(t,:))] ...
            + normrnd(0, imu_acc_err, [2, nCars, nSims]);
        
        gyro = x_truth(4,:,t) .* tan(del(t,:)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars, nSims]);
        
        mag = x_truth(3,:,t) ...
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        theta = EKF_LMK_x(3,:,:,t);

        accel_r = [...
            -sin(theta) .* accel(1,:,:) - cos(theta).*accel(2,:,:) ;...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ];

        EKF_LMK_x(:,:,:,t) = [...
            EKF_LMK_x(1,:,:,t-1) + EKF_LMK_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            EKF_LMK_x(2,:,:,t-1) + EKF_LMK_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (EKF_LMK_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            EKF_LMK_x(4,:,:,t-1) + accel_r(1) * dt;...
            EKF_LMK_x(5,:,:,t-1) + accel_r(2) * dt;...
            gyro];


        Q = diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu        ;...
                    imu_acc_err / rate_imu        ;...
                    imu_acc_err / rate_imu        ;...
                    imu_gyr_err]);

        F = [   1, 0, 0, dt, 0, 0;...
                0, 1, 0, 0, dt, 0;...
                0, 0, 1, 0, 0, dt;...
                0, 0, 0, 1, 0, 0;...
                0, 0, 0, 0, 1, 0;...
                0, 0, 0, 0, 0, 1];
        
        EKF_LMK_P(:,:,:,:) = pagemtimes(pagemtimes(F, EKF_LMK_P(:,:,:,:)), F) + Q;
        
        clear accel accel_r gyro mag theta Q F
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0

        z_vel = x_truth(4,:,t) ...
            + normrnd(0, enc_err, [1, nCars, nSims]);
        z_del = del(t,:) ...
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        z = [   z_vel;...
                z_vel.*tan(z_del) / wb ];

        kf_vel = sqrt( EKF_LMK_x(4,:,:,t).^2 + EKF_LMK_x(5,:,:,t).^2 );
        
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = EKF_LMK_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = EKF_LMK_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        
        h = [kf_vel; EKF_LMK_x(6,:,:,t)];

        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err;
        R(2,2,:,:) = str_err * z_vel / wb;

        K = pagediv(pagemtimes(EKF_LMK_P(:,:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,EKF_LMK_P(:,:,:,:)),'none',H,'transpose') + R));

        EKF_LMK_x(:,:,:,t) = EKF_LMK_x(:,:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [2, 1, nCars, nSims])), [6, nCars, nSims]);
        
        EKF_LMK_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1,nCars,nSims]) - pagemtimes(K,H))  , EKF_LMK_P(:,:,:,:));

        clear z_vel z_del z kf_vel H h R K
    end

    % GPS step at 10 Hz
    if mod(t, rate/rate_gps) == 0

        z_x = x_truth(1,:,t) ...
            + normrnd(0, gps_per, [1, nCars, nSims]);
        z_y = x_truth(2,:,t) ...
            + normrnd(0, gps_per, [1, nCars, nSims]);
        z_t = x_truth(3,:,t) ...
            + normrnd(0, gps_her, [1, nCars, nSims]);
        z_v = x_truth(4,:,t) ...
            + normrnd(0, gps_ver, [1, nCars, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt( EKF_LMK_x(4,:,:,t).^2 + EKF_LMK_x(5,:,:,t).^2 );
        
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = EKF_LMK_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = EKF_LMK_x(5,:,:,t) ./ kf_vel;
        
        h = [   EKF_LMK_x(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per;...
            gps_per;...
            gps_her;...
            gps_ver]), [1, 1, nCars, nSims]);

        K = pagediv(pagemtimes(EKF_LMK_P(:,:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,EKF_LMK_P(:,:,:,:)),'none',H,'transpose') + R));

        EKF_LMK_x(:,:,:,t) = EKF_LMK_x(:,:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [4, 1, nCars, nSims])), [6, nCars, nSims]);
        
        EKF_LMK_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1,nCars,nSims]) - pagemtimes(K,H))  , EKF_LMK_P(:,:,:,:));

        clear z_x z_y z_t z_v z kf_vel H h R K
    end
    
    % UWB Update Step
    if mod(t, rate/rate_uwb) == 0 && false
        
        z_x = repmat(x_truth(1,:,t),[nLmk,1,nSims]) - lmks(:,1);
        z_y = repmat(x_truth(2,:,t),[nLmk,1,nSims]) - lmks(:,2);

        z = sqrt(z_x.^2 + z_y.^2) ...
            + normrnd(0, gps_per, [nLmk, nCars, nSims]);
        
        z = max(z,0);
        
        h_x = repmat(EKF_LMK_x(1,:,:,t),[nLmk,1,1]) - lmks(:,1);
        h_y = repmat(EKF_LMK_x(2,:,:,t),[nLmk,1,1]) - lmks(:,2);
        
        h = sqrt(h_x.^2 + h_y.^2) ...
            + normrnd(0, gps_per, [nLmk, nCars, nSims]);
        
        H = zeros(nLmk,6,nCars,nSims);
        H(:,1,:,:) = h_x ./ sqrt(h);
        H(:,2,:,:) = h_y ./ sqrt(h);
        
        R = repmat(eye(nLmk)*uwb_err, [1, 1, nCars, nSims]);
        
        K = pagediv(pagemtimes(EKF_LMK_P(:,:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,EKF_LMK_P(:,:,:,:)),'none',H,'transpose') + R));

        EKF_LMK_x(:,:,:,t) = EKF_LMK_x(:,:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [nLmk, 1, nCars, nSims])), [6, nCars, nSims]);
        
        EKF_LMK_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1, nCars, nSims]) - pagemtimes(K,H))  , EKF_LMK_P(:,:,:,:));

        clear z_x z_y z H h R K
    end
end
