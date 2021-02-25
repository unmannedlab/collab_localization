if t == 1
    CCL_x = zeros(6*nCars, nSims, nTicks);
    CCL_P = zeros(6*nCars, 6*nCars, nSims);
    
    x = zeros(6,nCars);
    x(1:3,:) = x_truth(1:3,:,t);
    x(4,:) = -sin(x_truth(3,:,t)).*x_truth(4,:,t);
    x(5,:) =  cos(x_truth(3,:,t)).*x_truth(4,:,t);
    
    CCL_x(:,:,t) = repmat(reshape(x, [6*nCars,1]), [1, nSims]);
    
    CCL_P(:,:,:,t) = repmat(... 
            diag(...
            repmat([imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err], [nCars,1])), [1,1,nSims]);
                
else
    
    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) == 0
        
        accel = [acc(t,:)'; x_truth(4,:,t)'.^2 / wb.* tan(del(t,:))'] ...
            + normrnd(0, imu_acc_err, [2*nCars, nSims]);
        
        gyro = x_truth(4,:,t)' .* tan(del(t,:))' / wb ...
            + normrnd(0, imu_gyr_err, [nCars, nSims]);
        
        mag = x_truth(3,:,t)' ...
            + normrnd(0, imu_mag_err, [nCars, nSims]);
        
        theta = CCL_x([1:nCars]*3,:,t);
        
        accel_r = [...
            -sin(theta) .* accel(1,:,:) - cos(theta).*accel(2,:,:) ;...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ];

        for i =1:nCars
            m = (i-1)*6 + 1;
            n = (i-1)*2 + 1;

            CCL_x(m:m+5,:,t) = [...
                CCL_x(m+0,:,t-1) + CCL_x(m+3,:,t-1) * dt + accel_r(n+0,:) / 2 * dt^2; ...
                CCL_x(m+1,:,t-1) + CCL_x(m+4,:,t-1) * dt + accel_r(n+1,:) / 2 * dt^2; ...
               (CCL_x(m+2,:,t-1) + gyro(i,:) * dt) * 0.98 + 0.02 * mag(i,:);...
                CCL_x(m+3,:,t-1) + accel_r(n+0,:) * dt;...
                CCL_x(m+4,:,t-1) + accel_r(n+1,:) * dt;...
                gyro(i,:)];
        end; clear m n;
        
        Q = diag(...
                repmat(...
                   [imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err], [nCars,1]));


        Fs = [  1, 0, 0, dt, 0, 0;...
                0, 1, 0, 0, dt, 0;...
                0, 0, 1, 0, 0, dt;...
                0, 0, 0, 1, 0, 0;...
                0, 0, 0, 0, 1, 0;...
                0, 0, 0, 0, 0, 1];
        Fc = repmat({Fs},1,nCars);
        F = blkdiag(Fc{:});
        
        CCL_P(:,:,:) = pagemtimes(pagemtimes(F, CCL_P(:,:,:)),'none', F, 'transpose') + Q;
        
        clear accel accel_r gyro mag theta Q F Fs Fc
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 

        z_vel = x_truth(4,:,t)' ...
            + normrnd(0, enc_err, [nCars, nSims]);
        z_del = del(t,:)' ...
            + normrnd(0, imu_mag_err, [nCars, nSims]);
        
        z = [   z_vel;...
                z_vel.*tan(z_del) / wb ];

        kf_vel = sqrt( CCL_x([1:nCars]*6-2,:,t).^2 + CCL_x([1:nCars]*6-1,:,t).^2 );
        
        H = zeros(2*nCars,6*nCars,nSims);
        for i = 1:nCars
            m = (i-1)*6;
            H(i, m+4, :) = CCL_x(m+4,:,t) ./ kf_vel(i,:);
            H(i, m+5, :) = CCL_x(m+5,:,t) ./ kf_vel(i,:);
            H(i+nCars, m+6, :) = 1;
        end
        
        h = [   kf_vel; ...
                CCL_x([1:nCars]*6,:,t)];

        
        R = zeros(2*nCars, 2*nCars, nSims);
        for i = 1:nSims
            R = diag([repmat(enc_err, [nCars, 1]); ...
                      str_err .* z_vel(:,1) / wb]);
        end 
        

        K = pagediv(pagemtimes(CCL_P(:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,CCL_P(:,:,:)),'none',H,'transpose') + R));

        CCL_x(:,:,t) = CCL_x(:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [nCars*2, 1, nSims])), [6*nCars, nSims]);
       
        CCL_P(:,:,:) = pagemtimes((repmat(eye(6*nCars), [1,1,nSims]) - pagemtimes(K,H))  , CCL_P(:,:,:));

        clear z_vel z_del z kf_vel H h R K
    end

    % GPS step at 10 Hz 
    if mod(t, rate/rate_gps) == 0

        z_x = x_truth(1,:,t)' ...
            + normrnd(0, gps_per, [nCars, nSims]);
        z_y = x_truth(2,:,t)' ...
            + normrnd(0, gps_per, [nCars, nSims]);
        z_t = x_truth(3,:,t)' ...
            + normrnd(0, gps_her, [nCars, nSims]);
        z_v = x_truth(4,:,t)' ...
            + normrnd(0, gps_ver, [nCars, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt(  CCL_x([1:nCars]*6-2,:,t).^2 + ...
                        CCL_x([1:nCars]*6-1,:,t).^2 );

        H = zeros(4*nCars,6*nCars,nSims);
        for i = 1:nCars
            m = (i-1)*6;
            H(i+nCars*0, m+1, :) = 1;
            H(i+nCars*1, m+2, :) = 1;
            H(i+nCars*2, m+3, :) = 1;
            H(i+nCars*3, m+5, :) = CCL_x(m+4,:,t) ./ kf_vel(i,:);
            H(i+nCars*3, m+5, :) = CCL_x(m+5,:,t) ./ kf_vel(i,:);
        end
        
        h = zeros(4*nCars,nSims);
        for i = 1:nCars
            h(i+nCars*0, :) = CCL_x((i-1)*6+1, :, t);
            h(i+nCars*1, :) = CCL_x((i-1)*6+2, :, t);
            h(i+nCars*2, :) = CCL_x((i-1)*6+3, :, t);
        end
        h(13:16,:) = kf_vel;
            
        R = diag([  ...
            repmat(gps_per, [nCars,1]);...
            repmat(gps_per, [nCars,1]);...
            repmat(gps_her, [nCars,1]);...
            repmat(gps_ver, [nCars,1])]);

        K = pagediv(pagemtimes(CCL_P(:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,CCL_P(:,:,:)),'none',H,'transpose') + R));

        CCL_x(:,:,t) = CCL_x(:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [nCars*4, 1, nSims])), [6*nCars, nSims]);
     
        CCL_P(:,:,:) = pagemtimes((repmat(eye(6*nCars), [1,1,nSims]) - pagemtimes(K,H))  , CCL_P(:,:,:));

        
        
        clear z_x z_y z_t z_v z kf_vel H h R K
    end
end
