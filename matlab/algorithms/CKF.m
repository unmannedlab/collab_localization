if t == 1
    
    x = zeros(6,nCars);
    x(1:3,:) = x_truth(1:3,:,t);
    x(4,:) = -sin(x_truth(3,:,t)).*x_truth(4,:,t);
    x(5,:) =  cos(x_truth(3,:,t)).*x_truth(4,:,t);
    CKF_x = zeros(6*nCars, nSims, nTicks);
    CKF_x(:,:,t) = repmat(reshape(x, [6*nCars,1]), [1, nSims]);
    
    CKF_P = repmat( diag( repmat([...
                imu_acc_err / 2 / rate_imu^2    ;...
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
        
        theta = CKF_x((1:nCars)*3,:,t);
        
        accel_r = [...
            -sin(theta) .* accel(1:nCars,:) - cos(theta).*accel(nCars+1:nCars*2,:) ;...
             cos(theta) .* accel(1:nCars,:) - sin(theta).*accel(nCars+1:nCars*2,:) ];
        
        CKF_x(:,:,t) = zeros(6*nCars, nSims, 1);
        for i =1:nCars
            m = (i-1)*6 + 1;
            n = (i-1)*2 + 1;

            CKF_x(m:m+5,:,t) = [...
                CKF_x(m+0,:,t-1) + CKF_x(m+3,:,t-1) * dt + accel_r(n+0,:) / 2 * dt^2; ...
                CKF_x(m+1,:,t-1) + CKF_x(m+4,:,t-1) * dt + accel_r(n+1,:) / 2 * dt^2; ...
               (CKF_x(m+2,:,t-1) + gyro(i,:) * dt) * 0.98 + 0.02 * mag(i,:);...
                CKF_x(m+3,:,t-1) + accel_r(n+0,:) * dt;...
                CKF_x(m+4,:,t-1) + accel_r(n+1,:) * dt;...
                gyro(i,:)];
        end; clear m n;
        
        Q = diag(repmat([  ...
            imu_acc_err / 2 / rate_imu^2    ;...
            imu_acc_err / 2 / rate_imu^2    ;...
            imu_gyr_err / rate_imu          ;...
            imu_acc_err / rate_imu          ;...
            imu_acc_err / rate_imu          ;...
            imu_gyr_err                     ], [nCars,1]));

        Fs = [  1,  0,  0, dt,  0,  0   ;...
                0,  1,  0,  0, dt,  0   ;...
                0,  0,  1,  0,  0, dt   ;...
                0,  0,  0,  1,  0,  0   ;...
                0,  0,  0,  0,  1,  0   ;...
                0,  0,  0,  0,  0,  1];
        Fc = repmat({Fs},1,nCars);
        F = blkdiag(Fc{:});
        
        CKF_P = pagemtimes(pagemtimes(F, CKF_P),'none', F, 'transpose') + Q;
        
        clear accel accel_r gyro mag theta Q F Fs Fc
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 && sensors(1)

        z_vel = x_truth(4,:,t)' ...
            + normrnd(0, enc_err, [nCars, nSims]);
        z_del = del(t,:)' ...
            + normrnd(0, imu_mag_err, [nCars, nSims]);
        
        z = [   z_vel;...
                z_vel.*tan(z_del) / wb ];

        kf_vel = sqrt( CKF_x((1:nCars)*6-2,:,t).^2 + CKF_x((1:nCars)*6-1,:,t).^2 );
        
        H = zeros(2*nCars, 6*nCars, nSims);
        for i = 1:nCars
            m = (i-1)*6;
            H(i, m+4, :) = CKF_x(m+4,:,t) ./ kf_vel(i,:);
            H(i, m+5, :) = CKF_x(m+5,:,t) ./ kf_vel(i,:);
            H(i+nCars, m+6, :) = 1;
        end
        
        h = [   kf_vel; ...
                CKF_x((1:nCars)*6,:,t)];

        
        R = zeros(2*nCars, 2*nCars, nSims);
        for i = 1:nSims
            R(:,:,i) = diag([repmat(enc_err, [nCars, 1]); ...
                             str_err .* z_vel(:,1) / wb]);
        end 
        

        K = pageDiv(pagemtimes(CKF_P,'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,CKF_P),'none',H,'transpose') + R));

        CKF_x(:,:,t) = CKF_x(:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [nCars*2, 1, nSims])), [6*nCars, nSims]);
       
        CKF_P = pagemtimes((repmat(eye(6*nCars), [1,1,nSims]) - pagemtimes(K,H))  , CKF_P);

        clear z_vel z_del z kf_vel H h R K
    end

    % GPS step at 10 Hz 
    if mod(t, rate/rate_gps) == 0 && sensors(2)
        
        z_x = x_truth(1,:,t)' + normrnd(0, gps_per, [nCars, nSims]);
        z_y = x_truth(2,:,t)' + normrnd(0, gps_per, [nCars, nSims]);
        z_t = x_truth(3,:,t)' + normrnd(0, gps_her, [nCars, nSims]);
        z_v = x_truth(4,:,t)' + normrnd(0, gps_ver, [nCars, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt( CKF_x((1:nCars)*6-2,:,t).^2 +...
                       CKF_x((1:nCars)*6-1,:,t).^2 );

        H = zeros(4*nCars, 6*nCars, nSims);
        for i = 1:nCars
            m = (i-1)*6;
            H(i+nCars*0, m+1, :) = 1;
            H(i+nCars*1, m+2, :) = 1;
            H(i+nCars*2, m+3, :) = 1;
            H(i+nCars*3, m+4, :) = CKF_x(m+4,:,t) ./ kf_vel(i,:);
            H(i+nCars*3, m+5, :) = CKF_x(m+5,:,t) ./ kf_vel(i,:);
        end
        
        h = zeros(4*nCars,nSims);
        for i = 1:nCars
            h(i+nCars*0, :) = CKF_x((i-1)*6+1, :, t);
            h(i+nCars*1, :) = CKF_x((i-1)*6+2, :, t);
            h(i+nCars*2, :) = CKF_x((i-1)*6+3, :, t);
        end
        h(3*nCars+1:4*nCars,:) = kf_vel;

        R = diag([  ...
            repmat(gps_per, [nCars,1]);...
            repmat(gps_per, [nCars,1]);...
            repmat(gps_her, [nCars,1]);...
            repmat(gps_ver, [nCars,1])]);

        K = pageDiv(pagemtimes(CKF_P,'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,CKF_P),'none',H,'transpose') + R));

        CKF_x(:,:,t) = CKF_x(:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [nCars*4, 1, nSims])), [6*nCars, nSims]);
       
        CKF_P = pagemtimes((repmat(eye(6*nCars), [1,1,nSims]) - pagemtimes(K,H))  , CKF_P);

        clear z_x z_y z_t z_v z kf_vel H h R K
    end

    % UWB Update Step
    if mod(t, rate/rate_uwb) == 0 && nCars > 1 && sensors(3)
        B = bcombs(nCars);
        
        z_x = B * x_truth(1,:,t)';
        z_y = B * x_truth(2,:,t)';
        z = sqrt(z_x.^2 + z_y.^2) + normrnd(0, uwb_err, [size(B,1), nSims]);
        z = max(z,0);
        
        h_x = B * CKF_x((1:nCars)*6-5,:,t);
        h_y = B * CKF_x((1:nCars)*6-4,:,t);
        h = sqrt(h_x.^2 + h_y.^2);
        
        H = zeros(size(B,1), 6*nCars, nSims);        
        H(:,(1:nCars)*6-5,:) = repmat(B, [1,1,nSims]) .* repmat(reshape(h_x ./ h, [size(B,1), 1, nSims]), [1, nCars, 1]);
        H(:,(1:nCars)*6-4,:) = repmat(B, [1,1,nSims]) .* repmat(reshape(h_y ./ h, [size(B,1), 1, nSims]), [1, nCars, 1]);

        R = eye(size(B,1))*uwb_err;
        
        K = pageDiv( pagemtimes( CKF_P,'none',H,'transpose'), ( pagemtimes( pagemtimes( H,CKF_P), 'none', H, 'transpose') + R ) );

        CKF_x(:,:,t) = CKF_x(:,:,t) + reshape(pagemtimes(K,reshape(z - h, [size(B,1), 1, nSims])), [6*nCars, nSims]);
        CKF_P = pagemtimes((repmat(eye(6*nCars), [1,1,nSims]) - pagemtimes(K,H)), CKF_P);

        clear z_x z_y z H h R K
    end
end


function [t] = bcombs(nCars)
    persistent t_prev n_prev
    if isempty(t_prev) || n_prev ~= nCars
        t = zeros(nchoosek(nCars, 2), nCars);
        m = 1;
        n = 2;
        for i = 1:size(t,1)
            t(i,m) =  1;
            t(i,n) = -1;
            n = n + 1;
            if n > size(t,2)
                m = m + 1;
                n = m + 1;
            end
        end
        t_prev = t;
        n_prev = n;
    else    
        t = t_prev;
    end
end