if t == 1
    DCL_x = zeros(6, nCars, nSims, nTicks);
    DCL_P = zeros(6, 6, nCars, nSims);
    
    DCL_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    
    DCL_x(4,:,:,t) = ...
        repmat(-sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    DCL_x(5,:,:,t) = ...
        repmat( cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    
    DCL_P(:,:,:,:) = repmat(... 
            diag([  imu_acc_err / 2 / rate_imu^2;...
                    imu_acc_err / 2 / rate_imu^2;...
                    imu_gyr_err / rate_imu      ;...
                    imu_acc_err / rate_imu      ;...
                    imu_acc_err / rate_imu      ;...
                    imu_gyr_err]), [1,1,nCars,nSims]);
            

    DCL_p = zeros(6, 6*nCars, nCars, nSims);

else
    
    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) == 0
        
        accel = [acc(t,:); x_truth(4,:,t).^2 / wb.* tan(del(t,:))] ...
            + normrnd(0, imu_acc_err, [2, nCars, nSims]);
        
        gyro = x_truth(4,:,t) .* tan(del(t,:)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars, nSims]);
        
        mag = x_truth(3,:,t) ...
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        theta = DCL_x(3,:,:,t);

        accel_r = [...
            -sin(theta) .* accel(1,:,:) - cos(theta).*accel(2,:,:) ;...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ];

        DCL_x(:,:,:,t) = [...
            DCL_x(1,:,:,t-1) + DCL_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            DCL_x(2,:,:,t-1) + DCL_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (DCL_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            DCL_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            DCL_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro];
%         DCL_x(3,:,:,t) = mod(DCL_x(3,:,:,t),2*pi);

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
        
        DCL_P(:,:,:,:) = pagemtimes(pagemtimes(F, DCL_P(:,:,:,:)), F) + Q;
        
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

        kf_vel = sqrt( DCL_x(4,:,:,t).^2 + DCL_x(5,:,:,t).^2 );
        
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = DCL_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = DCL_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        
        h = [kf_vel; DCL_x(6,:,:,t)];

        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err;
        R(2,2,:,:) = str_err * z_vel / wb;

        K = pagediv(pagemtimes(DCL_P(:,:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,DCL_P(:,:,:,:)),'none',H,'transpose') + R));

        DCL_x(:,:,:,t) = DCL_x(:,:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [2, 1, nCars, nSims])), [6, nCars, nSims]);
%         DCL_x(3,:,:,t) = mod(DCL_x(3,:,:,t),2*pi);
        DCL_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1,nCars,nSims]) - pagemtimes(K,H))  , DCL_P(:,:,:,:));

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

        kf_vel = sqrt( DCL_x(4,:,:,t).^2 + DCL_x(5,:,:,t).^2 );
        
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = DCL_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = DCL_x(5,:,:,t) ./ kf_vel;
        
        h = [   DCL_x(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per;...
            gps_per;...
            gps_her;...
            gps_ver]), [1, 1, nCars, nSims]);

        K = pagediv(pagemtimes(DCL_P(:,:,:,:),'none',H,'transpose'), ...
            (pagemtimes(pagemtimes(H,DCL_P(:,:,:,:)),'none',H,'transpose') + R));

        DCL_x(:,:,:,t) = DCL_x(:,:,:,t) + ...
            reshape(pagemtimes(K,reshape(z - h, [4, 1, nCars, nSims])), [6, nCars, nSims]);
%         DCL_x(3,:,:,t) = mod(DCL_x(3,:,:,t),2*pi);
        DCL_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1,nCars,nSims]) - pagemtimes(K,H))  , DCL_P(:,:,:,:));

        clear z_x z_y z_t z_v z kf_vel H h R K
    end
    
    
    % UWB Update Step
%     if mod(t, rate/rate_uwb) == 0
%         
%         z_x = repmat(x_truth(1,:,t),[nLmk,1,nSims]) - lmks(:,1);
%         z_y = repmat(x_truth(2,:,t),[nLmk,1,nSims]) - lmks(:,2);
% 
%         z = sqrt(z_x.^2 + z_y.^2) ...
%             + normrnd(0, uwb_err, [nLmk, nCars, nSims]);
%         
%         z = max(z,0);
%         
%         h_x = repmat(EKF_LMK_x(1,:,:,t),[nLmk,1,1]) - lmks(:,1);
%         h_y = repmat(EKF_LMK_x(2,:,:,t),[nLmk,1,1]) - lmks(:,2);
%         
%         h = sqrt(h_x.^2 + h_y.^2);
%         
%         H = zeros(nLmk,6,nCars,nSims);
%         H(:,1,:,:) = h_x ./ sqrt(h);
%         H(:,2,:,:) = h_y ./ sqrt(h);
%         
%         R = repmat(eye(nLmk)*uwb_err, [1, 1, nCars, nSims]);
%         
%         K = pagediv(pagemtimes(EKF_LMK_P(:,:,:,:),'none',H,'transpose'), ...
%             (pagemtimes(pagemtimes(H,EKF_LMK_P(:,:,:,:)),'none',H,'transpose') + R));
% 
%         EKF_LMK_x(:,:,:,t) = EKF_LMK_x(:,:,:,t) + ...
%             reshape(pagemtimes(K,reshape(z - h, [nLmk, 1, nCars, nSims])), [6, nCars, nSims]);
%         
%         EKF_LMK_P(:,:,:,:) = pagemtimes((repmat(eye(6), [1,1, nCars, nSims]) - pagemtimes(K,H))  , EKF_LMK_P(:,:,:,:));
% 
%         clear z_x z_y z H h R K
%     end
        
%     n = (obj.car_num-1)*6 + 1;
%     
%     [dcl_dist, ~] = obj.sense_uwb();
%         
%     for j = obj.setdiff2(length(dcl_dist(:,1))/2, obj.car_num, 0)
%         m = (j-1)*6+1;
%         
%         x_i = obj.dcl_x;
%         Sigma_ii = obj.dcl_Sigma(n:n+5,n:n+5);
%         sigma_ij = obj.dcl_sigma(m:m+5,:);
%         
%         x_j = obj.uwb.dcl_states(:,j);
%         Sigma_jj = obj.uwb.dcl_Sigma(:,:,j);
%         sigma_ji = obj.uwb.dcl_sigma(n:n+5,:,j);
%         
%         if all(~isnan(x_j)) && all(all(~isinf(dcl_dist(j*2-1:j*2, :))))
%         
%             Ri = [  cos(x_i(3)), -sin(x_i(3));...
%                     sin(x_i(3)),  cos(x_i(3))];
% 
%             Rj = [  cos(x_j(3)), -sin(x_j(3));...
%                     sin(x_j(3)),  cos(x_j(3))];
% 
%             off_i = Ri * obj.tag_offsets(:,1:2)';
%             off_j = Rj * obj.tag_offsets(:,1:2)';
% 
%             X_i = off_i(1,:)+x_i(1);
%             Y_i = off_i(2,:)+x_i(2);
% 
%             X_j = off_j(1,:)+x_j(1);
%             Y_j = off_j(2,:)+x_j(2);
% 
%             z_diff = obj.tag_offsets(2,3) - obj.tag_offsets(1,3);
% 
%             f = [   norm([X_i(1)-X_j(1), Y_i(1)-Y_j(1), 0      ], 2);...
%                     norm([X_i(1)-X_j(2), Y_i(1)-Y_j(2), z_diff ], 2);...
%                     norm([X_i(2)-X_j(1), Y_i(2)-Y_j(1), z_diff ], 2);...
%                     norm([X_i(2)-X_j(2), Y_i(2)-Y_j(2), 0      ], 2)];
% 
%             z = [   dcl_dist(j*2-1, 1);...
%                     dcl_dist(j*2,   1);...
%                     dcl_dist(j*2-1, 2);...
%                     dcl_dist(j*2,   2)];
% 
%             Sigma_ij = sigma_ij * sigma_ji';
%             Sigma_aa = [Sigma_ii,   Sigma_ij;...
%                         Sigma_ij',  Sigma_jj];
%                 
%             F = zeros(4,6*2);
%             F(1,1) = (X_i(1)-X_j(1)) / sqrt(z(1));
%             F(1,2) = (Y_i(1)-Y_j(1)) / sqrt(z(1));
%             F(1,3) = -1 / sqrt(z(1)) * ...
%                 ((X_i(1)-X_j(1))*( obj.tag_offsets(1,1)*sin(x_i(3)) + obj.tag_offsets(1,2)*cos(x_i(3))) +...
%                  (Y_i(1)-Y_j(1))*(-obj.tag_offsets(1,1)*cos(x_i(3)) + obj.tag_offsets(1,2)*sin(x_i(3))) );        
% 
%             F(2,1) = (X_i(1)-X_j(2)) / sqrt(z(2));
%             F(2,2) = (Y_i(1)-Y_j(2)) / sqrt(z(2));
%             F(2,3) = -1 / sqrt(z(2)) * ...
%                 ((X_i(1)-X_j(2))*( obj.tag_offsets(1,1)*sin(x_i(3)) + obj.tag_offsets(1,2)*cos(x_i(3))) +...
%                  (Y_i(1)-Y_j(2))*(-obj.tag_offsets(1,1)*cos(x_i(3)) + obj.tag_offsets(1,2)*sin(x_i(3))) );    
% 
%             F(3,1) = (X_i(2)-X_j(1)) / sqrt(z(3));
%             F(3,2) = (Y_i(2)-Y_j(1)) / sqrt(z(3));
%             F(3,3) = -1 / sqrt(z(3)) * ...
%                 ((X_i(2)-X_j(1))*( obj.tag_offsets(2,1)*sin(x_i(3)) + obj.tag_offsets(2,2)*cos(x_i(3))) +...
%                  (Y_i(2)-Y_j(1))*(-obj.tag_offsets(2,1)*cos(x_i(3)) + obj.tag_offsets(2,2)*sin(x_i(3))) );       
% 
%             F(4,1) = (X_i(2)-X_j(2)) / sqrt(z(4));
%             F(4,2) = (Y_i(2)-Y_j(2)) / sqrt(z(4));
%             F(4,3) = -1 / sqrt(z(4)) * ...
%                 ((X_i(2)-X_j(2))*( obj.tag_offsets(2,1)*sin(x_i(3)) + obj.tag_offsets(2,2)*cos(x_i(3))) +...
%                  (Y_i(2)-Y_j(2))*(-obj.tag_offsets(2,1)*cos(x_i(3)) + obj.tag_offsets(2,2)*sin(x_i(3))) );    
%              
%              
%              
%             F(1,6+1) = -(X_i(1)-X_j(1)) / sqrt(z(1));
%             F(1,6+2) = -(Y_i(1)-Y_j(1)) / sqrt(z(1));
%             F(1,6+3) = 1 / sqrt(z(1)) * ...
%                 ((X_i(1)-X_j(1))*( obj.tag_offsets(1,1)*sin(x_j(3)) + obj.tag_offsets(1,2)*cos(x_j(3))) +...
%                  (Y_i(1)-Y_j(1))*(-obj.tag_offsets(1,1)*cos(x_j(3)) + obj.tag_offsets(1,2)*sin(x_j(3))) );        
% 
%             F(2,6+1) = -(X_i(1)-X_j(2)) / sqrt(z(2));
%             F(2,6+2) = -(Y_i(1)-Y_j(2)) / sqrt(z(2));
%             F(2,6+3) = 1 / sqrt(z(2)) * ...
%                 ((X_i(1)-X_j(2))*( obj.tag_offsets(1,1)*sin(x_j(3)) + obj.tag_offsets(1,2)*cos(x_j(3))) +...
%                  (Y_i(1)-Y_j(2))*(-obj.tag_offsets(1,1)*cos(x_j(3)) + obj.tag_offsets(1,2)*sin(x_j(3))) );    
% 
%             F(3,6+1) = -(X_i(2)-X_j(1)) / sqrt(z(3));
%             F(3,6+2) = -(Y_i(2)-Y_j(1)) / sqrt(z(3));
%             F(3,6+3) = 1 / sqrt(z(3)) * ...
%                 ((X_i(2)-X_j(1))*( obj.tag_offsets(2,1)*sin(x_j(3)) + obj.tag_offsets(2,2)*cos(x_j(3))) +...
%                  (Y_i(2)-Y_j(1))*(-obj.tag_offsets(2,1)*cos(x_j(3)) + obj.tag_offsets(2,2)*sin(x_j(3))) );       
% 
%             F(4,6+1) = -(X_i(2)-X_j(2)) / sqrt(z(4));
%             F(4,6+2) = -(Y_i(2)-Y_j(2)) / sqrt(z(4));
%             F(4,6+3) = 1 / sqrt(z(4)) * ...
%                 ((X_i(2)-X_j(2))*( obj.tag_offsets(2,1)*sin(x_j(3)) + obj.tag_offsets(2,2)*cos(x_j(3))) +...
%                  (Y_i(2)-Y_j(2))*(-obj.tag_offsets(2,1)*cos(x_j(3)) + obj.tag_offsets(2,2)*sin(x_j(3))) );    
%              
% 
% 
%             Q = eye(4)*obj.uwb_err;
%             K = (Sigma_aa * F') / (F * Sigma_aa * F' + Q);
% 
%             del = K * (z - f);
%             obj.dcl_x = obj.dcl_x + del(1:6);
%             
%             Sigma_t = (eye(6*2) - K*F)*Sigma_aa;
%             Sigma_ii = Sigma_t(1:6, 1:6 );
%             Sigma_ij = Sigma_t(1:6, 7:12);
%             
%             for k = obj.setdiff2(obj.nCars, obj.car_num, j)
%                 if obj.dcl_init(k)
%                     p = (k-1)*6+1;
%                     obj.dcl_sigma(p:p+5,:) = Sigma_ii / obj.dcl_Sigma(n:n+5,n:n+5) * obj.dcl_sigma(p:p+5,:);
%                 else
%                     obj.dcl_init(k) = true;
%                 end
%             end
%             
%             obj.dcl_Sigma(n:n+5,n:n+5) = Sigma_ii;
%             obj.dcl_sigma(m:m+5,:) = Sigma_ij;
%             
%         end
%     end
%     
%     obj.uwb.set_dcl(obj.car_num, obj.dcl_x, obj.dcl_Sigma(n:n+5,n:n+5), obj.dcl_sigma);

        
end
