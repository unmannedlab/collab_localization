function [] = estimate_ckf(obj)

% Predict IMU
    if mod(obj.tick, obj.rate / obj.rate_imu) == 0
        ckf_control(obj);
    end
            
% Update Kinematic Model
    if obj.kf_mdl && mod(obj.tick, obj.rate / obj.rate_mdl) == 0
        ckf_update_mdl(obj);
    end
    
% Update GPS
    if obj.kf_gps && mod(obj.tick, obj.rate / obj.rate_gps) == 0
        ckf_update_gps(obj);
    end

% Update UWB Landmarks and Collaboration
    if obj.kf_uwb && mod(obj.tick, obj.rate / obj.rate_uwb) == 0 
        ckf_update_uwb_lmk(obj);
        ckf_update_uwb_collab(obj);
    end
end

function [obj] = ckf_control(obj)
    dt = 1/obj.rate_imu;
    [accel, gyro, mag] = obj.sense_imu();
    
    theta = obj.ckf_x(3);
    
    R = [   cos(theta + pi/2), -sin(theta + pi/2),  0;...
            sin(theta + pi/2),  cos(theta + pi/2),  0;...
            0,                  0,                  1];

    accel_r = R * accel';

    obj.ckf_x = [...
                obj.ckf_x(1) + obj.ckf_x(4) * dt + accel_r(1) / 2 * dt^2;...
                obj.ckf_x(2) + obj.ckf_x(5) * dt + accel_r(2) / 2 * dt^2;...
               (obj.ckf_x(3) + gyro(3)  * dt) * 0.98 + 0.02 * mag(3);...
                obj.ckf_x(4) + accel_r(1) * dt;...
                obj.ckf_x(5) + accel_r(2) * dt;...
                gyro(3)];

    R = diag([  obj.imu_acc_err/2/obj.rate_imu^2    ;...
                obj.imu_acc_err/2/obj.rate_imu^2    ;...
                obj.imu_gyr_err/obj.rate_imu        ;...
                obj.imu_acc_err/obj.rate_imu        ;...
                obj.imu_acc_err/obj.rate_imu        ;...
                obj.imu_gyr_err]);

    G = [   1, 0, 0, dt, 0, 0;...
            0, 1, 0, 0, dt, 0;...
            0, 0, 1, 0, 0, dt;...
            0, 0, 0, 1, 0, 0;...
            0, 0, 0, 0, 1, 0;...
            0, 0, 0, 0, 0, 1];

    n = (obj.nCars-1)*6+1;
    obj.ckf_Sigma(n:n+5,n:n+5) = G * obj.ckf_Sigma(n:n+5,n:n+5) * G' + R;
    
    for j = setdiff(1:obj.nCars, obj.car_num)
        m = (j-1)*6+1;
        obj.ckf_sigma(m:m+5,:) = G * obj.ckf_sigma(m:m+5,:);
    end

    obj.uwb.set_ckf(obj.car_num, obj.ckf_x, obj.ckf_Sigma(n:n+5,n:n+5), obj.ckf_sigma);
end
function [obj] = ckf_update_mdl(obj)

    [ vel, delta ] = obj.sense_model();

    z = [   vel;...
            vel*tan(delta)/obj.wb ];
    
    kf_vel = sqrt( obj.ckf_x(4)^2 + obj.ckf_x(5)^2 );
    
    H = [   0, 0, 0, obj.ckf_x(4) / kf_vel, obj.ckf_x(5) / kf_vel, 0;...
            0, 0, 0, 0, 0, 1];

    h = [kf_vel; obj.ckf_x(6)];
        
    Q = diag([  obj.enc_err;...
                obj.str_err * vel / obj.wb]);

    n = (obj.nCars-1)*6+1;
    K = obj.ckf_Sigma(n:n+5,n:n+5) * H' / (H * obj.ckf_Sigma(n:n+5,n:n+5) * H' + Q);

    obj.ckf_x   = obj.ckf_x + K*(z - h);
    obj.ckf_Sigma(n:n+5,n:n+5) = (eye(6) - K*H) * obj.ckf_Sigma(n:n+5,n:n+5);
    
    for j = setdiff(1:obj.nCars, obj.car_num)
        m = (j-1)*6+1;
        obj.ckf_sigma(m:m+5,:) = (eye(6) - K*H) * obj.ckf_sigma(m:m+5,:);
    end
    
    obj.uwb.set_ckf(obj.car_num, obj.ckf_x, obj.ckf_Sigma(n:n+5,n:n+5), obj.ckf_sigma);
    
end
function [obj] = ckf_update_gps(obj)
    
    [ x, y, theta, vel ] = sense_gps(obj); 
    
    z = [   x;...
            y;...
            theta;...
            vel];

    kf_vel = sqrt( obj.ckf_x(4)^2 + obj.ckf_x(5)^2 );
    
    H = [   1, 0, 0, 0, 0, 0;...
            0, 1, 0, 0, 0, 0;...
            0, 0, 1, 0, 0, 0;...
            0, 0, 0, obj.ckf_x(4) / kf_vel, obj.ckf_x(5) / kf_vel, 0];
    
    h = [   obj.ckf_x(1:3);...
            kf_vel];
        
    Q = diag([  obj.gps_cep;...
                obj.gps_cep;...
                obj.gps_her;...
                obj.gps_ver]);

    n = (obj.nCars-1)*6+1;
    K = obj.ckf_Sigma(n:n+5,n:n+5) * H' / (H * obj.ckf_Sigma(n:n+5,n:n+5) * H' + Q);

    obj.ckf_x = obj.ckf_x + K*(z - h);
    obj.ckf_Sigma(n:n+5,n:n+5) = (eye(6) - K*H)*obj.ckf_Sigma(n:n+5,n:n+5);
    
    for j = setdiff(1:obj.nCars, obj.car_num)
        m = (j-1)*6+1;
        obj.ckf_sigma(m:m+5,:) = (eye(6) - K*H) * obj.ckf_sigma(m:m+5,:);
    end
    
    obj.uwb.set_ckf(obj.car_num, obj.ckf_x, obj.ckf_Sigma(n:n+5,n:n+5), obj.ckf_sigma);
end
function [obj] = ckf_update_uwb_lmk(obj)
    n = (obj.nCars-1)*6+1;
    
    [~, lmk_dist] = obj.sense_uwb();
    
    
    for i = 1:size(lmk_dist,1)
        if all(~isinf(lmk_dist(i, :)))
          
            lmk_x = obj.uwb.lmk_tags(:,i);

            R = [   cos(obj.ckf_x(3)), -sin(obj.ckf_x(3));...
                    sin(obj.ckf_x(3)),  cos(obj.ckf_x(3))];

            off = R * obj.tag_offsets(:,1:2)';

            X1 = off(1,:)+obj.ckf_x(1);
            Y1 = off(2,:)+obj.ckf_x(2);
            Z1 = obj.tag_offsets(:,3);

            h = [   norm([X1(1)-lmk_x(1), Y1(1)-lmk_x(2), Z1(1)-lmk_x(3)],2);...
                    norm([X1(2)-lmk_x(1), Y1(2)-lmk_x(2), Z1(2)-lmk_x(3)],2)];

            z = [   lmk_dist(i, 1);...
                    lmk_dist(i, 2)];

            H = zeros(2,6);
            H(1,1) = (X1(1)-lmk_x(1)) / sqrt(z(1));
            H(1,2) = (Y1(1)-lmk_x(2)) / sqrt(z(1));
            H(1,3) = 1 / sqrt(z(1)) * ...
                ((X1(1)-lmk_x(1))*( obj.tag_offsets(1,1)*sin(obj.ckf_x(3)) + obj.tag_offsets(1,2)*cos(obj.ckf_x(3))) +...
                 (Y1(1)-lmk_x(2))*(-obj.tag_offsets(1,1)*cos(obj.ckf_x(3)) + obj.tag_offsets(1,2)*sin(obj.ckf_x(3))) );        

            H(2,1) = (X1(2)-lmk_x(1)) / sqrt(z(2));
            H(2,2) = (Y1(2)-lmk_x(2)) / sqrt(z(2));
            H(2,3) = 1 / sqrt(z(2)) * ...
                ((X1(2)-lmk_x(1))*( obj.tag_offsets(1,1)*sin(obj.ckf_x(3)) + obj.tag_offsets(1,2)*cos(obj.ckf_x(3))) +...
                 (Y1(2)-lmk_x(2))*(-obj.tag_offsets(1,1)*cos(obj.ckf_x(3)) + obj.tag_offsets(1,2)*sin(obj.ckf_x(3))) );    

            Q = eye(2)* obj.uwb_err;

            K = obj.ckf_Sigma(n:n+5,n:n+5) * H' / (H * obj.ckf_Sigma(n:n+5,n:n+5) * H' + Q);
            obj.ckf_x = obj.ckf_x + K * (z - h);
            obj.ckf_Sigma(n:n+5,n:n+5) = (eye(6) - K*H) * obj.ckf_Sigma(n:n+5,n:n+5);
            
            for j = setdiff(1:obj.nCars, obj.car_num)
                m = (j-1)*6+1;
                obj.ckf_sigma(m:m+5,:) = (eye(6) - K*H) * obj.ckf_sigma(m:m+5,:);
            end
    
        end 
    end
    
    obj.uwb.set_ckf(obj.car_num, obj.ckf_x, obj.ckf_Sigma(n:n+5,n:n+5), obj.ckf_sigma);
end
function [obj] = ckf_update_uwb_collab(obj)
    n = (obj.nCars-1)*6+1;
    
    [ckf_dist, ~] = obj.sense_uwb();
        
    for i = setdiff(1:length(ckf_dist(:,1))/2, obj.car_num)
        m = (i-1)*6+1;
        
        x_i = obj.ckf_x;
        Sigma_ii = obj.ckf_Sigma(n:n+5,n:n+5);
        sigma_ij = obj.ckf_sigma(m:m+5,:);
        
        x_j = obj.uwb.ckf_states(:,i);
        Sigma_jj = obj.uwb.ckf_Sigma(:,:,i);
        sigma_ji = obj.uwb.ckf_sigma(n:n+5,:,i);
        
        if all(~isnan(x_j)) && all(all(~isinf(ckf_dist(i*2-1:i*2, :))))
        
            Ri = [  cos(x_i(3)), -sin(x_i(3));...
                    sin(x_i(3)),  cos(x_i(3))];

            Rj = [  cos(x_j(3)), -sin(x_j(3));...
                    sin(x_j(3)),  cos(x_j(3))];

            off1 = Ri * obj.tag_offsets(:,1:2)';
            off2 = Rj * obj.tag_offsets(:,1:2)';

            X1 = off1(1,:)+x_i(1);
            Y1 = off1(2,:)+x_i(2);

            X2 = off2(1,:)+x_j(1);
            Y2 = off2(2,:)+x_j(2);

            z_diff = obj.tag_offsets(2,3) - obj.tag_offsets(1,3);

            f = [   norm([X1(1)-X2(1), Y1(1)-Y2(1), 0       ],2);...
                    norm([X1(1)-X2(2), Y1(1)-Y2(2), z_diff  ],2);...
                    norm([X1(2)-X2(1), Y1(2)-Y2(1), z_diff  ],2);...
                    norm([X1(2)-X2(2), Y1(2)-Y2(2), 0       ],2)];

            z = [   ckf_dist(i*2-1, 1);...
                    ckf_dist(i*2,   1);...
                    ckf_dist(i*2-1, 2);...
                    ckf_dist(i*2,   2)];

            Sigma_ij = sigma_ij * sigma_ji';
            Sigma_aa = [Sigma_ii,   Sigma_ij;...
                        Sigma_ij',  Sigma_jj];
                
            F = zeros(4,6*2);
            F(1,1) = (X1(1)-X2(1)) / sqrt(z(1));
            F(1,2) = (Y1(1)-Y2(1)) / sqrt(z(1));
            F(1,3) =-1 / sqrt(z(1)) * ...
                ((X1(1)-X2(1))*( obj.tag_offsets(1,1)*sin(x_i(3)) + obj.tag_offsets(1,2)*cos(x_i(3))) +...
                 (Y1(1)-Y2(1))*(-obj.tag_offsets(1,1)*cos(x_i(3)) + obj.tag_offsets(1,2)*sin(x_i(3))) );        

            F(2,1) = (X1(1)-X2(2)) / sqrt(z(2));
            F(2,2) = (Y1(1)-Y2(2)) / sqrt(z(2));
            F(2,3) =-1 / sqrt(z(2)) * ...
                ((X1(1)-X2(2))*( obj.tag_offsets(1,1)*sin(x_i(3)) + obj.tag_offsets(1,2)*cos(x_i(3))) +...
                 (Y1(1)-Y2(2))*(-obj.tag_offsets(1,1)*cos(x_i(3)) + obj.tag_offsets(1,2)*sin(x_i(3))) );    

            F(3,1) = (X1(2)-X2(1)) / sqrt(z(3));
            F(3,2) = (Y1(2)-Y2(1)) / sqrt(z(3));
            F(3,3) =-1 / sqrt(z(3)) * ...
                ((X1(2)-X2(1))*( obj.tag_offsets(2,1)*sin(x_i(3)) + obj.tag_offsets(2,2)*cos(x_i(3))) +...
                 (Y1(2)-Y2(1))*(-obj.tag_offsets(2,1)*cos(x_i(3)) + obj.tag_offsets(2,2)*sin(x_i(3))) );       

            F(4,1) = (X1(2)-X2(2)) / sqrt(z(4));
            F(4,2) = (Y1(2)-Y2(2)) / sqrt(z(4));
            F(4,3) =-1 / sqrt(z(4)) * ...
                ((X1(2)-X2(2))*( obj.tag_offsets(2,1)*sin(x_i(3)) + obj.tag_offsets(2,2)*cos(x_i(3))) +...
                 (Y1(2)-Y2(2))*(-obj.tag_offsets(2,1)*cos(x_i(3)) + obj.tag_offsets(2,2)*sin(x_i(3))) );    
             
             
             
            F(1,6+1) = -(X1(1)-X2(1)) / sqrt(z(1));
            F(1,6+2) = -(Y1(1)-Y2(1)) / sqrt(z(1));
            F(1,6+3) = 1 / sqrt(z(1)) * ...
                ((X1(1)-X2(1))*( obj.tag_offsets(1,1)*sin(x_j(3)) + obj.tag_offsets(1,2)*cos(x_j(3))) +...
                 (Y1(1)-Y2(1))*(-obj.tag_offsets(1,1)*cos(x_j(3)) + obj.tag_offsets(1,2)*sin(x_j(3))) );        

            F(2,6+1) = -(X1(1)-X2(2)) / sqrt(z(2));
            F(2,6+2) = -(Y1(1)-Y2(2)) / sqrt(z(2));
            F(2,6+3) = 1 / sqrt(z(2)) * ...
                ((X1(1)-X2(2))*( obj.tag_offsets(1,1)*sin(x_j(3)) + obj.tag_offsets(1,2)*cos(x_j(3))) +...
                 (Y1(1)-Y2(2))*(-obj.tag_offsets(1,1)*cos(x_j(3)) + obj.tag_offsets(1,2)*sin(x_j(3))) );    

            F(3,6+1) = -(X1(2)-X2(1)) / sqrt(z(3));
            F(3,6+2) = -(Y1(2)-Y2(1)) / sqrt(z(3));
            F(3,6+3) = 1 / sqrt(z(3)) * ...
                ((X1(2)-X2(1))*( obj.tag_offsets(2,1)*sin(x_j(3)) + obj.tag_offsets(2,2)*cos(x_j(3))) +...
                 (Y1(2)-Y2(1))*(-obj.tag_offsets(2,1)*cos(x_j(3)) + obj.tag_offsets(2,2)*sin(x_j(3))) );       

            F(4,6+1) = -(X1(2)-X2(2)) / sqrt(z(4));
            F(4,6+2) = -(Y1(2)-Y2(2)) / sqrt(z(4));
            F(4,6+3) = 1 / sqrt(z(4)) * ...
                ((X1(2)-X2(2))*( obj.tag_offsets(2,1)*sin(x_j(3)) + obj.tag_offsets(2,2)*cos(x_j(3))) +...
                 (Y1(2)-Y2(2))*(-obj.tag_offsets(2,1)*cos(x_j(3)) + obj.tag_offsets(2,2)*sin(x_j(3))) );    
             


            Q = eye(4)*obj.uwb_err;
            K = (Sigma_aa * F') / (F * Sigma_aa * F' + Q);

            del = K * (z - f);
            obj.ckf_x = obj.ckf_x + del(1:6);
            
            Sigma_t = (eye(6*2) - K*F)*Sigma_aa;
            Sigma_ii = Sigma_t(1:6, 1:6 );
            Sigma_ij = Sigma_t(1:6, 7:12);
            
            for j = setdiff(1:obj.nCars, [obj.car_num, i])
                if obj.ckf_init(j)
                    k = (j-1)*6+1;
                    obj.ckf_sigma(k:k+5,:) = Sigma_ii / obj.ckf_Sigma(n:n+5,n:n+5) * obj.ckf_sigma(k:k+5,:);
                else
                    obj.ckf_init(j) = true;
                end
            end
            
            obj.ckf_Sigma(n:n+5,n:n+5) = Sigma_ii;
            obj.ckf_sigma(m:m+5,:) = Sigma_ij;
            
        end
    end
    
    obj.uwb.set_ckf(obj.car_num, obj.ckf_x, obj.ckf_Sigma(n:n+5,n:n+5), obj.ckf_sigma);
end
