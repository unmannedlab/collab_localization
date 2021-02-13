function [] = estimate_ekf_lmk(obj)

    % Predict IMU
    if mod(obj.tick, obj.rate / obj.rate_imu) == 0
        predict(obj);
    end

    % Update Kinematic Model
    if obj.kf_mdl && mod(obj.tick, obj.rate / obj.rate_mdl) == 0
        update_mdl(obj);
    end
    
    % Update GPS
    if obj.kf_gps && mod(obj.tick, obj.rate / obj.rate_gps) == 0
        update_gps(obj);
    end

    % Update UWB
    if obj.kf_uwb && mod(obj.tick, obj.rate / obj.rate_uwb) == 0 
        update_lmk(obj);
    end
    
end

function [obj] = predict(obj)

    dt = 1/obj.rate_imu;
    [accel, gyro, mag] = obj.sense_imu();
    
    theta = obj.ekf_lmk_x(3);
    
    R = [   cos(theta + pi/2), -sin(theta + pi/2),  0;...
            sin(theta + pi/2),  cos(theta + pi/2),  0;...
            0,                  0,                  1];

    accel_r = R * accel';

    obj.ekf_lmk_x = [...
                obj.ekf_lmk_x(1) + obj.ekf_lmk_x(4) * dt + accel_r(1) / 2 * dt^2;...
                obj.ekf_lmk_x(2) + obj.ekf_lmk_x(5) * dt + accel_r(2) / 2 * dt^2;...
               (obj.ekf_lmk_x(3) + gyro(3)  * dt) * 0.98 + 0.02 * mag(3);...
                obj.ekf_lmk_x(4) + accel_r(1) * dt;...
                obj.ekf_lmk_x(5) + accel_r(2) * dt;...
                gyro(3)];

    Q = diag([  obj.imu_acc_err/2/obj.rate_imu^2    ;...
                obj.imu_acc_err/2/obj.rate_imu^2    ;...
                obj.imu_gyr_err/obj.rate_imu        ;...
                obj.imu_acc_err/obj.rate_imu        ;...
                obj.imu_acc_err/obj.rate_imu        ;...
                obj.imu_gyr_err]);

    F = [   1, 0, 0, dt, 0, 0;...
            0, 1, 0, 0, dt, 0;...
            0, 0, 1, 0, 0, dt;...
            0, 0, 0, 1, 0, 0;...
            0, 0, 0, 0, 1, 0;...
            0, 0, 0, 0, 0, 1];

    obj.ekf_lmk_cov = F * obj.ekf_lmk_cov * F' + Q;
    
end
function [obj] = update_mdl(obj)

    [ vel, delta ] = obj.sense_model();

    z = [   vel;...
            vel*tan(delta)/obj.wb ];
    
    kf_vel = sqrt( obj.ekf_lmk_x(4)^2 + obj.ekf_lmk_x(5)^2 );
    
    H = [   0, 0, 0, obj.ekf_lmk_x(4) / kf_vel, obj.ekf_lmk_x(5) / kf_vel, 0;...
            0, 0, 0, 0, 0, 1];

    h = [kf_vel; obj.ekf_lmk_x(6)];
        
    r2 = diag([ obj.enc_err;...
                obj.str_err * vel / obj.wb]);

    K = obj.ekf_lmk_cov * H' / (H * obj.ekf_lmk_cov * H' + r2);

    obj.ekf_lmk_x   = obj.ekf_lmk_x + K*(z - h);
    obj.ekf_lmk_cov = (eye(6) - K*H) * obj.ekf_lmk_cov;
    
end
function [obj] = update_gps(obj)
    
    [ x, y, theta, vel ] = sense_gps(obj); 
    
    z = [   x;...
            y;...
            theta;...
            vel];

    kf_vel = sqrt( obj.ekf_lmk_x(4)^2 + obj.ekf_lmk_x(5)^2 );
    
    H = [   1, 0, 0, 0, 0, 0;...
            0, 1, 0, 0, 0, 0;...
            0, 0, 1, 0, 0, 0;...
            0, 0, 0, obj.ekf_lmk_x(4) / kf_vel, obj.ekf_lmk_x(5) / kf_vel, 0];
    
    h = [   obj.ekf_lmk_x(1:3);...
            kf_vel];
        
    R = diag([  obj.gps_cep;...
                obj.gps_cep;...
                obj.gps_her;...
                obj.gps_ver]);

    K = obj.ekf_lmk_cov * H' / (H * obj.ekf_lmk_cov * H' + R);

    obj.ekf_lmk_x = obj.ekf_lmk_x + K*(z - h);
    obj.ekf_lmk_cov = (eye(6) - K*H)*obj.ekf_lmk_cov;

end
function [obj] = update_lmk(obj)
    
    [~, lmk_dist] = obj.sense_uwb();
    
    for i = 1:size(lmk_dist,1)
        if all(~isinf(lmk_dist(i, :)))
          
            lmk_x = obj.uwb.lmk_tags(:,i);

            R = [   cos(obj.ekf_lmk_x(3)), -sin(obj.ekf_lmk_x(3));...
                    sin(obj.ekf_lmk_x(3)),  cos(obj.ekf_lmk_x(3))];

            off = R * obj.tag_offsets(:,1:2)';

            X1 = off(1,:)+obj.ekf_lmk_x(1);
            Y1 = off(2,:)+obj.ekf_lmk_x(2);
            Z1 = obj.tag_offsets(:,3);

            h = [   norm([X1(1)-lmk_x(1), Y1(1)-lmk_x(2), Z1(1)-lmk_x(3)],2);...
                    norm([X1(2)-lmk_x(1), Y1(2)-lmk_x(2), Z1(2)-lmk_x(3)],2)];

            z = [   lmk_dist(i, 1);...
                    lmk_dist(i, 2)];

            H = zeros(2,6);
            H(1,1) = (X1(1)-lmk_x(1)) / sqrt(z(1));
            H(1,2) = (Y1(1)-lmk_x(2)) / sqrt(z(1));
            H(1,3) = -1 / sqrt(z(1)) * ...
                ((X1(1)-lmk_x(1))*( obj.tag_offsets(1,1)*sin(obj.ekf_lmk_x(3)) + obj.tag_offsets(1,2)*cos(obj.ekf_lmk_x(3))) +...
                 (Y1(1)-lmk_x(2))*(-obj.tag_offsets(1,1)*cos(obj.ekf_lmk_x(3)) + obj.tag_offsets(1,2)*sin(obj.ekf_lmk_x(3))) );        

            H(2,1) = (X1(2)-lmk_x(1)) / sqrt(z(2));
            H(2,2) = (Y1(2)-lmk_x(2)) / sqrt(z(2));
            H(2,3) = -1 / sqrt(z(2)) * ...
                ((X1(2)-lmk_x(1))*( obj.tag_offsets(1,1)*sin(obj.ekf_lmk_x(3)) + obj.tag_offsets(1,2)*cos(obj.ekf_lmk_x(3))) +...
                 (Y1(2)-lmk_x(2))*(-obj.tag_offsets(1,1)*cos(obj.ekf_lmk_x(3)) + obj.tag_offsets(1,2)*sin(obj.ekf_lmk_x(3))) );    

            R = eye(2)* obj.uwb_err;

            K = obj.ekf_lmk_cov * H' / (H * obj.ekf_lmk_cov * H' + R);
            obj.ekf_lmk_x = obj.ekf_lmk_x + K * (z - h);
            obj.ekf_lmk_cov = (eye(6) - K*H) * obj.ekf_lmk_cov;
        end 
    end
end