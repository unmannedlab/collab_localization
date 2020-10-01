function [] = estimate_ekf(obj)

% Predict IMU
    if mod(obj.tick, obj.rate / obj.rate_imu) == 0
        kf_predict(obj);
    end
    
    % Update Kinematic Model
    if obj.kf_mdl && mod(obj.tick, obj.rate / obj.rate_mdl) == 0
        kf_update_mdl(obj);
    end
    
    % Update GPS
    if obj.kf_gps && mod(obj.tick, obj.rate / obj.rate_gps) == 0
        kf_update_gps(obj);
    end
end

function [obj] = kf_predict(obj)
    dt = 1/obj.rate_imu;
    [accel, gyro, mag] = obj.sense_imu();
    
    theta = obj.ekf_x(3);
    
    R = [   cos(theta + pi/2), -sin(theta + pi/2),  0;...
            sin(theta + pi/2),  cos(theta + pi/2),  0;...
            0,                  0,                  1];

    accel_r = R * accel';

    obj.ekf_x = [...
                obj.ekf_x(1) + obj.ekf_x(4) * dt + accel_r(1) / 2 * dt^2;...
                obj.ekf_x(2) + obj.ekf_x(5) * dt + accel_r(2) / 2 * dt^2;...
               (obj.ekf_x(3) + gyro(3)  * dt) * 0.98 + 0.02 * mag(3);...
                obj.ekf_x(4) + accel_r(1) * dt;...
                obj.ekf_x(5) + accel_r(2) * dt;...
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

    obj.ekf_cov = F * obj.ekf_cov * F' + Q;

%     obj.uwb.set_car_state(obj.car_num, obj.ekf_x, obj.ekf_cov);
end
function [obj] = kf_update_mdl(obj)

    [ vel, delta ] = obj.sense_model();

    z = [   vel;...
            vel*tan(delta)/obj.wb ];
    
    kf_vel = sqrt( obj.ekf_x(4)^2 + obj.ekf_x(5)^2 );
    
    H = [   0, 0, 0, obj.ekf_x(4) / kf_vel, obj.ekf_x(5) / kf_vel, 0;...
            0, 0, 0, 0, 0, 1];

    h = [kf_vel; obj.ekf_x(6)];
        
    r2 = diag([ obj.enc_err;...
                obj.str_err * vel / obj.wb]);

    K = obj.ekf_cov * H' / (H * obj.ekf_cov * H' + r2);

    obj.ekf_x   = obj.ekf_x + K*(z - h);
    obj.ekf_cov = (eye(6) - K*H) * obj.ekf_cov;
    
%     obj.uwb.set_car_state(obj.car_num, obj.ekf_x, obj.ekf_cov);
    
end
function [obj] = kf_update_gps(obj)
    
    [ x, y, theta, vel ] = sense_gps(obj); 
    
    z = [   x;...
            y;...
            theta;...
            vel];

    kf_vel = sqrt( obj.ekf_x(4)^2 + obj.ekf_x(5)^2 );
    
    H = [   1, 0, 0, 0, 0, 0;...
            0, 1, 0, 0, 0, 0;...
            0, 0, 1, 0, 0, 0;...
            0, 0, 0, obj.ekf_x(4) / kf_vel, obj.ekf_x(5) / kf_vel, 0];
    
    h = [   obj.ekf_x(1:3);...
            kf_vel];
        
    R = diag([  obj.gps_cep;...
                obj.gps_cep;...
                obj.gps_her;...
                obj.gps_ver]);

    K = obj.ekf_cov * H' / (H * obj.ekf_cov * H' + R);

    obj.ekf_x = obj.ekf_x + K*(z - h);
    obj.ekf_cov = (eye(6) - K*H)*obj.ekf_cov;
    
%     obj.uwb.set_car_state(obj.car_num, obj.ekf_x, obj.ekf_cov);
end
