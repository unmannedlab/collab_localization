function [ accel, gyro, mag ] = sense_imu(obj)
    
    accel = normrnd([   obj.accel, obj.vel^2 / obj.wb * tan(obj.delta),0],...
                        obj.imu_acc_err);
    gyro  = normrnd([0,0,obj.vel*tan(obj.delta)/obj.wb], obj.imu_gyr_err);
    mag   = normrnd([0,0,obj.theta],                     obj.imu_mag_err);
    
end

