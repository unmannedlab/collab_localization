rate = 150;
dt = 1/rate;
ticks = 0:runtime*rate-1;

rate_imu = 150;
rate_mdl =  30;
rate_gps =  10;
rate_uwb =   3;

wb = 4;
tw = 2;

gps_per = 1.2       *10^SF;
gps_her = 0.035     *10^SF;
gps_ver = 0.05      *10^SF;
enc_err = 0.05      *10^SF;
str_err = 0.05      *10^SF;
uwb_err = 0.30      *10^SF;
imu_acc_err = 0.1   *10^SF;
imu_gyr_err = 0.1   *10^SF;
imu_mag_err = 0.1   *10^SF;

nLmk = size(lmks,1);
nTicks = length(ticks);
acc = zeros(nTicks, nCars);
del = zeros(nTicks, nCars);

i = 1;
for t = 1:nTicks
    if i < size(acc_cmd,1) && ticks(t)/rate == acc_cmd(i+1,1)
        i = i + 1;
    end
    acc(t,:) = acc_cmd(i,2:end);
end

i = 1;
for t = 1:nTicks
    if i < size(del_cmd,1) && ticks(t)/rate == del_cmd(i+1,1)
        i = i + 1;
    end
    del(t,:) = del_cmd(i,2:end);
end


clear i