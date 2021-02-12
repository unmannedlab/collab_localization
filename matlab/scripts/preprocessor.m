rate = 1200;
dt = 1/rate;
ticks = 0:runtime*rate-1;

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