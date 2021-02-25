% Truth Model
%
% x(1) : x position
% x(2) : y position
% x(3) : orientation
% x(4) : velocity
%

if t == 1
    x_truth = zeros(4,nCars,nTicks);
    x_truth(:,:,1) = x0;
else
    
    x_truth(:,:,t) = x_truth(:,:,t-1) ...
        + [-dt * x_truth(4,:,t-1) .* sin(x_truth(3,:,t-1));...
            dt * x_truth(4,:,t-1) .* cos(x_truth(3,:,t-1));...
            dt * x_truth(4,:,t-1) .* tan(del(t,:))/wb;...
            dt * acc(t,:)];
end
