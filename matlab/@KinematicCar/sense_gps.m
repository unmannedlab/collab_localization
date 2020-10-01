function [ x, y, theta, v ] = sense_gps(obj)

% https://en.wikipedia.org/wiki/Circular_error_probable

    dist = raylrnd(0.8493 * obj.gps_cep);
    theta = rand()*2*pi;
    
    x = obj.x_pos + dist * cos(theta);
    y = obj.y_pos + dist * sin(theta);
    
    theta = normrnd(obj.theta, obj.gps_her);
    
    v = normrnd(obj.vel, obj.gps_ver);

end