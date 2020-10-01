function obj = step(obj)

    obj.x_pos   = obj.x_pos - obj.dt*obj.vel*sin(obj.theta);
    obj.y_pos   = obj.y_pos + obj.dt*obj.vel*cos(obj.theta); 
    obj.theta   = obj.theta + obj.dt*obj.vel*tan(obj.delta)/obj.wb;
    obj.vel     = obj.vel + obj.dt*obj.accel;

    R = [   cos(obj.theta), -sin(obj.theta);...
            sin(obj.theta),  cos(obj.theta)];

    RA = R * obj.tag_offsets(:,1:2)'; 

    obj.tags(1,1:2)  = RA(:,1)' + [obj.x_pos, obj.y_pos];
    obj.tags(2,1:2)  = RA(:,2)' + [obj.x_pos, obj.y_pos];
    obj.tags(:,3)    = obj.tag_offsets(:,3);

    obj.uwb.set_car_tag(obj.car_num, 1,... 
                        obj.tags(1,1), ...
                        obj.tags(1,2), ...
                        obj.tags(1,3));

    obj.uwb.set_car_tag(obj.car_num, 2,... 
                        obj.tags(2,1), ...
                        obj.tags(2,2), ...
                        obj.tags(2,3));
    
    obj.estimate_ekf()
    obj.estimate_ukf()
    obj.estimate_ckf()
    obj.tick = obj.tick + 1;
    obj.time = obj.tick / obj.rate;
end