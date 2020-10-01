function obj = draw(obj)

    obj.fh.Visible = 'on';
    axis equal;
    if obj.track_veh
        axis([  obj.cars{1}.x_pos,...
                obj.cars{1}.x_pos,...
                obj.cars{1}.y_pos,...
                obj.cars{1}.y_pos] + obj.axis_lim);
    else 
        axis(obj.axis_lim);
    end

    for i = 1:length(obj.cars)
        car = obj.cars{i};

        % Update Body Position
        RB = [  cos(car.theta), -sin(car.theta);...
                sin(car.theta),  cos(car.theta)];
        A = [[-car.tw,  car.tw,  car.tw, -car.tw,  -car.tw]/2;...
             [ 0,       0,       car.wb,  car.wb,   0     ]];
        RA = RB*A; 
        car.body.XData = RA(1,:)+car.x_pos;
        car.body.YData = RA(2,:)+car.y_pos;

        
        % Update Body Position - CKF
        R_ckf = [   cos(car.ekf_x(3)), -sin(car.ekf_x(3));...
                    sin(car.ekf_x(3)),  cos(car.ekf_x(3))];
        A = [[-car.tw,  car.tw,  car.tw, -car.tw,  -car.tw]/2;...
             [ 0,       0,       car.wb,  car.wb,   0     ]];
        RA = R_ckf*A; 
        car.body_ckf.XData = RA(1,:)+car.ckf_x(1);
        car.body_ckf.YData = RA(2,:)+car.ckf_x(2);     
        
        
        % Update Body Position - EKF
        R_ekf = [   cos(car.ekf_x(3)), -sin(car.ekf_x(3));...
                    sin(car.ekf_x(3)),  cos(car.ekf_x(3))];
        A = [[-car.tw,  car.tw,  car.tw, -car.tw,  -car.tw]/2;...
             [ 0,       0,       car.wb,  car.wb,   0     ]];
        RA = R_ekf*A; 
        car.body_ekf.XData = RA(1,:)+car.ekf_x(1);
        car.body_ekf.YData = RA(2,:)+car.ekf_x(2);     
        
        
        % Update Body Position - UKF
        R_ukf = [   cos(car.ekf_x(3)), -sin(car.ekf_x(3));...
                    sin(car.ekf_x(3)),  cos(car.ekf_x(3))];
        A = [[-car.tw,  car.tw,  car.tw, -car.tw,  -car.tw]/2;...
             [ 0,       0,       car.wb,  car.wb,   0     ]];
        RA = R_ukf*A; 
        car.body_ukf.XData = RA(1,:)+car.ukf_x(1);
        car.body_ukf.YData = RA(2,:)+car.ukf_x(2);     
        
        
        % Wheel Rotation Matrix
        RW =[   cos(car.theta+car.delta), -sin(car.theta+car.delta);...
                sin(car.theta+car.delta),  cos(car.theta+car.delta)];   

        % Update Back Left Wheel Position
        A = [[-0.15, 0.15, 0.15,-0.15,-0.15]-car.tw/2;...
             [-0.35,-0.35, 0.35, 0.35,-0.35]];
        RA = RB*A;
        car.w_bl.XData = RA(1,:) + car.x_pos;
        car.w_bl.YData = RA(2,:) + car.y_pos;

        % Update Back Right Wheel Position
        A = [[-0.15, 0.15, 0.15,-0.15,-0.15]+car.tw/2;...
             [-0.35,-0.35, 0.35, 0.35,-0.35]];
        RA = RB*A;        
        car.w_br.XData = RA(1,:)+car.x_pos;
        car.w_br.YData = RA(2,:)+car.y_pos;


        % Update Front Left Wheel Position
        A  =[[-0.15, 0.15, 0.15,-0.15,-0.15];...
             [-0.35,-0.35, 0.35, 0.35,-0.35]];
        RA = RW*A + RB*[-car.tw/2; car.wb];      
        car.w_fl.XData = RA(1,:)+car.x_pos;
        car.w_fl.YData = RA(2,:)+car.y_pos;  


        % Update Front Right Wheel Position
        A = [[-0.15, 0.15, 0.15,-0.15,-0.15];...
             [-0.35,-0.35, 0.35, 0.35,-0.35]];
        RA = RW*A + RB*[car.tw/2; car.wb];     
        car.w_fr.XData = RA(1,:)+car.x_pos;
        car.w_fr.YData = RA(2,:)+car.y_pos;   
    end
        
    drawnow;
    
end