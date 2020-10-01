classdef KinematicCar < handle
    properties
        car_num
        x_pos
        y_pos
        theta
        delta
        vel
        accel
        
        wb  = 4
        tw  = 2
        beta
        gamma
        
        gps_cep = 1.0
        gps_her = 0.005
        gps_ver = 0.05
        enc_err = 0.05
        str_err = 0.05
        uwb_err = 0.10
        
        imu_acc_err = 0.1;
        imu_gyr_err = 0.1;
        imu_mag_err = 0.1;
        
        rate_imu = 50;
        rate_mdl = 30;
        rate_gps = 10;
        rate_uwb =  3;
        
        rate
        dt 
          
        ekf_x
        ekf_cov  
        ukf_x
        ukf_cov  
        
        ckf_x
        ckf_Sigma
        ckf_sigma
        ckf_init 
        
        uwb
        anchors
        tags
        tag_offsets     

        tick = 0;
        time
        body
        w_bl
        w_br
        w_fl
        w_fr
        body_ckf
        body_ekf
        body_ukf
        fh
        
        nCars = 1;
        
        kf_mdl = true;
        kf_gps = true;
        kf_uwb = true;
    end
    
    
    methods
        function obj = KinematicCar(x_0, varargin)
            
            p = inputParser;
            addRequired(p,'UWB_Object')
            addRequired(p,'nCars')
            addOptional(p,'plot_Fig', true);
            addOptional(p,'kf_switch', [1,1,1]);
            addOptional(p,'perfect', false);
            parse(p,varargin{:});
            
            obj.uwb    = p.Results.UWB_Object;
            obj.nCars  = p.Results.nCars;
            obj.kf_mdl = p.Results.kf_switch(1);
            obj.kf_gps = p.Results.kf_switch(2);
            obj.kf_uwb = p.Results.kf_switch(3);
            
            if p.Results.perfect
                obj.gps_cep = 1e-12;
                obj.gps_her = 1e-12;
                obj.gps_ver = 1e-12;
                obj.enc_err = 1e-12;
                obj.str_err = 1e-12;
                obj.uwb_err = 1e-12;

                obj.imu_acc_err = 1e-12;
                obj.imu_gyr_err = 1e-12;
                obj.imu_mag_err = 1e-12;
            end


            obj.car_num = x_0(1);
            obj.x_pos   = x_0(2);
            obj.y_pos   = x_0(3);
            obj.theta   = x_0(4);
            obj.delta   = x_0(5);
            obj.vel     = x_0(6);
            obj.accel   = 0;
            obj.ekf_x    = [ x_0(2); 
                            x_0(3); 
                            x_0(4);... 
                           -x_0(6)*sin(x_0(4)); 
                            x_0(6)*cos(x_0(4));
                            obj.vel*tan(obj.delta)/obj.wb];
                    
            obj.ekf_cov = [ ...
                    2.0e-2, -1.0e-5,  0.0e-0,  4.0e-4, -7.0e-5,  0.0e-0; ...
                   -1.0e-5,  2.0e-2,  0.0e-0, -7.0e-5,  3.0e-4,  0.0e-0;...
                    0.0e-0,  0.0e-0,  3.0e-3,  0.0e-0,  0.0e-0,  7.0e-5;...
                    4.0e-4, -7.0e-5,  0.0e-0,  1.0e-2, -1.0e-3,  0.0e-0;...
                   -7.0e-5,  3.0e-4,  0.0e-0, -1.0e-3,  1.0e-2,  0.0e-0;...
                    0.0e-0,  0.0e-0,  7.0e-5,  0.0e-0,  0.0e-0,  1.0e-2];
            
            obj.ckf_x   = obj.ekf_x;
            obj.ckf_sigma = repmat(eye(6),[obj.nCars,1]);
            obj.ckf_Sigma = zeros(obj.nCars*6);
            n = (obj.car_num-1)*6+1;
            obj.ckf_Sigma(n:n+5,n:n+5) = obj.ekf_cov;
            obj.ckf_init = false(obj.nCars,1);
                    
            obj.ukf_x   = obj.ekf_x;
            obj.ukf_cov = obj.ekf_cov;
                
            obj.rate =  lcm(obj.rate_imu, ...
                        lcm(obj.rate_mdl, ...
                        lcm(obj.rate_gps,obj.rate_uwb)));
            obj.dt   = 1/obj.rate;
            
            obj.tag_offsets = [ -obj.tw / 2, obj.wb, 0.2;...
                                 obj.tw / 2, 0     , 1.8];  

            obj.beta = atan2(obj.tw, obj.wb);
            obj.gamma = atan2(1.6, sqrt(obj.tw^2 + obj.wb^2));
            
            if p.Results.plot_Fig
                Carray = get(gca,'colororder');
                obj.body = line(0,0,'Color',Carray(obj.car_num, :));
                obj.body_ekf = line(0,0,'Color',Carray(obj.car_num, :),'LineStyle','--');
                obj.body_ukf = line(0,0,'Color',Carray(obj.car_num, :),'LineStyle',':');
                obj.body_ckf = line(0,0,'Color',Carray(obj.car_num, :),'LineStyle','-.');
                obj.w_bl = line(0,0,'Color','k');
                obj.w_br = line(0,0,'Color','k');
                obj.w_fl = line(0,0,'Color','k');
                obj.w_fr = line(0,0,'Color','k');
            end
        end
        
        function obj = update_accel(obj, accel)
            obj.accel = accel;
        end
        
        function obj = update_steer(obj, delta)
            obj.delta = delta;
        end
        
        function obj = update_kf_switches(obj, mdl, gps, uwb)
            obj.kf_mdl = mdl;
            obj.kf_gps = gps;
            obj.kf_uwb = uwb;
        end
    end
end