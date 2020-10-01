function [] = init(obj, scene)
    
    switch scene
        case 'Encircle'
            UWB = UltraWideband(2);
            
            %    num,  x0,  y0,  theta,  delta,   vel,  
            x1 = [2,   10,   0,   0.0,    0.25,   1.0];  
            x2 = [1,  -10,   0,  -0.0,   -0.0,    0.0];
            
            obj.cars = { ...
                KinematicCar(x1, UWB, 2, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x2, UWB, 2, obj.plot_fig, obj.kf_switch)};
            
            obj.axis_lim = [-5 15 -5 15];
            obj.track_veh = false;
            
        case 'Parallel'
            UWB = UltraWideband(2);
            UWB.set_lmk_tag(1, -15, -10,  2);
            UWB.set_lmk_tag(2, -15,   0,  2);
            UWB.set_lmk_tag(3, -15,  10,  2);
            UWB.set_lmk_tag(4, -15,  20,  2);
            
            %    num,  x0,  y0,  theta,  delta,   vel,    
            x1 = [1,   -5,   0,  -0.0,   -0.0,   1.0];
            x2 = [2,    5,   0,   0.0,    0.0,   1.0];

            
            obj.cars = { ...
                KinematicCar(x1, UWB, 2, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x2, UWB, 2, obj.plot_fig, obj.kf_switch)};
            
            obj.axis_lim = [-15 15 -15 15];
            obj.track_veh = true;
            
        case 'Street_Cross'
            UWB = UltraWideband(4);
            UWB.set_lmk_tag(1, -10, -10,  2);
            UWB.set_lmk_tag(2, -10,  10,  2);
            UWB.set_lmk_tag(3,  10, -10,  2);
            UWB.set_lmk_tag(4,  10,  10,  2);
            
            %    num,  x0,  y0,  theta,  delta,  vel,  
            x1 = [2,  15,   2,  pi/2,    0.0,    1.0];  
            x2 = [1,   0, -10,   0.0,    0.0,    0.0];
            x3 = [3,   0,  10,    pi,    0.0,    0.0];
            x4 = [4, -15,  -2, -pi/2,    0.0,    1.0];

            
            obj.cars = { ...
                KinematicCar(x1, UWB, 4, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x2, UWB, 4, obj.plot_fig, obj.kf_switch);...
                KinematicCar(x3, UWB, 4, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x4, UWB, 4, obj.plot_fig, obj.kf_switch)};
            
            obj.axis_lim = [-15 15 -15 15];
            obj.track_veh = false;
            
        case 'Tunnel'
            UWB = UltraWideband(6);
            UWB.set_lmk_tag(1,  10, -50,  2);
            UWB.set_lmk_tag(2,  10,   0,  2);
            UWB.set_lmk_tag(3,  10,  50,  2);
            UWB.set_lmk_tag(4,  10, 100,  2);
            
            %    num,  x0,  y0,  theta,  delta,  vel,   
            x1 = [1,   -2,   0,  -0.0,   -0.0,   5.0];
            x2 = [2,    2,  -5,   0.0,    0.0,   5.0];
            x3 = [3,   -2,  10,  -0.0,   -0.0,   5.0];
            x4 = [4,    2,   5,   0.0,    0.0,   5.0];
            x5 = [5,   -2, -10,  -0.0,   -0.0,   5.0];
            x6 = [6,    2, -15,   0.0,    0.0,   5.0];
            
            obj.cars = { ...
                KinematicCar(x1, UWB, 6, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x2, UWB, 6, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x3, UWB, 6, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x4, UWB, 6, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x5, UWB, 6, obj.plot_fig, obj.kf_switch); ...
                KinematicCar(x6, UWB, 6, obj.plot_fig, obj.kf_switch)};
            
            obj.axis_lim = [-20 20 -20 20];
            obj.track_veh = true;
            
        case 'Landmark'
            UWB = UltraWideband(4);
                       
            UWB.set_lmk_tag(1,  5, -10,  2);
            UWB.set_lmk_tag(2,  5,   0,  2);
            UWB.set_lmk_tag(3,  5,  10,  2);
            UWB.set_lmk_tag(4,  5,  20,  2);
            
            %    num,  x0,  y0,  theta,  delta,   vel,    
            x1 = [1,   0,   0,   0.0,    0.0,    1.0];
            obj.cars = {KinematicCar(x1, UWB, obj.plot_fig, obj.kf_switch)};
            
        otherwise
            error('Sim Error: Not a valid scene')
    end
    
    obj.tri_lmk = cell(size(UWB.lmk_tags,1),1);
    for i = 1:size(UWB.lmk_tags,2)
        obj.tri_lmk{i} = triangle(UWB.lmk_tags(1,i),UWB.lmk_tags(2,i));
    end
end

function ps = triangle(x,y)
    a = 0.5;
    h = sqrt(3)/2*a;
    ps = line(  [-a/2, a/2,  0,     -a/2] + x,...
                [-h/3, -h/3, 2*h/3, -h/3] + y);
end