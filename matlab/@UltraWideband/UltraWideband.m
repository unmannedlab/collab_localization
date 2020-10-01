classdef UltraWideband < handle
    properties
        car_tags    = zeros(2,3);
        
        ckf_states
        ckf_Sigma
        ckf_sigma
        
        lmk_tags    = zeros(3,0);
    end   
    
    methods
        
        function obj = UltraWideband(nCars)
            obj.ckf_states  = zeros(6,nCars);
            obj.ckf_Sigma   = zeros(6,6,nCars);
            obj.ckf_sigma   = repmat(eye(6),[nCars,1]);
        end
        
        function obj = set_ckf(obj, car_num, x, Sigma, sigma)
            obj.ckf_states(:,car_num) = x;
            obj.ckf_Sigma(:,:,car_num) = Sigma;
            obj.ckf_sigma(:,:,car_num) = sigma;
        end
        
        function obj = set_lmk_tag(obj, tag_num, x, y, z)
            obj.lmk_tags(:,tag_num) = [x,y,z];
        end
        
        function obj = set_car_tag(obj, car_num, tag_num, x, y, z)
            obj.car_tags((car_num - 1)*2 + tag_num,:) = [x,y,z];
        end
        
    end
end