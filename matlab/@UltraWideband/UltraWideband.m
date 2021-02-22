classdef UltraWideband < handle
    properties
        car_tags    = zeros(2,3);
        
        dcl_states
        dcl_Sigma
        dcl_sigma
        
        dc2_states
        dc2_Sigma
        dc2_sigma
        
        dcl_lmk_states
        dcl_lmk_Sigma
        dcl_lmk_sigma
        
        lmk_tags    = zeros(3,0);
    end   
    
    methods
        
        function obj = UltraWideband(nCars)
            obj.dcl_states  = zeros(6,nCars);
            obj.dcl_Sigma   = zeros(6,6,nCars);
            obj.dcl_sigma   = repmat(eye(6),[nCars,1]);
            
            obj.dcl_lmk_states  = zeros(6,nCars);
            obj.dcl_lmk_Sigma   = zeros(6,6,nCars);
            obj.dcl_lmk_sigma   = repmat(eye(6),[nCars,1]);
        end
        
        function obj = set_dcl(obj, car_num, x, Sigma, sigma)
            obj.dcl_states(:, car_num) = x;
            obj.dcl_Sigma(:,:,car_num) = Sigma;
            obj.dcl_sigma(:,:,car_num) = sigma;
        end
        
        function obj = set_dc2(obj, car_num, x, Sigma, sigma)
            obj.dc2_states(:, car_num) = x;
            obj.dc2_Sigma(:,:,car_num) = Sigma;
            obj.dc2_sigma(:,:,car_num) = sigma;
        end
        
        function obj = set_dcl_lmk(obj, car_num, x, Sigma, sigma)
            obj.dcl_lmk_states(:,car_num)  = x;
            obj.dcl_lmk_Sigma(:,:,car_num) = Sigma;
            obj.dcl_lmk_sigma(:,:,car_num) = sigma;
        end
        
        function obj = set_lmk_tag(obj, tag_num, x, y, z)
            obj.lmk_tags(:,tag_num) = [x,y,z];
        end
        
        function obj = set_car_tag(obj, car_num, tag_num, x, y, z)
            obj.car_tags((car_num - 1)*2 + tag_num,:) = [x,y,z];
        end
        
    end
end