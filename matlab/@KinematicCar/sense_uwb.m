function [ ckf_dist, lmk_dist] = sense_uwb( obj )
%  EXAMPLE OUTPUT CAR 1: 
% --------Tag_1---Tag_2
% -Tag_1:   0      d12 
% -Tag_2:  d21      0 
% -Tag_3:  d31     d32 
% - ...    ...     ... 
    
    ckf_dist = zeros(length(obj.uwb.car_tags(:,1)), 2);
    lmk_dist = zeros(size(obj.uwb.lmk_tags,2), 2);
    
    for i = 1:length(ckf_dist(:,1))
        for j = 1:2    
            
            tag_num = (obj.car_num - 1) * 2 + j;
            
            if i == tag_num
                ckf_dist(i,j) = 0;
            else 
                r = norm([  obj.uwb.car_tags(i,1)-obj.uwb.car_tags(tag_num,1), ...
                            obj.uwb.car_tags(i,2)-obj.uwb.car_tags(tag_num,2), ...
                            obj.uwb.car_tags(i,3)-obj.uwb.car_tags(tag_num,3)], 2);
                        
                if r > 100
                    ckf_dist(i,j) = Inf; 
                else 
                    ckf_dist(i,j) = normrnd(r, obj.uwb_err);
                end
            end                    
        end
    end
    
    for i = 1:length(lmk_dist(:,1))
        for j = 1:2    
            tag_num = (obj.car_num - 1) * 2 + j;
            
            r = norm([  obj.uwb.lmk_tags(1,i)-obj.uwb.car_tags(tag_num,1), ...
                        obj.uwb.lmk_tags(2,i)-obj.uwb.car_tags(tag_num,2), ...
                        obj.uwb.lmk_tags(3,i)-obj.uwb.car_tags(tag_num,3)], 2);

            if r > 100
                lmk_dist(i,j) = Inf; 
            else 
                lmk_dist(i,j) = normrnd(r, obj.uwb_err);
            end
        end
    end
end

