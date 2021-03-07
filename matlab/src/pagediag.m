function [B] = pagediag(A, vargin)
    if length(vargin) == 1
        error('Not enough inputs');
    elseif length(vargin) == 2
        d1 = vargin(1);
        d2 = vargin(2);
    elseif length(vargin) == 3
        d1 = vargin(1);
        d2 = vargin(2);
        d3 = vargin(3);       
    else
        error('Too many inputs');
    end
    
%     if size(A, dim1) ~= size(A, dim2)
%         error('yo thats a bad size'); 
%     end
%     if abs(dim1-dim2)>1
%         error('must use sequential dimensions')
%     end
%     if dim1==dim2
%         error('sizes must be different')
%     end
    
    new_size = size(A);
    new_size(min(dim1,dim2)) = 1;
    
%     dot_size = size(A);
%     dot_size(dim1) = 1;
%     dot_size(dim2) = 1;
    
    B = zeros(new_size);
%     C = repmat(eye(size(A, dim1)), dot_size);
%     B = pagemtimes(ones(new_size), A.*C);
%     B = reshape(B,new_size(dim2:end));
    
%     Add dot product of sorts here using pagemtimes
end
