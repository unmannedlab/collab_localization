function [out] = pageChol(in)
    out = zeros(size(in));
    for i = 1:size(in,3)
        try out(:,:,i) = chol(in(:,:,i));
        catch
            disp('Matrix is not symmetric positive definite')
        end
        
    end
end

