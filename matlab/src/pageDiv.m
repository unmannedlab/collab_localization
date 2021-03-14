function [C] = pageDiv(A,B)
    % Check Sizes
    sz_A = size(A);
    sz_B = size(B);
    if  sz_A(3:end) ~= sz_B(3:end)
        error('Bad Table Dimensions');
    elseif size(A,2) ~= size(B,1)
        error('Col A =/= Row B');
    elseif size(B,1) ~= size(B,2)
        error('Non-Square Matrix');
    end
    
    A = reshape(A, [sz_A(1), sz_A(2), prod(sz_A(3:end))]);
    B = reshape(B, [sz_B(1), sz_B(2), prod(sz_B(3:end))]);
    C = zeros(sz_A(1), sz_B(2), prod(sz_B(3:end)));
    
    for i = 1:prod(sz_B(3:end))
        C(:,:,i) = A(:,:,i) / B(:,:,i);
    end
    
    C = reshape(C, [sz_A(1), sz_B(2), sz_B(3:end)]);
    
end

