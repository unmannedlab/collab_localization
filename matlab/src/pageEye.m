function [ E ] = pageEye(len, dim)
    n = power(len, dim);
    a = [1, zeros(1, (n-len)/(len - 1))];
    b = [repmat(a, [1, len-1]), 1];
    E = reshape(b, ones(1, dim)*len);
end

