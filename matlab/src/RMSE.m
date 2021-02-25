function err = RMSE(x)
    m = size(x,1);
    n = size(x,3);
    
    N = size(x,2) * size(x,4);
    err = zeros(m, n);
    
    for i = 1:m
        for j = 1:n
            err(i,j) = sqrt( sum(x(i,:,j,:).^2,[2,4]) / N);
        end
    end
end