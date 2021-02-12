function out = setdiff2(~, n, a, b)
    if b ==0
        out = [1:a-1,a+1:n];
    elseif a < b
        out = [1:a-1,a+1:b-1,b+1:n];
    else
        out = [1:b-1,b+1:a-1,a+1:n];
    end
end