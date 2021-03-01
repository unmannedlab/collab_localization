function [t] = bcombs(nCars)
    persistent t_prev n_prev
    if isempty(t_prev) || n_prev ~= nCars
        t = zeros(nchoosek(nCars, 2), nCars);
        m = 1;
        n = 2;
        for i = 1:size(t,1)
            t(i,m) =  1;
            t(i,n) = -1;
            n = n + 1;
            if n > size(t,2)
                m = m + 1;
                n = m + 1;
            end
        end
        t_prev = t;
        n_prev = n;
    else    
        t = t_prev;
    end
end