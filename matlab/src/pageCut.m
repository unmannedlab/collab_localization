function [out] = pageCut(in)
    out = zeros(size(in));
    for i = 1:size(in,3)
        t = tril(in(:,:,i));
        out(:,:,i) = t + t' - diag(diag(in(:,:,i)));
    end
end

