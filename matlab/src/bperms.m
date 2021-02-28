function [t] = bperms(n)
    dims = repmat({[false, true]}, 1, n);
    [grids{1:n}] = ndgrid(dims{:});
    grids = cellfun(@(g) reshape(g,[],1), grids, 'UniformOutput',false);
    t = [grids{:}];

end


