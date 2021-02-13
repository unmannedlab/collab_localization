
a = 2;
b = 3;
c = 10^6;

v = 1:b;
tic 
for i = 1:c
    for j = 1:a
        v + normrnd(0,1,[1,b]).*v;
    end
end
toc 


tic
    
repmat(v,[a,1,c]) + normrnd(0,1,[a,b,c]).*repmat(v,[a,1,c]);

toc