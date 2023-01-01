function [out] = convolve(a, b)
n = length(a);
m = length(b);

if m==0 || n==0
    out = 0;
    return
end

out = [];
a = [a , zeros(1, m-1)];
b = [b , zeros(1, n-1)];
b = b(m+n-1:-1:1);

for i=m+n-1:-1:1
    out = [sum(a.*b) out];
    b = [b(2:end) 0];
end

end

