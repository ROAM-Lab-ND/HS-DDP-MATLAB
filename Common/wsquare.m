function s = wsquare(x, W)
x = x(:);
s = x'*W*x;
end