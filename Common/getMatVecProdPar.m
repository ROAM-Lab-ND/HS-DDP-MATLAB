function Abx = getMatVecProdPar(A,Ax,b,bx)
% This function computes partial of A(x)b(x) w.r.t. x 
if isempty(A) || isempty(Ax) || isempty(b) || isempty(bx)
    Abx = [];
    return;
end
Abx = zeros(size(A,1), size(Ax, 3));
for i = 1:size(Ax, 3)
    Abx(:,i) = Ax(:,:,i)*b;
end
Abx = Abx + A*bx;