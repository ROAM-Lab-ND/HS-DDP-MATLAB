function K_x = MatrixPartial(K, x)
dim_K = size(K);

K_x = sym(zeros(dim_K(1),dim_K(2),length(x)));
for i = 1:dim_K(1)
    for j = 1:dim_K(2)
        K_x(i,j,:) = jacobian(K(i,j),x);
    end
end
end

