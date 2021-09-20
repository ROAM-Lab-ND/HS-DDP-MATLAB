function varargout = getFDPar(K,Kx,b,bx,bu)
% This function returns [FDx, FDu, gx, gu] for smooth dynamics and [FDx,
% gx] for reset map which does NOT depend on u
fKKT_x = zeros(size(bx));
for i = 1:size(Kx,3)
    Kinvxi = -K\Kx(:,:,i)/K;
    fKKT_x(:,i) = Kinvxi*b;
end

fKKT_x = fKKT_x + K\bx;
FDx = fKKT_x(1:7, :);
gx = []; gu = [];

if length(K) > 7 % If nontrivial KKT dynamics
    gx = fKKT_x(8:end, :);
end
    
if nargin > 4 % bu appears in smooth dynamics
    
    fKKT_u = K\bu;
        
    FDu = fKKT_u(1:7, :);
        
    if length(K) > 7        
        gu = fKKT_u(8:end, :);
    end
    varargout{1} = FDx;
    varargout{2} = FDu;
    varargout{3} = gx;
    varargout{4} = gu;
else        % u dose not appearh in reset map
    varargout{1} = FDx;
    varargout{2} = gx;
end
end