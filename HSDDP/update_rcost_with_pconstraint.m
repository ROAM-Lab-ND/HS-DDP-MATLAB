function [l, lpar] = update_rcost_with_pconstraint(l, lpar, pConstr, params, dt)
% l: running cost
% lpar: derivative of running cost
% pConstr: 1xn structure array of path constraints
% params:  1xn structure array of ReB parameters
n = length(pConstr);
assert(n==length(params), 'Path constraints and ReB params have different size');
for i = 1:n
    eps = dt * (params(i).eps);
    delta = params(i).delta;
    c = pConstr(i).c;
    cx = pConstr(i).cx(:);
    cy = pConstr(i).cy(:);
    cu = pConstr(i).cu(:);
    cxx = pConstr(i).cxx;
    cuu = pConstr(i).cuu;
    cyy = pConstr(i).cyy;   
    
    [B, Bc, Bcc] = ReducedBarrier(c, delta);
    l = l + eps*B;
    lpar.lx = lpar.lx + eps * Bc * cx; % cx is column vector
    lpar.lu = lpar.lu + eps * Bc * cu;
    lpar.ly = lpar.ly + eps * Bc * cy;
    
    lpar.lxx = lpar.lxx + eps * (Bcc*(cx*cx') + Bc*cxx);
    lpar.luu = lpar.luu + eps * (Bcc*(cu*cu') + Bc*cuu);
    lpar.lyy = lpar.lyy + eps * (Bcc*(cy*cy') + Bc*cyy);
end
end