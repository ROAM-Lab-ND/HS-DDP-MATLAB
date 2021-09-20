function [phi, phi_par] = update_tcost_with_tconstraint(phi, phi_par, tConstr, params)
% tConstr: 1xn structure array of equality constraints at one time instant
% params:  1xn structure array of augmented lagrangian parameters

n = length(tConstr);
assert(n==length(params), 'Terminal constraint and AL params have different size');
for i = 1:n
    sigma = params(i).sigma;
    lambda = params(i).lambda;
    h = tConstr(i).h;
    hx = tConstr(i).hx;
    hxx = tConstr(i).hxx;
    
    phi = phi + (sigma/2)^2 * (h)^2 + lambda*h;
    
    phi_par.G = phi_par.G + 2*(sigma/2)^2*h*hx + lambda*hx;
    phi_par.H = phi_par.H + ...
                2*(sigma/2)^2*(h*hxx + hx*hx') + ...
                lambda*hxx;
end
end