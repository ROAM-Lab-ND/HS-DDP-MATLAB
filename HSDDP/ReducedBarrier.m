function varargout = ReducedBarrier(z, delta)
% ReB Reduced barrier function penalizing z > = 0
% delta relaxation parameter
% Bz, Bzz Derivative of B w.r.t. z
k = 2;      % second order polynominal extension
if z>delta 
    B  = -log(z);
    Bz  = -z ^(-1);
    Bzz  = z ^(-2);
else
    B  = (k-1)/k*(((z -k*delta )/((k-1)*delta ))^k - 1) - log(delta );
    Bz  =  ((z -k*delta )/((k-1)*delta ))^(k-1)/delta ;
    Bzz  = ((z -k*delta )/((k-1)*delta ))^(k-2);
end
if nargout >= 1
    varargout{1} = B;
end
if nargout >= 2
    varargout{2} = Bz;  
end
if nargout == 3
    varargout{3} = Bzz;
end
end
