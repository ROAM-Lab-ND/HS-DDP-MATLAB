function [G_new,H_new] = impact_aware_step(Px, G_prime,H_prime)
    G_new       = Px'*G_prime;
    H_new       = Px'*H_prime*Px;
end