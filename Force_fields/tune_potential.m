function [R, U_0] = tune_potential(d_c, d_inf, U_c, U_inf)
R   = (d_inf - d_c) / log(U_c/U_inf);
U_0 = U_inf * exp(d_inf/R);