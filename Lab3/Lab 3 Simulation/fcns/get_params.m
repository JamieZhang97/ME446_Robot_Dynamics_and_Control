function p = get_params(mass)

% parameters for matrix calculations
[l1, l2, l3, M1, M2, M3, J1, J2, J3, g] = fcn_params(mass);
params = [l1, l2, l3, M1, M2, M3, J1, J2, J3, g];

p.l1 = l1;
p.params = params;
p.N_animate = 30;   % for animation time smoothness
