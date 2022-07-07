function [l1, l2, l3, M1, M2, M3, J1, J2, J3, g] = fcn_params(mass)

if mass == false
    l1 = 10*2.54/100;
    l2 = l1;
    l3 = l2;

    M1 = 3.12;
    M2 = 1.29;
    M3 = 1.14;

    J1 = 0.0149;
    J2 = 0.03;
    J3 = 0.0388;

    g = 9.81;

else
    % With the extra mass
    l1 = 10*2.54/100;
    l2 = l1;
    l3 = l2;

    M1 = 3.12;
    M2 = 1.29;
    M3 = 1.64;

    J1 = 0.0149;
    J2 = 0.03;
    J3 = 0.067;

    g = 9.81;
end

