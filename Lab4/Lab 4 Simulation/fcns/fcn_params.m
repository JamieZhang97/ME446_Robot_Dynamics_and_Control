function [l1, l2, l3, M1, M2, M3, J1, J2, J3, g] = fcn_params()
% l1,l2,l3 Length of link 1-3
% M1,M2,M3 Mass of link 1-3
% J1,J2,J3 Moment of Inertia of Link 1-3
% g Gravity constant
l1 = 10*2.54/100; % unit: meter
l2 = l1;
l3 = l2;

M1 = 1; % unit: Kilogram
M2 = 0.8;
M3 = 0.6;

J1 = 1e-2; 
J2 = 5e-1;
J3 = 1e-2;

g = 9.81;

