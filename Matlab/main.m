g = 9.81;
m = 0.08;
R = 0.05;
J_w = (m*R^2)/2;
M = 0.77;
W = 0.12;
H = 0.16;
D = 0.04;
L = H/2;
J_psi = (M*L^2)/3;
J_phi = M*(W^2 + D^2)/12;
J_m = 1e-5;
R_m = 6.69;
K_b = 0.47;
K_t = 0.32;
n = 1;
f_m = 0.0022;
f_w = 0;
alpha = (n*K_t)/R_m;
beta = n*K_t*K_b/R_m + f_m;


E = [(2*m + M)*R^2 + 2*J_w + 2*n^2*J_m  M*L*R - 2*n^2*J_m;
     M*L*R - 2*n^2*J_m  M*L^2 + J_psi + 2*n^2*J_m];
F = [2*beta + 2*f_w  -2*beta; 
     -2*beta  2*beta];
G = [0  0; 
     0  -M*g*L];
H = [alpha  alpha; 
     -alpha  -alpha];

I = m*W^2/2 + J_phi + W^2/(2*R^2)*(J_w + n^2*J_m);
J = W^2/(2*R^2)*(beta + f_w);
K = W/(2*R)*alpha;

A_1(3,2) = -g*M*L*E(1,2)/det(E);
A_1(3,3) = -2*((beta + f_w)*E(2,2)+ beta*E(1,2))/det(E);
A_1(3,4) = 2*beta*(E(1,2) + E(2,2))/det(E);
A_1(4,2) = -g*M*L*E(1,1)/det(E);
A_1(4,3) = 2*((beta + f_w)*E(1,2) + beta*E(1,1))/det(E);
A_1(4,4) = -2*beta*(E(1,1) + E(1,2))/det(E);

A_1 = [0 0 1 0;
       0 0 0 1; 
       0 A_1(3,2) A_1(3,3) A_1(3,4); 
       0 A_1(4,2) A_1(4,3) A_1(4,4)];

B_1(3) = alpha*(E(1,2) + E(2,2))/det(E);
B_1(4) = -alpha*(E(1,1) + E(1,2))/det(E);
  
B_1 = [0 0;
       0 0;
       B_1(3) B_1(3);
       B_1(4) B_1(4)];
   
A_2 = [0 1;
       0 -J/I];

B_2 = [0 0;
       -K/I K/I];
   
Q_1 = eye(4);
Q_2 = eye(2);
R_1 = 1;
R_2 = 1;

[K_1, P_1, E_1] = lqr(A_1, B_1, Q_1, R_1)
[K_2, P_2, E_2] = lqr(A_2, B_2, Q_2, R_2);


