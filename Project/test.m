theta_dot = 1; 
theta_ddot = 1; 
psi = 1; 
psi_dot = 1; 
psi_ddot = 1; 
phi_dot = 1; 
phi_ddot = 1;


p_ss = 1.05;

g = 9.81;
m = 0.08;
R = 0.05;
M = 0.77;
W = 0.12;
H = 0.16;
D = 0.04;

L = H/2;
J_w = (m*R^2)/2;
J_psi = (M*L^2)/3;
J_phi = M*(W^2 + D^2)/12;
J_m = 1e-5;
R_m = 6.69;
K_b = 0.47;
K_t = 0.32;
n = 1;
f_m = 0.0022;
f_w = 0;

m_ss = m*p_ss;
R_ss = R*p_ss;
M_ss = M*p_ss;
W_ss = W*p_ss;

% theta_dot = (theta_l_dot + theta_r_dot)/2;
% phi_dot = R_ss/W_ss*(theta_r_dot - theta_l_dot);

theta_l_dot = theta_dot - W_ss*phi_dot/(2*R_ss);
theta_r_dot = theta_dot + W_ss*phi_dot/(2*R_ss);

F_theta = ((2*m_ss+M_ss)*R_ss^2 + 2*J_w + 2*n^2*J_m)*theta_ddot + (M_ss*L*R_ss*cos(psi) - 2*n^2*J_m)*psi_ddot - M_ss*L*R_ss*psi_dot^2*sin(psi);
F_psi = (M_ss*L*R_ss*cos(psi) - 2*n^2*J_m)*theta_ddot + (M_ss*L^2 + J_psi + 2*n^2*J_m)*psi_ddot - M_ss*g*L*sin(psi) - M_ss*L^2*phi_dot^2*sin(psi)*cos(psi);
F_phi = (m_ss*W_ss^2/2 + J_phi + W_ss^2/(2*R_ss^2)*(J_w + n^2*J_m) + M_ss*L^2*(sin(psi)^2))*phi_ddot + 2*M_ss*L^2*psi_dot*phi_dot*sin(psi)*cos(psi);

% F_theta = F_l + F_r;
% F_phi = W_ss/(2*R_ss)*(F_r - F_l);

F_l = F_theta/2 - R_ss*F_phi/W_ss;
F_r = F_theta/2 + R_ss*F_phi/W_ss;

i_l = (F_l + f_w*theta_l_dot - f_m*(psi_dot - theta_l_dot))/(n*K_t); 
i_r = (F_r + f_w*theta_r_dot - f_m*(psi_dot - theta_r_dot))/(n*K_t); 

% F_l = n*K_t*i_r + f_m*(psi_dot - theta_r_dot) - f_w*theta_r_dot; 
% F_r = n*K_t*i_r + f_m*(psi_dot - theta_r_dot) - f_w*theta_r_dot; 
% F_psi = -n*K_t*i_l - n*K_t*i_r - f_m*(psi_dot - theta_l_dot) - f_m*(psi_dot - theta_r_dot);

v_l = K_b*(psi_dot - theta_l_dot) - i_l*R_m
v_r = K_b*(psi_dot - theta_r_dot) - i_r*R_m