function u = fcn_controller(X,desired_traj,p)

params = p.params;
flag_ctrl = p.flag_ctrl;

q = X(1:3);
dq = X(4:6);


% Added the "forController" just to make it clear that these are the D,C and G that are being calculated in your control code and in an
% actual robot we would not know the exact D,C and G.  That is also why I added some perturbations to the Mass and Inertias because
% if they were exactly the same as the model we would not need the outer loop control at all.  
%params = [l1, l2, l3, M1, M2, M3, J1, J2, J3, g];
params(4) = params(4)*0.95; %M1
params(5) = params(5)*1.04; %M2
params(6) = params(6)*0.92; %M3
params(7) = params(7)*0.9;  %J1
params(8) = params(8)*1.06; %J2
params(9) = params(9)*0.93; %J3
DforController = fcn_De(q,params);
CforController = fcn_Ce(q,dq,params);
GforController = fcn_Ge(q,params);
BforController = fcn_Be(q,params);


if flag_ctrl == 0 || flag_ctrl == 2 
    qd = desired_traj;
elseif flag_ctrl == 1 || flag_ctrl == 3 || flag_ctrl == 4 || p.flag_ctrl == 5 || p.flag_ctrl == 6 || p.flag_ctrl == 7 
    qd = desired_traj(1:3);
    dqd = desired_traj(4:6);
    ddqd = desired_traj(7:9);
end

    
if flag_ctrl == 0 || flag_ctrl == 2% joint space PD control
    
    Kp = zeros(3,3);
    Kp(1,1) = 100;
    Kp(2,2) = 200;
    Kp(3,3) = 100;
    
    Kd = zeros(3,3);
    Kd(1,1) = 8;
    Kd(2,2) = 16;
    Kd(3,3) = 5;
    
    u = Kp * (qd - q) - Kd * (dq);
    
elseif flag_ctrl == 1  % Feed Forward + PD control

    J = diag([0.0149 0.03 0.0388]);

    Kp = zeros(3,3);
    Kp(1,1) = 200;
    Kp(2,2) = 200;
    Kp(3,3) = 200;
    
    Kd = zeros(3,3);
    Kd(1,1) = 3;
    Kd(2,2) = 3;
    Kd(3,3) = 3;
    
    u = J * ddqd + Kp * (qd - q) + Kd * (dqd - dq);

elseif flag_ctrl == 3 || flag_ctrl == 4 %  Inverse Dynamics control
    % TODO: Create your inverse dynamics controller here    
    
    Kp = zeros(3,3);
    Kp(1,1) = 100;
    Kp(2,2) = 100;
    Kp(3,3) = 100;
    
    Kd = zeros(3,3);
    Kd(1,1) = 1;
    Kd(2,2) = 1;
    Kd(3,3) = 1;
    
    a_theta = Kp*(qd - q) + Kd * (dqd - dq) + ddqd;
    
    u = DforController*a_theta + CforController*dq + GforController;
    
    
elseif p.flag_ctrl == 5 % joint space PD plus feedforward + Fast Trajectory
    % TODO: Tune the gains of PD controller for the fast trajectory
    J = diag([0.0149 0.03 0.0388]);

    Kp = zeros(3,3);
    Kp(1,1) = 200;
    Kp(2,2) = 300;
    Kp(3,3) = 200;
    
    Kd = zeros(3,3);
    Kd(1,1) = 4;
    Kd(2,2) = 12;
    Kd(3,3) = 6;
    
    u = J * ddqd + Kp * (qd - q) + Kd * (dqd - dq);
    
else
    u = [0 0 0]';

end

%% control satuation
umax = 10;
u(u>umax) = umax;
u(u<-umax) = -umax;


end