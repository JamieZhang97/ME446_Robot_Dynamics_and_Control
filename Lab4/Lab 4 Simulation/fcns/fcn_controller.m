function u = fcn_controller(X,sd,p)

params = p.params;
flag_ctrl = p.flag_ctrl;

q = X(1:3);
dq = X(4:6);

xyz = fcn_p4(q,params);
J4 = fcn_J4(q,params);
xyzdot = J4 * dq;

% desired trajectory
xyz_d = sd(1:3);
xyzdot_d = sd(4:6);

if flag_ctrl == 0 || flag_ctrl == 3  % Task Space PD control and Straight Line following without weakening 
    %TODO: Design task Space PD controller
    Kp = zeros(3,3);
    % Kp limit from 1000 to 100000
    Kp(1,1) = 4000; 
    Kp(2,2) = 6000;
    Kp(3,3) = 4000;
    
    % Kd limit from 100 to 1000
    Kd = zeros(3,3);
    Kd(1,1) = 300;
    Kd(2,2) = 500;
    Kd(3,3) = 300;
    
    % task space PD control law
    u = J4.' * (Kp * (xyz_d - xyz) + Kd*(xyzdot_d - xyzdot));

elseif flag_ctrl == 1 % Task Space Impedence control (World Frame) 
    %TODO: Design task Space Impednece controller
    % When weaken Y or XY, mostly lessen the kp to let it can bear large
    % position error, and partly lessen the kd, to let it move within a
    % proper speed
    Kp = zeros(3,3);
    Kp(1,1) = 50;
    Kp(2,2) = 50;
    Kp(3,3) = 3000;
    
    Kd = zeros(3,3);
    Kd(1,1) = 100;
    Kd(2,2) = 100;
    Kd(3,3) = 300;

    u = J4.' * (Kp * (xyz_d - xyz) + Kd*(xyzdot_d - xyzdot));

    
elseif flag_ctrl == 2 % Task Space PD control (Rotation) 
    %TODO: Design task Space Impednece controller (Rotation)
    Kp = zeros(3,3);
    Kp(1,1) = 19000;
    Kp(2,2) = 19000;
    Kp(3,3) = 19000;
    
    Kd = zeros(3,3);
    Kd(1,1) = 900;
    Kd(2,2) = 900;
    Kd(3,3) = 900;
    
    % 1st rotation matrix
    % R_zxy = [0.5 -0.866 0; 0.866 0.5 0; 0 0 1]; 
    % 2nd rotation matrix
    R_zxy = [0.5 0 0.866; 0.433 0.866 -0.25; -0.75 0.5 0.433];
    
    
    u = J4.' * R_zxy * (Kp * R_zxy.' * (xyz_d - xyz) + Kd*R_zxy.' * (xyzdot_d - xyzdot));

elseif p.flag_ctrl == 4 % (Rotation) % Staright Line Following (Rotation)
    %TODO: Design task Space Impednece controller (Rotation)
    Kp = zeros(3,3);
    Kp(1,1) = 9000;
    Kp(2,2) = 1000;
    Kp(3,3) = 9000;
    
    Kd = zeros(3,3);
    Kd(1,1) = 900;
    Kd(2,2) = 100;
    Kd(3,3) = 900;
    % rotate the original frame along z for:-(pi/2 + acctan(1/5))
    R_zxy = [-0.196 0.981 0; 0.981 -0.196 0; 0 0 1];
    
    
    u = J4.' * R_zxy * (Kp * R_zxy.' * (xyz_d - xyz) + Kd*R_zxy.' * (xyzdot_d - xyzdot));



else
    u = [0 0 0]';

end

%% control satuation
umax = 100;
u(u>umax) = umax;
u(u<-umax) = -umax;


end
