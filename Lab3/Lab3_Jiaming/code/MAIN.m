% 3-link manipulator simulation
% Author: Yanran Ding, Mengchao Zhang
% Last modified: 2020/3/17

clear all
close all
clc

addpath gen
addpath fcns

% --- parameters ---
% set "mass = true" if you are planning to use controller 6 or 7
mass = true;
p = get_params(mass);

% flag_ctrl:
% 0 - Joint Space PD Control
% 1 - Joint Space Feed Forward + PD
% 2 - Fun Trajectory (Joint Space PD)
% 3 - Inverse Dynamics control
% 4 - Inverse Dynamics control + Fast Trajectory
% 5 - Joint Space PD plus Feedforward + Fast Trajectory
% 6 - Joint Space PD plus Feedforward + Added Mass
% 7 - Inverse Dynamics control + Added Mass
p.flag_ctrl = 4;

% initial condition
if p.flag_ctrl == 2
    % Fun Trajectory.
    x = 15+4*cos(0); 
    y = 4*sin(0); 
    z = 10;

    % Inverse Kinematic
    q1 = atan2(y,x); 
    q3 = pi - acos((200-(x^2+y^2)-(z-10)^2)/200); 
    q2 = -(q3/2 + atan2(z-10,sqrt(x^2+y^2)));

    q0 = [q1;q2;q3];    
else
    q0 = [0;-1/2*pi;1/2*pi]; 
end
dq0 = [0;0;0];
ic = [q0;dq0];

% plotRobot(ic,p);

% recording
tstart = 0;
tfinal = 6;

tout = tstart;
Xout = ic';

% simulate
[t,X] = ode45(@(t,X)dyn_manip(t,X,p),[tstart, tfinal], Xout(end,:));

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
tstart = tout(end);

% reconstruct
[traj_d,uout] = fcn_reconstruct(tout,Xout,p);


%% visualize
p.flag_movie = 1;
%animateRobot(tout,Xout,traj_d,uout,p)

figure(1);
subplot(2,2,1);
plot(tout,traj_d(:,1),tout,Xout(:,1));
title('Theta1');
subplot(2,2,2);
plot(tout,traj_d(:,2),tout,Xout(:,2));
title('Theta2');
subplot(2,2,3);
plot(tout,traj_d(:,3),tout,Xout(:,3));
title('Theta3');
subplot(2,2,4);
plot(tout,uout(:,1),tout,uout(:,2),tout,uout(:,3));
title('Torques');

figure(2);
subplot(2,2,1);
plot(tout,traj_d(:,1)-Xout(:,1));
title('Theta1 error');
subplot(2,2,2);
plot(tout,traj_d(:,2)-Xout(:,2));
title('Theta2 error');
subplot(2,2,3);
plot(tout,traj_d(:,3)-Xout(:,3));
title('Theta3 error');
subplot(2,2,4);
plot(tout,uout(:,1),tout,uout(:,2),tout,uout(:,3));
title('Torques');















