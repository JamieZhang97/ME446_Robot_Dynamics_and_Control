% 3-link manipulator simulation
% Author: Yanran Ding and Mengchao Zhang
% Last modified: 2020/3/16

clear all
close all
clc

addpath gen
addpath fcns

% --- parameters ---
% Get the parameters about the robot and simulation settings
p = get_params();


% flag_ctrl:
% 0 - Task Space PD control
% 1 - Task Space Impedence control (one or two axes weakened)
% 2 - Task Space Impedence control (with Rotation)
% 3 - Straight Line Following 
% 4 - Straight Line Following (with Rotation)
p.flag_ctrl = 2;

% initial condition for flag_ctrl 1
if p.flag_ctrl == 0 || p.flag_ctrl == 1 || p.flag_ctrl == 2 
    x = 0.381;
    y = 0;
    z = 0.254;
    q1 = atan2(y,x); 
    q3 = pi - acos((0.129-(x^2+y^2)-(z-0.254)^2)/0.129); 
    q2 = -(q3/2 + atan2(z-0.254,sqrt(x^2+y^2)));
    q0 = [q1;q2;q3]; 
elseif p.flag_ctrl == 3
    start_point = [0.35,-0.2,0.25];
    x = start_point(1);
    y = start_point(2);
    z = start_point(3);
    q1 = atan2(y,x); 
    q3 = pi - acos((0.129-(x^2+y^2)-(z-0.254)^2)/0.129); 
    q2 = -(q3/2 + atan2(z-0.254,sqrt(x^2+y^2)));
    q0 = [q1;q2;q3]; 
elseif p.flag_ctrl == 4
    start_point = [0.35,-0.2,0.25];
    x = start_point(1);
    y = start_point(2);
    z = start_point(3);
    q1 = atan2(y,x); 
    q3 = pi - acos((0.129-(x^2+y^2)-(z-0.254)^2)/0.129); 
    q2 = -(q3/2 + atan2(z-0.254,sqrt(x^2+y^2)));
    q0 = [q1;q2;q3];     
end

dq0 = [0;0;0];
ic = [q0;dq0];


% plotRobot(ic,p);

% recording
tstart = 0;
if p.flag_ctrl == 3 || p.flag_ctrl == 4
    tfinal = 2;
elseif p.flag_ctrl == 0 || p.flag_ctrl == 1 || p.flag_ctrl == 2 
    tfinal = 5;
end

tout = tstart;
Xout = ic';

% simulate
[t,X] = ode45(@(t,X)dyn_manip(t,X,p),[tstart, tfinal], Xout(end,:));

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
tstart = tout(end);

% reconstruct
[xyz_world,traj_d,uout] = fcn_reconstruct(tout,Xout,p);


%% visualize
p.flag_movie = 1;
animateRobot(tout,Xout,traj_d,uout,p)


figure(2);
subplot(2,2,1);
plot(tout,traj_d(:,1),tout,xyz_world(:,1));
axis([0 max(t) 0 .51])
title('Xworld');
xlabel('seconds');
ylabel('m');
subplot(2,2,2);
plot(tout,traj_d(:,2),tout,xyz_world(:,2));
axis([0 max(t) -.254 .254])
title('Yworld');
ylabel('m');
subplot(2,2,3);
plot(tout,traj_d(:,3),tout,xyz_world(:,3));
axis([0 max(t) 0 .51])
title('Zworld');
xlabel('seconds');
ylabel('m');
subplot(2,2,4);
subplot(4,2,6);
plot(tout,uout(:,1),tout,uout(:,2),tout,uout(:,3));
title('Torques');
ylabel('N');
subplot(4,2,8);
plot(tout,Xout(:,1),tout,Xout(:,2),tout,Xout(:,3));
axis([0 max(t) -2 2])
title('Thetas');
xlabel('seconds');
ylabel('rad');

% for rotation about z only
% figure(3);
% plot(-xyz_world(:,2),xyz_world(:,1));
% axis([-.46 .46 0 .46*2]);
% pbaspect([1 1 1]);
% grid on;
% title('x,negative y plot');
% xlabel('negative y so z out of page');
% ylabel('x')

%for rotation about x and y and z weak
figure(3);
plot3(xyz_world(:,1),xyz_world(:,2),xyz_world(:,3));
xlim([0 0.46*2])
ylim([-0.46 0.46])
zlim([0 0.46*2])
view([2 2 1])
pbaspect([1 1 1]);
grid on;













