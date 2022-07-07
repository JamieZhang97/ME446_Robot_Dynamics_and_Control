function dXdt = dyn_manip(t,X,p)

params = p.params;

q = X(1:3);
dq = X(4:6);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% gen desired traj in workspace
% traj_d = [pd,dpd,ddpd]
traj_d = fcn_gen_traj(t,p);

% Design your controller here
u = fcn_controller(X,traj_d,p);

ddq = De \ (Be * u - Ce * dq - Ge);
% ddq = De \ (Be * u + J' * F_ext - Ce*dq - Ge);

dXdt = [dq;ddq];
