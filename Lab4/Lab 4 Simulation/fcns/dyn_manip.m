function dXdt = dyn_manip(t,X,p)

params = p.params;

q = X(1:3);
dq = X(4:6);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

J4 = fcn_J4(q,params);
% gen desired traj in workspace
traj_d = fcn_gen_traj(t,p);

% Design your controller here
u = fcn_controller(X,traj_d,p);

if p.flag_ctrl == 1 || p.flag_ctrl == 2 || p.flag_ctrl == 4
    % Exert a force on the end effector
    t_mod = mod(t,2);
    if t_mod < 1
        f = [-5;-5;-5];
    else
        f = [5;5;5];
    end
    ddq = De \ (Be * u - Ce*dq - Ge + J4'*f);

else
    ddq = De \ (Be * u - Ce*dq - Ge); 
end

dXdt = [dq;ddq];
