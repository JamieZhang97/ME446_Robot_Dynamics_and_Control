function dXdt = dyn_aerial(t,X,p)

params = p.params;

q = X(1:4);
dq = X(5:8);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% swing controller
qd = [0.75*pi;-0.5*pi];     % desired joint position
q1 = q(3);
q2 = q(4);
dq1 = dq(3);
dq2 = dq(4);

% joint PD control during aerial phase
u = [50*(qd(1)-q1)+2*(0-dq1);...
     50*(qd(2)-q2)+2*(0-dq2)];

ddq = De \ (Be * u - Ce*dq - Ge);

dXdt = [dq;ddq];
