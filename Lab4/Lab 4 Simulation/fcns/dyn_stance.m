function dXdt = dyn_stance(t,X,p)

params = p.params;
tTD = p.tTD;            % touchdown time
ptTD = p.ptTD;          % touchdown toe position
Tst = p.Tst;

q = X(1:4);
dq = X(5:8);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);
J = fcn_J_toe(q,params);

% holonomic constraints
Jhc = fcn_Jhc(q,ptTD,params);
dJhc = fcn_dJhc(q,dq,ptTD,params);

%% Controller

% feedforward force
Fx = 0;
s = (t - tTD) / Tst;        % stance phase var
% force profile
Fz = polyval_bz([0 0 0 0 50 0],s);

F = [Fx;0;Fz];

% solve the linear system:
% D * ddq + C * dq + G = J' * F + B * u (4 eqns)
% Jhc * ddq + dJhc * dq = 0 (2 eqns)
% unknowns: ddq(4x1), u(2x1)
% control: F (3x1)
Amat = [De -Be; Jhc zeros(2,2)];
bvec = [J'*F - Ce*dq - Ge; -dJhc*dq];

ddqu = Amat \ bvec;
ddq = ddqu(1:4);

dXdt = [dq;ddq];



