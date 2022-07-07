function X_post = fcn_impactMap(X_prev,p)

params = p.params;

q_prev = X_prev(1:4)';
dq_prev = X_prev(5:8)';

De = fcn_De(q_prev,params);
J = fcn_J_toe(q_prev,params);

dqF = [De -J';J zeros(3)] \ [De*dq_prev ; zeros(3,1)];

dq_post = dqF(1:4);
F_impact = dqF(5:7);        % impact impulse


X_post = [q_prev;dq_post];










