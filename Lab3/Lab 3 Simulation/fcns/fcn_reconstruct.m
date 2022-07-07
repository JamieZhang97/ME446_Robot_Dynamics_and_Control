function [traj_d,u] = fcn_reconstruct(t,X,p)

lent = length(t);
if p.flag_ctrl == 0 || p.flag_ctrl == 2
    traj_d = zeros(lent,3);
elseif p.flag_ctrl == 1
    traj_d = zeros(lent,9);
end
u = zeros(lent,3);

for ii = 1:lent
    traj_d(ii,:) = fcn_gen_traj(t(ii),p)';
    u(ii,:) = fcn_controller(X(ii,:)',traj_d(ii,:)',p);
end

end