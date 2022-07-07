function [xyz,traj_d,u] = fcn_reconstruct(t,X,p)

lent = length(t);

traj_d = zeros(lent,6);
u = zeros(lent,3);
xyz = zeros(lent,3);

for ii = 1:lent
    xyz(ii,:) = fcn_p4(X(ii,1:3)',p.params);
    traj_d(ii,:) = fcn_gen_traj(t(ii),p)';
    u(ii,:) = fcn_controller(X(ii,:)',traj_d(ii,:)',p);
end




end
