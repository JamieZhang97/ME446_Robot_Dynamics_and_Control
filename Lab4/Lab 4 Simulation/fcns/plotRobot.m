function plotRobot(X,p)

params = p.params;


q = X(1:3);
p1 = [0 0 0]';
p2 = fcn_p2(q,params);
p3 = fcn_p3(q,params);
p4 = fcn_p4(q,params);


chain = [p1 p2 p3 p4];


plot3(chain(1,:),chain(2,:),chain(3,:),'color','black','linewidth',4);
xlabel('x [in]','fontsize',12)
ylabel('y [in]','fontsize',12)
zlabel('z [in]','fontsize',12)



end
