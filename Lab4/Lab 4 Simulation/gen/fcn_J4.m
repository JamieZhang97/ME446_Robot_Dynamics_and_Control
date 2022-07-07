function [J4] = fcn_J4(q,p)

% Jacobian : Maps task space coordinates in the world frame (x,y,z) to joint space coordinates (q1, q2, q3)
% The jacobian and jacobian transpose are a function a joint space coordinates (q1, q2, q3)

J4 = zeros(3,3);

  J4(1,1)=p(3)*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1))) - p(2)*cos(q(2))*sin(q(1));
  J4(1,2)=- p(3)*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2))) - p(2)*cos(q(1))*sin(q(2));
  J4(1,3)=-p(3)*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)));
  J4(2,1)=p(3)*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3))) + p(2)*cos(q(1))*cos(q(2));
  J4(2,2)=- p(3)*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2))) - p(2)*sin(q(1))*sin(q(2));
  J4(2,3)=-p(3)*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)));
  J4(3,1)=0;
  J4(3,2)=- p(3)*(cos(q(2))*cos(q(3)) - sin(q(2))*sin(q(3))) - p(2)*cos(q(2));
  J4(3,3)=-p(3)*(cos(q(2))*cos(q(3)) - sin(q(2))*sin(q(3)));

 