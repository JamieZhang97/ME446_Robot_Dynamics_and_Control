function [Ce] = fcn_Ce(q,dq,p)

Ce = zeros(3,3);

  Ce(1,1)=- dq(2)*((p(5)*p(2)^2*sin(2*q(2)))/8 + (p(6)*p(2)^2*sin(2*q(2)))/2 + (p(6)*p(3)^2*sin(2*...
         q(2) + 2*q(3)))/8 + (p(6)*p(2)*p(3)*sin(2*q(2) + q(3)))/2) - (p(6)*dq(3)*p(3)*(p(3)*sin(2*q(2) + 2*q(3)) +...
          2*p(2)*sin(q(3)) + 2*p(2)*sin(2*q(2) + q(3))))/8;
  Ce(1,2)=-dq(1)*((p(5)*p(2)^2*sin(2*q(2)))/8 + (p(6)*p(2)^2*sin(2*q(2)))/2 + (p(6)*p(3)^2*sin(2*...
         q(2) + 2*q(3)))/8 + (p(6)*p(2)*p(3)*sin(2*q(2) + q(3)))/2);
  Ce(1,3)=-(p(6)*dq(1)*p(3)*(p(3)*sin(2*q(2) + 2*q(3)) + 2*p(2)*sin(q(3)) + 2*p(2)*sin(2*q(2) + q(3))))/8;
  Ce(2,1)=dq(1)*((p(5)*p(2)^2*sin(2*q(2)))/8 + (p(6)*p(2)^2*sin(2*q(2)))/2 + (p(6)*p(3)^2*sin(2*...
         q(2) + 2*q(3)))/8 + (p(6)*p(2)*p(3)*sin(2*q(2) + q(3)))/2);
  Ce(2,2)=-(p(6)*dq(3)*p(2)*p(3)*sin(q(3)))/2;
  Ce(2,3)=-(p(6)*p(2)*p(3)*sin(q(3))*(dq(2) + dq(3)))/2;
  Ce(3,1)=(p(6)*dq(1)*p(3)*(p(3)*sin(2*q(2) + 2*q(3)) + 2*p(2)*sin(q(3)) + 2*p(2)*sin(2*q(2) + q(3))))/8;
  Ce(3,2)=(p(6)*dq(2)*p(2)*p(3)*sin(q(3)))/2;
  Ce(3,3)=0;

 