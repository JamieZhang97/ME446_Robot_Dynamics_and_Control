function [Ge] = fcn_Ge(q,p)

Ge = zeros(3,1);

  Ge(1,1)=0;
  Ge(2,1)=-p(10)*(p(6)*((p(3)*cos(q(2) + q(3)))/2 + p(2)*cos(q(2))) + (p(5)*p(2)*cos(q(2)))/2);
  Ge(3,1)=-(p(6)*p(10)*p(3)*cos(q(2) + q(3)))/2;

 