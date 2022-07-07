syms a0 a1 a2 a3
% for point 0
t0=4;
q0=0.5;
v0=0;

t1=7;
q1=-0.3;
v1=0;


eq1 = a0+a1*t0+a2*(t0^2)+a3*(t0^3)-q0;
eq2 = a1+2*a2*t0+3*a3*(t0^2)-v0; 
eq3 = a0+a1*t1+a2*(t1^2)+a3*(t1^3)-q1;
eq4 = a1+2*a2*t1+3*a3*(t1^2)-v1;
[a0,a1,a2,a3]=solve([eq1,eq2,eq3,eq4],[a0,a1,a2,a3]);
a0=vpa(a0,3)
a1=vpa(a1,3)
a2=vpa(a2,3)
a3=vpa(a3,3)
a22=vpa(2*a2,3)
a32=vpa(3*a3,3)
a33=vpa(a32*2,3)