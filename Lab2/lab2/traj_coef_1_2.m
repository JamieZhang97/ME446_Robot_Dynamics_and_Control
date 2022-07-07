syms a0 a1 a2 a3
t0=1;
q0=0.5;
v0=0;
t1=2;
q1=0;
v1=0;
eq1 = a0+a1*t0+a2*(t0^2)+a3*(t0^3)-q0;
eq2 = a1+2*a2*t0+3*a3*(t0^2)-v0; 
eq3 = a0+a1*t1+a2*(t1^2)+a3*(t1^3)-q1;
eq4 = a1+2*a2*t1+3*a3*(t1^2)-v1;
[a0,a1,a2,a3]=solve([eq1,eq2,eq3,eq4],[a0,a1,a2,a3])