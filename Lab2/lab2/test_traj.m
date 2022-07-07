t=0:0.01:2.1;
al=[];
bl=[];
cl=[];
for i = t
    [a,b,c]=traj(i);
    al=[al,a];
    bl=[bl,b];
    cl=[cl,c];
end

plot(t,al)