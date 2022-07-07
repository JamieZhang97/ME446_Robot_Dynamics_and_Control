function [td0, td1, td2] = traj(t)
    if t<=0
        td0=0;
        td1=0;
        td2=0;
    elseif t<=1
        td0=1.5*t^2-t^3;
        td1=3*t-3*t^2;
        td2=3-6*t;
    elseif t<=2
        td0=-2+6*t-9/2*t^2+t^3;
        td1=6-9*t+3*t^2;
        td2=-9+6*t;
    else
        td0=0;
        td1=0;
        td2=0;   
    end
end