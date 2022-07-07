function traj_d = fcn_gen_traj(t,p)

if p.flag_ctrl == 0
    %TODO: Design the target position and then step back and forth
    x = 0.5 ; 
    y = 0.2 ;
    z = 0.4 ;
    % hold at a point, so the differential will be zero
    xdot = 0;
    ydot = 0;
    zdot = 0;

    traj_d = [x,y,z,xdot,ydot,zdot]';

    
elseif p.flag_ctrl == 1 || p.flag_ctrl == 2 
    %TODO: Design the target position
    x = 0.5; 
    y = 0.2;
    z = 0.4;

    xdot = 0;
    ydot = 0;
    zdot = 0;

    traj_d = [x,y,z,xdot,ydot,zdot]';



elseif p.flag_ctrl == 3 
    %TODO: Design the straight line trajectory
    start_point = [0.35, -0.2, 0.25];
    end_point = [0.25, 0.3, 0.15];
    t_total = 2;
    % use the start point and end point and time to calculate the
    % line trajectory
    x = 0.35 - 0.05*t; 
    y = -0.2 + 0.25*t;
    z = 0.25 - 0.05*t;

    xdot = -0.05;
    ydot = 0.25;
    zdot = -0.05;

    traj_d = [x,y,z,xdot,ydot,zdot]';



elseif p.flag_ctrl == 4
    %TODO: Design the straight line trajectory
    start_point = [0.35, -0.2, 0.25];
    end_point = [0.25, 0.3, 0.25];
    t_total = 2;
    % use the start point and end point and time to calculate the
    % line trajectory    
    x = 0.35 - 0.05*t; 
    y = -0.2 + 0.25*t;
    z = 0.25;

    xdot = -0.05;
    ydot = 0.25;
    zdot = 0;

    traj_d = [x,y,z,xdot,ydot,zdot]';


end

end
