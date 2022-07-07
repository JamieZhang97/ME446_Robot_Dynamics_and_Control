function traj_d = fcn_gen_traj(t,p)

    if p.flag_ctrl == 0
        
        t_mod = mod(t,2);
        
        if t_mod < 1
            qd = [pi/6;
                  -1/3*pi;
                   2/3*pi];
        else
            qd = [0;
                  -1/2*pi;
                   1/2*pi];
        end
        traj_d = qd;

    elseif p.flag_ctrl == 1 || p.flag_ctrl == 3
        
        t_mod = mod(t,2);
        
        if t_mod < 1
            qd = [-t_mod^3+1.5*t_mod^2;
                  -t_mod^3+1.5*t_mod^2 - pi/2;
                  -t_mod^3+1.5*t_mod^2 + pi/2];

            dqd = [-3*t_mod^2+3*t_mod;
                   -3*t_mod^2+3*t_mod;
                   -3*t_mod^2+3*t_mod];

            ddqd = [-6*t_mod+3;
                    -6*t_mod+3;
                    -6*t_mod+3];
        else
            qd = [t_mod^3-4.5*t_mod^2+6*t_mod-2;
                  t_mod^3-4.5*t_mod^2+6*t_mod-2 - pi/2;
                  t_mod^3-4.5*t_mod^2+6*t_mod-2 + pi/2];

            dqd = [3*t_mod^2-9*t_mod+6;
                   3*t_mod^2-9*t_mod+6;
                   3*t_mod^2-9*t_mod+6];

            ddqd = [6*t_mod-9;
                    6*t_mod-9;
                    6*t_mod-9];
        end
        traj_d = [qd;dqd;ddqd];

    elseif p.flag_ctrl == 2
        
        x = 15+4*cos(t); 
        y = 4*sin(t); 
        z = 10;
        
        % Inverse Kinematic
        q1 = atan2(y,x); 
        q3 = pi - acos((200-(x^2+y^2)-(z-10)^2)/200); 
        q2 = -(q3/2 + atan2(z-10,sqrt(x^2+y^2)));
        
        qd = [q1;q2;q3];
        traj_d = qd;
    
    elseif p.flag_ctrl == 4 || p.flag_ctrl == 5 || p.flag_ctrl == 6 || p.flag_ctrl == 7
        
        % TODO: Create your fast trajectory here
        t_mod = mod(t,2);
        
        if ((t_mod > 0) && (t_mod <= 0.33))
            qd = [-27*t_mod^3 + 13.5*t_mod^2;
                  -27*t_mod^3 + 13.5*t_mod^2 - pi/2;
                  -27*t_mod^3 + 13.5*t_mod^2 + pi/2];
            dqd = [-81*t_mod^2 + 27*t_mod;
                   -81*t_mod^2 + 27*t_mod;
                   -81*t_mod^2 + 27*t_mod];
            ddqd = [-162*t_mod + 27;
                    -162*t_mod + 27;
                    -162*t_mod + 27];
        
        elseif ((t_mod > 0.33) && (t_mod < 1))          
            qd = [0.5; 0.5 - pi/2; 0.5 + pi/2];
            dqd = [0.0; 0.0; 0.0];
            ddqd = [0.0; 0.0; 0.0];
            
        elseif ((t_mod >= 1) && (t_mod <= 1.33))
            qd = [27*t_mod^3 - 94.5*t_mod^2 + 108*t_mod - 40;
                  27*t_mod^3 - 94.5*t_mod^2 + 108*t_mod - 40 - pi/2;
                  27*t_mod^3 - 94.5*t_mod^2 + 108*t_mod - 40 + pi/2];
            dqd = [81*t_mod^2 - 189*t_mod + 108;
                   81*t_mod^2 - 189*t_mod + 108;
                   81*t_mod^2 - 189*t_mod + 108];
            ddqd = [162*t_mod - 189;
                    162*t_mod - 189;
                    162*t_mod - 189];
        else          
            qd = [0.0; -0.5*pi; 0.5*pi];
            dqd = [0.0; 0.0; 0.0];
            ddqd = [0.0; 0.0; 0.0];
        end
%         disp(t_mod);
%         disp(qd);
%         disp(dqd);
%         disp(ddqd);
        traj_d = [qd; dqd; ddqd];
    end
end