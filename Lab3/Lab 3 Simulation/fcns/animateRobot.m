function animateRobot(tin,Xin,traj_din,uin,p)

% For animation
N = p.N_animate;
flag_movie = p.flag_movie;
if flag_movie
    name = ['test.mp4'];
    vidfile = VideoWriter(name,'MPEG-4');
    open(vidfile);
end

[t,X] = even_sample(tin,Xin,N);
[t,traj_d] = even_sample(tin,traj_din,N);
[t,u] = even_sample(tin,uin,N);


% animate
figure('Position',[200 100 1000 600]);

nt = length(t);
err = zeros(nt,3);
traj = zeros(nt,3);

for ii = 1:nt
    
    %% The main animation
    subplot(2,2,1)
    hold on;grid on;box on;
    % plot robot
    plotRobot(X(ii,:),p);
    
    % plot past tracking
    N = floor(p.N_animate/2);
    if ii <= N
        p4 = fcn_get_p4(X(1:ii,:),p);
        if p.flag_ctrl == 2
            td = fcn_p4(traj_d(ii,1:3),p.params)';
        else
            td = traj_d(1:ii,:);
        end
    else
        p4 = fcn_get_p4(X(ii-N:ii,:),p);
        if p.flag_ctrl == 2
            td = fcn_p4(traj_d(ii,1:3),p.params)';
        else
            td = traj_d(1:ii,:);
        end
    end
    % trace of end-effector
    plot3(p4(:,1),p4(:,2),p4(:,3),'b')
   
    % plot desired trajectory
    plot3(td(:,1),td(:,2),td(:,3),'r','linewidth',0.5)
    plot3(td(end,1),td(end,2),td(end,3),'ro','linewidth',1.5)

    % set view
    xlim([0 0.6])
    ylim([-0.3 0.3])
    zlim([0 0.6])
    view([2 2 1])
    
    %% The tracking error
    if p.flag_ctrl ~=2
        subplot(2,2,2)
        hold on;grid on;box on;
        traj(ii,:) = X(ii,1:3)';
        err(ii,:) = traj_d(ii,1:3) - traj(ii,:);
        plot(t(1:ii),err(1:ii,1),'r')
        plot(t(1:ii),err(1:ii,2),'b')
        plot(t(1:ii),err(1:ii,3),'g')
        legend('Joint1','Joint2','Joint3')
        title('Tracking Error')
        xlabel('Time [s]')
        ylabel('Error [rad]')
    elseif p.flag_ctrl == 2
        subplot(2,2,2)
        hold on;grid on;box on;
        traj(ii,:) = fcn_p4(X(ii,1:3),p.params)';
        traj_d(ii,:) = fcn_p4(traj_d(ii,1:3),p.params)';
        err(ii,:) = traj_d(ii,1:3) - traj(ii,:);
        plot(t(1:ii),err(1:ii,1),'r')
        plot(t(1:ii),err(1:ii,2),'b')
        plot(t(1:ii),err(1:ii,3),'g')
        legend('ex','ey','ez')
        title('Tracking Error')
        xlabel('Time [s]')
        ylabel('Error [m]')
    end
    %% The control
    subplot(2,2,3)
    hold on;grid on;box on;
    plot(t(1:ii),u(1:ii,1),'r')
    plot(t(1:ii),u(1:ii,2),'b')
    plot(t(1:ii),u(1:ii,3),'g')
    legend('u1','u2','u3')
    title('Control Input')
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    
    %% The position
    if p.flag_ctrl ~= 2
        subplot(2,2,4)
        hold on;grid on;box on;
        plot(t(1:ii),traj(1:ii,1),'r')
        plot(t(1:ii),traj(1:ii,2)+pi/2,'b')
        plot(t(1:ii),traj(1:ii,3)-pi/2,'g')
        plot(t(1:ii),traj_d(1:ii,1),'m')
        legend('Joint1','Joint2','Joint3','Ref')
        title('Joint Position')
        xlabel('Time [s]')
        ylabel('Relative Angle [rad]')
    end    
    % animate
    pause(0.0001)
    
    % record video
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end
    
    if ii < nt
        clf
    end
    
end

if flag_movie
    close(vidfile);
end


end

function p4 = fcn_get_p4(X,p)

params = p.params;

len = size(X,1);
p4 = zeros(len,3);
for ii = 1:len
    q = X(ii,1:3);
    p4(ii,:) = fcn_p4(q,params);
    
end

end


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







