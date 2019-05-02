function teleop(this)
%TELEOP Summary of this function goes here
%   Detailed explanation goes here

    pub = rospublisher('RosAria/cmd_vel','geometry_msgs/Twist');
    msg = rosmessage(pub);
    fig = figure;
    subplot(2,1,1);
    dmsg = ['Reading from keyboard',newline,...
            '---------------------',newline,...
            'Press q to stop the program.',newline,...
            'a/d - move forward/backward',newline,...
            'w/s - move left/right'];
         
    set(gcf,'Color', 'white')
    text(0,0.5,dmsg,'fontsize',24,'color','k')
    set(gca,'Color','white');
    set(gca,'XColor','white');
    set(gca,'YColor','white');

    
    updatePlot(this,[],[]);
    
    t = timer;
    t.Period = 1;
    t.TasksToExecute = inf;
    t.ExecutionMode = 'fixedRate';
    t.TimerFcn = @(src, event) updatePlot(this,src,event);
    start(t)

    while(true)
        
        p = ' ';
        w = waitforbuttonpress;
        if w
           p = get(gcf, 'CurrentCharacter');
        end
        if double(p)==113
            this.v = 0;
            this.w = 0;
            msg.Angular.Z = this.w;
            msg.Linear.X = this.v;
            send(pub,msg);
            close(fig);
            stop(t)
            break;
        elseif double(p)==119
            this.v = 0.2;
            this.w = 0;
            msg.Angular.Z = this.w;
            msg.Linear.X = this.v;
            send(pub,msg);
        elseif double(p)==115
            this.v = -0.2;
            this.w = 0;
            msg.Angular.Z = this.w;
            msg.Linear.X = this.v;
            send(pub,msg);
        elseif double(p)==97
            this.v = 0;
            this.w = 0.5;
            msg.Angular.Z = this.w;
            msg.Linear.X = this.v;
            send(pub,msg);
        elseif double(p)==100
            this.v = 0;
            this.w = -0.5;
            msg.Angular.Z = this.w;
            msg.Linear.X = this.v;
            send(pub,msg);        
        end
    end
end

function updatePlot(client,~,~)
    subplot(2,1,2);
    u = cos(client.th);
    v = sin(client.th);
    quiver(client.x,client.y,u,v,'b-','linewidth',3)
    title('Current position')
    xlabel('position m');
    ylabel('position m');
    axis equal;
    axis([-1+client.x,1+client.x,-1+client.y,1+client.y]);
end
