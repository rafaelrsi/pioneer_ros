function execute(this,Q,R)
%EXECUTE Summary of this function goes here
%   Detailed explanation goes here
    pub = rospublisher('RosAria/cmd_vel','geometry_msgs/Twist');
    msg = rosmessage(pub);

    plot_ris(this.O,this.P,this.x_lb,this.x_ub,this.y_lb,this.y_ub);
    hold on;
    plot(this.X(:,1),this.X(:,2),'r:',this.Xr(:,1),this.Xr(:,2),'b:');
    axis equal
    
    % Trajectory tracking using feedback linearization and LQR
    K = dlqr(this.A,this.B,Q,R);
    
    n = 3;
    m = 2;
    u_lb = [0,this.w_lb]';
    u_ub = [0.7,this.w_ub]';
    px = this.x;
    py = this.y;
    th = this.th;
    vx = 0.1*cos(th);
    vy = 0.1*sin(th);
    v = 0;
    w = 0;
    h1 = plot(px,py,'bo','MarkerFaceColor','b','MarkerSize',10);
    h2 = quiver(px,py,vx,vy,'b','MaxHeadSize',24);
    h1.XDataSource = 'px';
    h1.YDataSource = 'py';
    h2.XDataSource = 'px';
    h2.YDataSource = 'py';
    h2.UDataSource = 'vx';
    h2.VDataSource = 'vy';
    pxr = this.X(1,1);
    pyr = this.X(1,2);
    thr = this.X(1,3);
    vr  = this.X(1,4);
    vxr = 0.1*cos(thr);
    vyr = 0.1*sin(thr);
    h3 = plot(pxr,pyr,'ro','MarkerFaceColor','r','MarkerSize',10);
    h4 = quiver(pxr,pyr,vxr,vyr,'r','MaxHeadSize',24);
    h3.XDataSource = 'pxr';
    h3.YDataSource = 'pyr';
    h4.XDataSource = 'pxr';
    h4.YDataSource = 'pyr';
    h4.UDataSource = 'vxr';
    h4.VDataSource = 'vyr';
    N = size(this.X,1);
    this.xs = zeros(N,n);
    this.us = zeros(N,m);
    this.xs(1,:) = [px,py,th];
    this.us(1,:) = [v,w];
    k=1;
    pxr = this.X(1,1);
    pyr = this.X(1,2);
    i = 1;
    while(i<N || (norm([px;py]-[pxr;pyr])>0.1 || (v<0.05 && w <0.1)))
        k=k+1;
        ts = tic;
        px = this.x;
        py = this.y;
        th = this.th;
        vx = 0.1*cos(th);
        vy = 0.1*sin(th);
        if i<N && norm([px;py]-[pxr;pyr])<=0.5
            i = i+1;
        end
        pxr = this.X(i,1);
        pyr = this.X(i,2);
        thr = this.X(i,3);
        vr  = this.X(i,4);   
        vxr = 0.1*cos(thr);
        vyr = 0.1*sin(thr);
        xsb = [px,py,v*[cos(th),sin(th)]];
        xrb = [pxr,pyr,vr*[cos(thr),sin(thr)]];
        this.xs(k,:) = [px,py,th];
        this.us(k,:) = [v,w];
        uhb = -K*(xsb-xrb)';     
        a = uhb(1)*cos(th)+uhb(2)*sin(th);
        if v<=0.05
            w = (thr-th)/this.Ts;
        else
            w = min(max((uhb(2)*cos(th)-uhb(1)*sin(th))/v,u_lb(2)),u_ub(2));
        end
        v = min(max(v + a*this.Ts,u_lb(1)),u_ub(1));
        this.v = v;
        this.w = w;
        msg.Angular.Z = this.w;
        msg.Linear.X = this.v;
        send(pub,msg);
        refreshdata(h1,'caller');
        refreshdata(h2,'caller');
        refreshdata(h3,'caller');
        refreshdata(h4,'caller');
        plot(this.xs(1:k,1),this.xs(1:k,2),'b-');
        t = toc(ts);
        pause(max(this.Ts-t,0.01));
    end
    hold off;    
end


function plot_ris(O,P,x_lb,x_ub,y_lb,y_ub)
    Options1.color='k';
    Options2.shade=0.25;
    
    for i=1:length(O)
        if i==1
            P_o = polytope(O{i});
        else
            P_o = [P_o,polytope(O{i})];
        end
    end
    plot(P_o,Options1);
    hold on;
    
    for i=1:length(P)
        if i==1
            P_p = polytope(P{i});
        else
            P_p = [P_p,polytope(P{i})];
        end
    end
    plot(P_p,Options2);
    for i=1:length(P)
        xc = analyticCenter(P_p(i));
        text(xc(1),xc(2),['p',num2str(i-1)],'HorizontalAlignment','center','FontWeight','bold','FontSize',18,'Color',[0,0,0]);
    end
    Options2.shade=0;
    plot(polytope([eye(2);-eye(2)],[x_ub;y_ub;-x_lb;-y_lb]),Options2);

    hold off;
end

