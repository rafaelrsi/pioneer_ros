function execute4(this,H,Q,R)
%EXECUTE Summary of this function goes here
%   Detailed explanation goes here
    pub = rospublisher('RosAria/cmd_vel','geometry_msgs/Twist');
    msg = rosmessage(pub);

    plot_ris(this.O,this.P,this.x_lb,this.x_ub,this.y_lb,this.y_ub);
    hold on;
    plot(this.X(:,1),this.X(:,2),'r:',this.Xr(:,1),this.Xr(:,2),'b:');
    axis equal
    
    % Trajectory tracking using feedback linearization
    n = 4;
    m = 2;        
    
    % MPC with horizon H parameters
    A = this.A;
    B = this.B;
    xidx = reshape((1:n)'+(n+m)*(0:H),(H+1)*n,1);
    pxidx = 1+(n+m)*(0:H);
    pyidx = 2+(n+m)*(0:H);
    u_lb = [-2,-2];
    u_ub = [ 2, 2];
    vub = 0.7;
    vlb = 0;
    wub = deg2rad( 140);
    wlb = deg2rad(-140);    
    xub = this.x_ub;
    xlb = this.x_lb;
    yub = this.y_ub;
    ylb = this.y_lb;
    problem.H = [zeros(n,(n+m)*H+n);zeros(H*(n+m),n),kron(eye(H),blkdiag(R,Q))];
    problem.Aeq = [eye(n),zeros(n,(n+m)*H);
        [kron(eye(H),[A,B]),zeros(n*H,n)]+...
               [zeros(n*H,n),kron(eye(H),[zeros(size(B)),-eye(size(A))])]];
    problem.beq = zeros(n*(H+1),1);
    problem.ub = [xub;yub; vub; vub;repmat([u_ub';xub;yub; vub; vub],H,1)];
    problem.lb = [xlb;ylb;-vub;-vub;repmat([u_lb';xlb;ylb;-vub;-vub],H,1)];
    problem.Aineq = [];
    problem.bineq = [];
    problem.f = zeros(H*(n+m)+n,1);
    problem.solver = 'quadprog';
    problem.options = optimoptions('quadprog','Display','off');
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
    pxw = px;
    pyw = py;
    h3 = plot(pxr,pyr,'ro','MarkerFaceColor','r','MarkerSize',10);
    h4 = quiver(pxr,pyr,vxr,vyr,'r','MaxHeadSize',24);
    h5 = plot(pxw,pyw,'r--','linewidth',3);
    h3.XDataSource = 'pxr';
    h3.YDataSource = 'pyr';
    h4.XDataSource = 'pxr';
    h4.YDataSource = 'pyr';
    h4.UDataSource = 'vxr';
    h4.VDataSource = 'vyr';
    h5.XDataSource = 'pxw';
    h5.YDataSource = 'pyw';
    N = size(this.X,1);
    xr = zeros(N,n);
    this.xs = zeros(N,3);
    this.us = zeros(N,2);
    for i=1:N
        xr(i,:) = [this.X(i,1),this.X(i,2),this.X(i,4)*[cos(this.X(i,3)),sin(this.X(i,3))]];
    end
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
        if i<N && norm([px;py]-[pxr;pyr])<=0.25
            i = i+1;
        end
        pxr = this.X(i,1);
        pyr = this.X(i,2);
        thr = this.X(i,3);
        vr  = this.X(i,4);   
        vxr = 0.1*cos(thr);
        vyr = 0.1*sin(thr);
        this.xs(k,:) = [px,py,th];
        this.us(k,:) = [v,w];
        
        % MPC with horizon H.
        problem.beq(1:4) = [px,py,v*[cos(th),sin(th)]];
        if i+1+H<=N
            problem.f(xidx) = -reshape(xr(i+1:i+1+H,:)',(H+1)*n,1);
        else
            problem.f(xidx) = -[reshape(xr(i+1:N,:)',(N-i)*n,1);repmat(xr(N,:)',H+1+i-N,1)];
        end
        problem.f = problem.H'*problem.f;
        z = quadprog(problem);
        pxw = z(pxidx);
        pyw = z(pyidx);
        
        uhb = z(n+(1:m));
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
        refreshdata(h5,'caller');
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

