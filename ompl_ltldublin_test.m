% Control synthesis from LTL specifications for a Dublin's vehicle
% Usage: x = ompl_ltldublin(O, P, phi1, phi2, x0, x_lb, x_ub, u_lb, u_ub)

close all;
clear variables;
T = 0.025; % integration time
Ts = 0.1; % sampling time
n = 3;
m = 2;
O = {[0.0,0.9;
      1.1,0.9;
      1.1,1.1;
      0.0,1.1]};
P = {[0.9,0.3;
      1.1,0.3;
      1.1,0.5;
      0.9,0.5]; % p0
     [1.5,1.6;
      1.6,1.6;
      1.6,1.7;
      1.5,1.7]; % p1
     [0.2,1.7;
      0.3,1.7;
      0.3,1.8;
      0.2,1.8]}; % p2
phi1 = '! p0 U ((p2 & !p0) & XF p0)';
phi2 = 'G ! p1';
x0 = [0.2,0.2,0]';
x_lb = [-2.03,-2.35]';
x_ub = [2.84,2.85]';
u_lb = [ 0.0,-140*pi/180]';
u_ub = [ 0.7, 140*pi/180]';

x = p3.planner(O, P, phi1, phi2, x0, x_lb, x_ub, u_lb, u_ub);
xr = interpolate_solution(x,T);


plot_ris(O,P,x_lb,x_ub);
hold on;
plot(xr(:,1),xr(:,2),'r:',x(:,1),x(:,2),'b:');
axis equal

% Trajectory tracking using feedback linearization and LQR
kp = [1,1];
kd = [1,1];
sysd = c2d(ss([0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0],[0,0;0,0;1,0;0,1],eye(4),[]),Ts);
Q = [500,0,0,0;
     0,500,0,0;
     0,0,500,0;
     0,0,0,500];
R = eye(2);
K = dlqr(sysd.A,sysd.B,Q,R);

px = xr(1,1);
py = xr(1,2);
th = xr(1,3);
vx = 0.1*cos(th);
vy = 0.1*sin(th);
h1 = plot(px,py,'ko','MarkerFaceColor','k','MarkerSize',12);
h2 = quiver(px,py,vx,vy,'k','MaxHeadSize',24);
h1.XDataSource = 'px';
h1.YDataSource = 'py';
h2.XDataSource = 'px';
h2.YDataSource = 'py';
h2.UDataSource = 'vx';
h2.VDataSource = 'vy';
N = floor(size(xr,1)*T/Ts);
xs = zeros(N,n);
us = zeros(N,m);
xs(1,:) = xr(1,1:n);
v = 0;
for k=2:N
    ts = tic;
    px = xs(k-1,1);
    py = xs(k-1,2);
    th = xs(k-1,3);
    pxr = xr(floor((k-1)*Ts/T),1);
    pyr = xr(floor((k-1)*Ts/T),2);
    thr = xr(floor((k-1)*Ts/T),3);
    vr  = xr(floor((k-1)*Ts/T),4);    
    xsb = [px,py,v*[cos(th),sin(th)]];
    xrb = [pxr,pyr,vr*[cos(thr),sin(thr)]];
    uhb = -K*(xsb-xrb)';     
    a = uhb(1)*cos(xs(k-1,3))+uhb(2)*sin(xs(k-1,3));
    if v==0
        w = (thr-th)/Ts;
    else
        w = min(max((uhb(2)*cos(xs(k-1,3))-uhb(1)*sin(xs(k-1,3)))/v,u_lb(2)),u_ub(2));
    end
    v = min(max(v + a*Ts,u_lb(1)),u_ub(1));
    for i=1:Ts/T
        px = px + v*cos(th)*T;
        py = py + v*sin(th)*T;
        th = wrapToPi(th + w*T);    
    end    
    xs(k,:) = [px,py,th];
    us(k,:) = [v,w];
    vx = 0.1*cos(th);
    vy = 0.1*sin(th);
    refreshdata;
    plot(xs(1:k,1),xs(1:k,2),'b-');
    t = toc(ts);
    pause(max(Ts-t,0.01));
end
hold off;

function x = interpolate_solution(xr,T)
    n = 3;
    m = 2;
    steps = round(xr(:,6)/T);
    K = sum(steps)+1;
    x = zeros(K,n+m);
    x(1,:) = xr(1,1:5);
    k=1;
    for i=2:size(xr,1)
        v = xr(i,4);
        w = xr(i,5);    
        for j=1:steps(i)
            x(k+1,:) = [x(k,1:2)+T*[v*cos(x(k,3)),v*sin(x(k,3))],...
                                                wrapToPi(x(k,3)+T*w),v,w];
            k = k+1;
        end
    end
end

function plot_ris(O,P,x_lb,x_ub)
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
    plot(polytope([eye(2);-eye(2)],[x_ub;-x_lb]),Options2);

    hold off;
end