function [outputArg1,outputArg2] = plan(this,O, P, phi1, phi2, x_lb, x_ub, y_lb, y_ub)
%PLANNER Summary of this function goes here
% Plans a trajectory (state and input) of a Dublin's vehicle which 
% satisfies a LTLcosafe and a LTL safety formulas.
% 
% Requires: OMLP (http://ompl.kavrakilab.org) and 
%           SPOT (https://spot.lrde.epita.fr/)
%
% Usage : from MATLAB
%      >> plan(O, P, phi1, phi2, x0, x_lb, x_ub, u_lb, u_ub)
%
%      * O is a cell vector with vertices of obstacles, i.e. O(i) is a 
%        numeric matrix with a row for each 2D vertices.
%
%      * P is a cell vector with vertices of propositions, i.e. P(i) is a 
%        numeric matrix with a row for each 2D vertices.
%
%      * phi1, phi2 are cosafety LTL and safety LTL formulas, respectively,
%        i.e. these arguments are strings. It uses SPOT syntax.
%
%      * x_lb,x_ub,y_lb,y_ub are scalars with lower and upper bounds of the
%        state space, i.e. (px,py).
%

    x0 = [this.x,this.y,this.th]';
    u_lb = [this.v_lb,this.w_lb]';
    u_ub = [this.v_ub,this.w_ub]';
    this.O = O;
    this.P = P;
    this.x_lb = x_lb;
    this.x_ub = x_ub;
    this.y_lb = y_lb;
    this.y_ub = y_ub;
    this.phi1 = phi1;
    this.phi2 = phi2;
    this.Xr = p3.planner(O, P, phi1, phi2, x0, [x_lb,y_lb]', [x_ub,y_ub]', u_lb, u_ub);
    this.X = interpolate_solution(this.Xr,this.Ts);

end

function x = interpolate_solution(xr,Ts)
    n = 3;
    m = 2;
    steps = xr(:,6)/Ts;
    K = ceil(sum(steps)+1);
    x = zeros(K,n+m);
    x(1,:) = xr(1,1:5);
    k=1;
    T = 0;
    v = 0;
    w = 0;
    for i=2:size(xr,1)
        h = xr(i,6);
        T = T + h;        
        if T >= Ts
            % collocation scheme
            dT = T-Ts;
            
            % collocation points N=2 {x(0),x(h)}
            % x(t) = a0 + a1*t + a2*t^2
            % xdot(t) = a1 + 2*a2*t
            % x(0) = a0
            % xdot(0) = f(0,x(0)) = a1
            % xdot(h) = f(t1,x(h)) = a1 + 2*a2*h => a2 = a1/(2*h)
            a0 = xr(i-1,1:3)';
            a1 = [xr(i-1,4)*cos(xr(i-1,3));
                  xr(i-1,4)*cos(xr(i-1,3));
                  xr(i-1,5)];
            a2 = a1./(2*h);
            % x(dT) = a0 + a1*dT + a2*dt^2
            k=k+1;
            x(k,:) = [a0+a1.*dT+a2.*dT^2;v/Ts+xr(i,4)*dT;w/Ts+xr(i,5)*dT]';
            T = h-dT;
            v = xr(i,4)*(h-dT);            
            w = xr(i,5)*(h-dT);
        else
            v = v + h*xr(i,4);
            w = w + h*xr(i,5);
            if i==size(xr,1)
                dT = Ts;
                k=k+1;
                x(k,:) = [xr(i,1:3)';v/Ts;w/Ts]';
            end                
        end
    end
    
    x = x(1:k,:);        
end
