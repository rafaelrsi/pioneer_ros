classdef client < handle
    %P3CLIENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess=private)
        x
        y
        th
        thd
        v
        w
        host
        isMotorEnabled
        v_lb
        v_ub
        w_lb
        w_ub
        Ts
        X
        xs
        us
    end
    
    properties (Access=private)
        sub_pose
        sub_motor
        Xr
        phi1
        phi2
        O
        P
        x_lb
        x_ub
        y_lb
        y_ub
        A
        B
    end
    
    methods
        function this = client(host)
            %P3CLIENT Construct an instance of this class
            %   Detailed explanation goes here
            this.x = [];
            this.y = [];
            this.th = [];
            this.thd = [];
            this.v = 0;
            this.w = 0;
            this.v_lb = 0;
            this.v_ub = 0.35;
            this.w_lb = -140*pi/180;
            this.w_ub = -this.w_lb;
            this.Ts = 0.25;
            this.A = [1,0,this.Ts,0;
                      0,1,0,this.Ts;
                      0,0,1,0;
                      0,0,0,1];
            this.B = [this.Ts^2/2,0;
                      0,this.Ts^2/2;
                      this.Ts,0;
                      0,this.Ts];
            
            if nargin == 0
                this.host = 'localhost';
            else
                this.host = host;
            end
        end
        
        function connect(this)
            rosinit(this.host);
            pause(1);
            this.sub_pose = rossubscriber('RosAria/pose','nav_msgs/Odometry',...
                @(src,msg) p3.client.poseROSCallback(this,src,msg));
            this.sub_motor = rossubscriber('RosAria/motors_state','std_msgs/Bool',...
                @(src,msg) p3.client.motorROSCallback(this,src,msg));
        end
        
        execute4(this,H,Q,R)
        execute3(this,Q,R)
        execute2(this,Q,R)
        execute(this,Q,R)
        
    end
        
    methods (Static)
         function shutdown()
            rosshutdown;
        end
        
        function enable_motors()
            rosservice call /RosAria/enable_motors
        end        
       
        
        function poseROSCallback(p3,~,msg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            p3.x = msg.Pose.Pose.Position.X;
            p3.y = msg.Pose.Pose.Position.Y;
            [~,~,p3.th] = quat2angle([msg.Pose.Pose.Orientation.X;
                                      msg.Pose.Pose.Orientation.Y;
                                      msg.Pose.Pose.Orientation.Z;
                                      msg.Pose.Pose.Orientation.W]');
            p3.thd = rad2deg(p3.th);
        end
        function motorROSCallback(p3,~,msg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            p3.isMotorEnabled = msg.Data;
        end
    end
end

