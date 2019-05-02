close all
clear variables
clc

% initialize ros
client = p3.client();
client.connect();
pause(5);

if (client.x>0.5 || client.y>0.5)
    client.teleop();
end

% Q = [200,0,0,0;
%      0,200,0,0;
%      0,0,0.01,0;
%      0,0,0,0.01];
Q = diag([1,1,1,1]);
R = eye(2);
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
client.plan(O, P, phi1, phi2, -2.03, 2.84, -2.35, 2.85);
client.execute4(5,Q,R);
client.shutdown;