clear all; close all; clc;
% Define D-H parameters 
a = [ 0, 0, 0, 0, 0, 0,0 ];
alpha= [ pi/2,-pi/2, pi/2, -pi/2,pi/2,-pi/2, 0];
d1 = 0.124;d3 = 0.14;d5 = 0.082;d7 = 0.182;
d =  [ d1, 0, d3, 0, d5, 0, d7] ;
%define servo limits
ql = [-90,90;-90,45;-135,45;-120,120;-135,125;-130,30;-135,125];
ql = deg2rad(ql);

% create robot's Links
for i = 1:7
L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% generate the object robot
robot = SerialLink(L);
%Insert servo limitations to the robotic model
robot.qlim = ql;
%name the robot
robot.name = "Ros";

%Select an initial joint angle configuration
q = TODO
ue = TODO

robot.plot(q,'trail', '-','lightpos', [1 1 1]);
t = 0;
dt = 1e-2;
i=1;
dir = ones(7,1);

while(t<10)

    qdot =TODO
    
    q(i+1,:) = q(i,:) + dt*qdot;

    check = robot.islimit(q(i+1,:));
    if norm(check(:,1)) >=1
        q(i+1,:) = q(i,:);
        ue= -ue;
    end
    t = t+dt;
    i = i+1;
end
robot.plot(q,'trail', 'k--','delay',dt);