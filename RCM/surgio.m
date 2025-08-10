%%%%% TEAM PROJECT - SURGICAL ROBOTICS %%%%%
clc; clear; close all;

% DH parameters
a = [ 0, 0, 0, 0, 0, 0,0 ];
alpha= deg2rad([ 90,-90, 90, -90,90,-90, 0]);
d1 = 0.124; d3 = 0.14; d5 = 0.082; d7 = 0.3;
d =  [ d1, 0, d3, 0, d5, 0, d7] ;

% Robot limits
ql = deg2rad([-90,90; -45,90; -135,45; -120,120; -135,125; -30,130; -135,125]);
N=7;

% Create robot's Links
for i = 1:7
    L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% Generate the object robot
robot = SerialLink(L);

% Insert servo limitations to the robotic model
robot.qlim = ql;
robot.name="RAMIS";

% Starting deg
q=deg2rad([0 80.1 0 80.1 0 0 0]);

% Determine if Plot or Arduino from User input
% while 1
%     Choice = input('Do you want Simulation(1) or Arduino communication(2)?: ');
%     if ((Choice == 1) || (Choice == 2))
%         break
%     end
% end
% 
% if Choice == 2
%     %Run this once
%     % Arduino communication
%     delete(instrfind('Type', 'serial')); % Assure that no serial port is open anymore
%     arduino = serialport('COM8',9600);   
%     configureTerminator(arduino,"CR");
%     arduino.Timeout = 40;
%     pause(2);  % give servo some time to initialize
% 
%     % %Calibration in order to set the robot to its starting postition
%     dq = [-5,0,6,-4,0,-2,5];
%     q0 = [90,90,90,90,90,90,90]+dq;
%     commands(1:n) = q0;
%     packet = sprintf('%s,' , string(commands));
%     readline(arduino)
%     write(arduino,packet,"string");
% end
%------------------------------------------------------------------%
% RCM
Pc = [0 0 0.4960];
% Tooltip Velocity
Vu = [0 1 0 0 0];
Vt = [0 0 0 0 0 0];

dt = 1e-2;
t=0;
i=1;
 while(t<10)
    T = fwkin(q(i,:),d,a); % Forward Kinematics
    Pt = T(1:3,4)'; % Position
    Rt = T(1:3,1:3); % Rotation
    Nt = Rt(3,1:3); % Z Axis
    Bc = Rt(1:3,1:2); % X Y Axis
    Jac = robot.jacobe(q(i,:)); % Jacobian in tool orientation
    % Ax is the RCM constraint Jacobian in tool tip operational space
    Ax = Bc' * [eye(3,3), skew(Pt - Pc)];
    %Matrix A denotes the RCM constraint Jacobian in the joint space.
    A = Ax * Jac;   
    % Zx is the basis for the null space of Ax
    Zx = [Nt, zeros(1,3); Rt' * skew(Pt - Pc), Rt'];  
    Z = pinv(Jac) * Zx';
    % G is the base matrix of the null space of the Jacobian
    G = null(Jac)  
    % N is the basis of the null space of the RCM constraint Jacobian
    N = [Z G];
    % Matrix S is a full rank matrix assuming
    % a motion away from kinematics singularities
    S = [ pinv(A), N];
    % Calculate Qdot using previous velocities
    qdot = pinv(Jac) * Vt' ;
    % Calculate Xcdot
    Xcdot = A * qdot ;
    % Calculate Qdot using Xcdot
    qdot = S * [Xcdot ; Vu'];
    % Extracting the new velocities
    Vt = ([Jac * Z , Jac * G] * Vu' + Jac*pinv(A)*Xcdot)';
    % Saving the new angles
    q(i+1,:) = q(i,:) + dt * qdot';
    %Check robot limits
    check = robot.islimit(q(i+1,:));
    if norm(check(:,1)) >=1
            q(i+1,:) = q(i,:);
            Vt= -Vt;
            Vu = -Vu;
    end
    t = t+dt;
    i = i+1;
 end
% Plotting the Robot
robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);