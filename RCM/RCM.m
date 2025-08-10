clear all; close all; clc;
% construct the object arduino
% a = arduino('COM3');
% % num of servos
% n = 7;
% s1 = servo(a, 'D3',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s2 = servo(a, 'D4',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s3 = servo(a, 'D5',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s4 = servo(a, 'D6',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s5 = servo(a, 'D7',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s6 = servo(a, 'D8',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s7 = servo(a, 'D9',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% s = [s1;s2;s3;s4;s5;s6;s7];
% pause(1);

%DH PARAMETRES
alpha = [deg2rad(90), deg2rad(-90),deg2rad(-90), deg2rad(90), deg2rad(90), deg2rad(-90),deg2rad(0)];
a= [0,0,0,0,0,0,0];
d1 = 0.124;d3 = 0.14;d5 = 0.082;d7 = 0.182;
d =  [ d1, 0, d3, 0, d5, 0, d7] ;

ql = [-90,90;-45,90;-135,45;-120,120;-135,125;-30,130;-135,125];% evros kinhshs
ql = deg2rad(ql);
N=7;

% create robot's Links
for i = 1:7
    L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% generate the object robot
robot = SerialLink(L);
% Insert servo limitations to the robotic model
robot.qlim = ql;
robot.name="ROS";

dt=0.005;% apeirosth metatopish tou xronou
K=1;% kerdos

t=0;
i=1;
q=deg2rad([-45 -45 0 90 0 0 0]);
Pc = [0 0 0.75];
ue = [0 0 0 0 0 0]; 
vu = [0 0 1 0 0];

robot.plot(q,'trail', '-','lightpos', [1 1 1]);
while(t<10)
    T= fwkin(q(i,:),d,a);
    At=T(1:3,1);
    Ot=T(1:3,2);
    Nt=T(1:3,3);
    Bc=T(1:3,1:2);
    Rt = T(1:3,1:3);
    Pt= T(1:3,4)';
    xc =Bc'*(Pt-Pc)';
    Ax = Bc' *[eye(3,3) ,skew(Pt-Pc)];
    jac = robot.jacobe(q(i,:));
    
    qdot= pinv(jac)* ue';
    Zx=[Nt', zeros(1,3); 
        Rt'*skew(Pt-Pc), Rt'];
    A=Ax*jac;
    xcdot=A*qdot;   
    Z = pinv(jac)*Zx';

    G = null(jac);% G * jac' =0
    N = [Z, G];

    S =[pinv(A) N];

    qdot = S*[xcdot'  vu]';

    q(i+1,:) = q(i,:) + dt*qdot(:,1)';
    
    ue = ([Zx' , zeros(6,1)] * vu' + jac*pinv(A)*xcdot)';

    check = robot.islimit(q(i+1,:));
    if norm(check(:,1)) >=1
        q(i+1,:) = q(i,:);
        ue= -ue;
        vu =-vu;
    end
    t = t+dt;
    i = i+1;
end
robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);

