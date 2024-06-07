% Using Peter Corke ToolBox
clc;
clear all;
close all;

%Connection lengths are determined (Link1,2,3).
L_1 = 122;
L_2 = 117;
L_3 = 80;

%A three-link robot is created using Denavit-Hartenberg parameters.
L(1) = Link([0 122 0 pi/2],'standard');
L(2) = Link([0 0 117 0],'standard');
L(3) = Link([0 0 80 pi/2],'standard');

% a (link uzunluğu): Önceki eklemin Z ekseni boyunca OX eksenine olan mesafesi.
% α (link açısı): Önceki eklemin Z ekseni etrafında OX ekseni etrafında dönme açısı.
% d (eklem uzunluğu): Z ekseni boyunca olan mesafe (eklemin uzunluğu).
% θ (eklem açısı): Eksen etrafında dönme açısı.

%Created robot "Elif's Robot"
Robot = SerialLink(L)
Robot.name = 'Elif''s Robot';

%p0 = [0 pi/2 0]; %rest position
%Robot.plot([0 0 0]);

%Movement timing is defined
t = 0:0.15:3;

%STEP-1 Joints are determined for the starting and ending configurations (q1_sl and q2_sl).
q1_sl = [0 pi/2 0];
q2_sl = [0 pi/4 0];
Q1 = jtraj(q1_sl, q2_sl,t);

% Endpoint positions are extracted for step 1
TRAJ1 = fkine(Robot,Q1); %Forward kinematics are calculated for each configuration and the results are stored in TRAJ1.
for i = 1:1:length(t);
    T1 = TRAJ1(i);
    trsl = transl(T1);
    xxl(i) = trsl(1); %For each time step, the position of the endpoint is extracted from the forward kinematics results and stored in the xxl, yyl, and zzl arrays.
    yyl(i) = trsl(2);
    zzl(i) = trsl(3);
end

%STEP-2 Joints are determined for the starting and ending configurations (q1_s2 and q2_s2).
q1_s2 = [0 pi/4 0];
q2_s2 = [-pi/2 -pi/4 0];
Q2= jtraj(q1_s2, q2_s2,t);

% Endpoint positions are extracted for step 2
TRAJ2 = fkine(Robot,Q2); %Forward kinematics are calculated for each configuration and the results are stored in TRAJ2.
for i = 1:1:length(t);
    T2 = TRAJ2(i);
    trs2 = transl(T2);
    xx2(i) = trs2(1);
    yy2(i) = trs2(2);
    zz2(i) = trs2(3);
end

%STEP-3 Joints are determined for the starting and ending configurations (q1_s3 and q2_s3).
q1_s3 = [-pi/2 -pi/4 0];
q2_s3 = [pi/2 pi/3 pi/4];
Q3= jtraj(q1_s3, q2_s3,t);

%Endpoint positions are extracted for step 3
TRAJ3 = fkine(Robot,Q3);
for i = 1:1:length(t);
    T3 = TRAJ3(i); %Forward kinematics are calculated for each configuration and the results are stored in TRAJ3.
    trs3 = transl(T3);
    xx3(i) = trs3(1);
    yy3(i) = trs3(2);
    zz3(i) = trs3(3);
end

hold on
% Drawing trajectories in 3D space
plot3(xxl,yyl,zzl,'Color',[1 0 0],'LineWidth',3);
plot3(xx2,yy2,zz2,'Color',[0 1 1],'LineWidth',3);
plot3(xx3,yy3,zz3,'Color',[1 0 1],'LineWidth',3);

% Drawing robot configurations
plot(Robot,Q1);
plot(Robot,Q2);
plot(Robot,Q3);