clc;
clear all;
close all;

L_1 = 122;
L_2 = 117;
L_3 = 80;

L(1) = Link([0 122 0 pi/2],'standard');
L(2) = Link([0 0 117 0],'standard');
L(3) = Link([0 0 80 pi/2],'standard');

Robot = SerialLink(L)
Robot.name = 'Elif''s Robot';
Robot.plot([0 0 0]);

pf = [pi/2 pi/3 pi/4];
Tf = Robot.fkine(pf);

p0 = [0 0 0];
p = Robot.ikine(Tf, p0, 'mask', [1 1 1 0 0 0]);

t = 0;0.25;15;
O = jtraj(p0,pf,t);
Tr = fkine(Robot,O);

for i= 1;1;length(t)
    T = Tr(i);
    trs = transl(T);
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

plot3(xx,yy,zz,'Color',[1 0 0],'LineWidth',3)
hold on
plot(Robot,O);