% Rigid body tree oluşturulması
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',5);

% Eklem uzunlukları
L1 = 4;
L2 = 6;
L3 = 5;
L4 = 3;
L5 = 2;

% Link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint, trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

% Link 2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

% Link 3
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');

% Link 4
body = rigidBody('link4');
joint = rigidBodyJoint('joint4','revolute');
setFixedTransform(joint, trvec2tform([L3,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link3');

% Link 5
body = rigidBody('link5');
joint = rigidBodyJoint('joint5','revolute');
setFixedTransform(joint, trvec2tform([L4,0,0]));
joint.JointAxis = [0 0 1]; 
body.Joint = joint;
addBody(robot, body, 'link4');

% End Effector
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L5, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link5');

% Robotun gösterilmesi
show(robot);

% Zaman ve hareket yolu oluşturulması
t = (0:0.2:80)'; % Artırılan süre
count = length(t);
center = [15 -5 0];
radius = 1.75;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% Ters kinematik ve çözümlerin depolanması
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 1, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector, trvec2tform(point), weights, qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

% Animasyonlu gösterim
figure
show(robot, qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1), points(:,2), 'k', 'LineWidth', 1.5)
axis([-15 25 -15 25])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot, qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end