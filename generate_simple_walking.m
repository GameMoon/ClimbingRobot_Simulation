robot = importrobot("climbingrobot_exported/climbingrobot_exported.urdf");
%%
interactiveRigidBodyTree(robot,'MarkerBodyName','magnet11','Frames','off');
%% Init position

% initPos = zeros(1,16);
initPos = zeros(1,12);
robot.DataFormat = 'column';
hConfig = homeConfiguration(robot);

defaultAngle = pi/8;
for i = 1:3:12
    initPos(i) = 0;
    initPos(i+1) = defaultAngle;
    initPos(i+2) = -defaultAngle;
end

hConfig = initPos;
initConfig = hConfig;


ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];

show(robot,hConfig','Frames','off');

%%

% 
% 
% tform = getTransform(robot,initConfig,"feet1")*trvec2tform([0,0,0.03]);
% [configSoln,solInfo] = ik('feet1',tform,weights,initConfig);
% show(robot,configSoln,'Frames','off');

%% Animation



% Reset joints


t = (0:0.5:10)'; % Time
count = length(t);
step_size = 0.24;
theta = -step_size + t*(step_size*2/t(end));

feetTr = tform2trvec(getTransform(robot,initConfig',"magnet41"));
d = [0,0.1,0];

points = feetTr +  theta * d;


ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];
q0 = hConfig;
qs = zeros(count,12);

qInit = initPos';


newFeet = feetTr + 0.09 * [0,1,0];
figure
%show(robot,hConfig','Frames','off');
show(robot,qSol,'Frames','off');
hold on
scatter3(feetTr(1),feetTr(2),feetTr(3),'filled');
scatter3(newFeet(1),newFeet(2),newFeet(3),'filled');
[comLocation,comJac] = centerOfMass(robot);
scatter3(comLocation(1),comLocation(2),comLocation(3),'filled');
hold off;
%%

%plot3(feetTr(1),feetTr(2),feetTr(3),newFeet(1),newFeet(2),newFeet(3))

%% Calculate step length

bodyName
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];
feetTr = tform2trvec(getTransform(robot,initConfig,"magnet41"));
step_size = 0.5;
qInit = initConfig;
while step_size > 0
      newFeet = feetTr + step_size * [0,1,0];
      [qSol, solnInfo] = ik('magnet41',trvec2tform(newFeet),weights,qInit);
      if strcmp(solnInfo.Status,'success')
          qs(i,:) = qSol;
          qInit = qSol;
          solnInfo;
          break;
      else
          step_size = step_size - 0.01
      end
end


%%
% for i= 1:count
%       point = points(i,:);
%       [qSol, solnInfo] = ik('magnet41',trvec2tform(point),weights,qInit);
%       if strcmp(solnInfo.Status,'success')
%           qs(i,:) = qSol;
%           qInit = qSol;
%       else
%           lastPoint = point;
%       end
% end
%%
show(robot,qs(20,:)','Frames','off');
%%
setRobotPos(initConfig);
%%
setRobotPos(qs(20,:));
%%
setRobotPos(qs(1,:));

%% Stepping

step_size = 0.09;
step_height = 0.02;
qSol = initConfig;

% First move 
feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet3"));
newPoint = feet1Tr + [0 -step_size 0];
[qSol, solnInfo] = ik('feet3',trvec2tform(newPoint),weights,initConfig');

feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet2"));
newPoint = feet1Tr + [0 step_size 0];
[qSol, solnInfo] = ik('feet2',trvec2tform(newPoint),weights,qSol);

setRobotPos(qSol);

for i = 1:4
    %
    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet3"));
    newPoint = feet1Tr + [0 step_size step_height];
    [phase_11, solnInfo] = ik('feet3',trvec2tform(newPoint),weights,qSol);
    newPoint = feet1Tr + [0 step_size 0];
    setRobotPos(phase_11);
    [phase_11, solnInfo] = ik('feet3',trvec2tform(newPoint),weights,qSol);
    setRobotPos(phase_11);

    %
    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet2"));
    newPoint = feet1Tr + [0 -step_size step_height];
    [phase_12, solnInfo] = ik('feet2',trvec2tform(newPoint),weights,phase_11);
    setRobotPos(phase_12);
    newPoint = feet1Tr + [0 -step_size 0];
    [phase_12, solnInfo] = ik('feet2',trvec2tform(newPoint),weights,phase_11);
    setRobotPos(phase_12);


    %

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet2"));
    newPoint = feet1Tr + [0 0 0];
    [phase_13, solnInfo] = ik('feet2',trvec2tform(newPoint),weights,phase_12);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet3"));
    newPoint = feet1Tr + [0 0 0];
    [phase_13, solnInfo] = ik('feet3',trvec2tform(newPoint),weights,phase_13);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet1"));
    newPoint = feet1Tr + [0 -step_size 0];
    [phase_13, solnInfo] = ik('feet1',trvec2tform(newPoint),weights,phase_13);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet4"));
    newPoint = feet1Tr + [0 step_size 0];
    [phase_13, solnInfo] = ik('feet4',trvec2tform(newPoint),weights,phase_13);

    setRobotPos(phase_13);

    %
    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet4"));
    newPoint = feet1Tr + [0 -step_size step_height];
    [phase_21, solnInfo] = ik('feet4',trvec2tform(newPoint),weights,phase_13);
    setRobotPos(phase_21);
    newPoint = feet1Tr + [0 -step_size 0];
    [phase_21, solnInfo] = ik('feet4',trvec2tform(newPoint),weights,phase_13);
    setRobotPos(phase_21);
    
    %
    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet1"));
    newPoint = feet1Tr + [0 step_size step_height];
    [phase_22, solnInfo] = ik('feet1',trvec2tform(newPoint),weights,phase_21);
    setRobotPos(phase_22);
    newPoint = feet1Tr + [0 step_size 0];
    [phase_22, solnInfo] = ik('feet1',trvec2tform(newPoint),weights,phase_21);
    setRobotPos(phase_22);
    
    %
    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet2"));
    newPoint = feet1Tr + [0 step_size 0];
    [phase_23, solnInfo] = ik('feet2',trvec2tform(newPoint),weights,phase_22);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet3"));
    newPoint = feet1Tr + [0 -step_size 0];
    [phase_23, solnInfo] = ik('feet3',trvec2tform(newPoint),weights,phase_23);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet1"));
    newPoint = feet1Tr + [0 0 0];
    [phase_23, solnInfo] = ik('feet1',trvec2tform(newPoint),weights,phase_23);

    feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet4"));
    newPoint = feet1Tr + [0 0 0];
    [phase_23, solnInfo] = ik('feet4',trvec2tform(newPoint),weights,phase_23);

    setRobotPos(phase_23);
    qSol = phase_23;
end
%%
% 
% pause(1.0);
% feet1Tr = tform2trvec(getTransform(robot,initConfig',"feet4"));
% newPoint = feet1Tr + [0 -step_size 0];
% [qSol, solnInfo] = ik('feet4',trvec2tform(newPoint),weights,initConfig');
% setRobotPos(qSol);
% pause(1.0);
% 
% 
% feet1Tr = tform2trvec(getTransform(robot,qSol,"feet1"));
% newPoint = feet1Tr + [0 step_size 0];
% [qSol, solnInfo] = ik('feet1',trvec2tform(newPoint),weights,qSol);
% setRobotPos(qSol);



%%

%tibia_x = [-pi/2,0,pi/2];



servo_pos = interp1(x,v,qs(1:0));
%servo_pos([3 6 9 12]) = interp1(tibia_x,v,); 

%%



figure
%show(robot,qs(20,:)','Frames','off');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3),'k');

%%
setRobotPos(initFonci)
%%
% for i = 1:count
%     sol = qs(i,:)
%     sol(12) = sol(12) * -1;
%     sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
%     write(client,uint16(interp1(x,v,sol)));
%     pause(0.3);
% end

%%
framesPerSecond = 30;
r = rateControl(framesPerSecond);

figure
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false,'Frames','off');
    waitfor(r);
end

% figure
% set(gcf,'Visible','on');
% rc = rateControl(30);
% for i = 1:count
%     show(robot,qs(i,:)','FastUpdate',true,'PreservePlot',false);
%     waitfor(rc);
% end

%%
setRobotPos(qSol);
%%
function setRobotPos(sol)
    client = tcpclient("192.168.0.38",3333);
    x = [-pi/2,0,pi/2];
    v = [128,66,32];
    sol(12) = sol(12) * -1;
    
    sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
    msg = uint8(interp1(x,v,sol));
    msg(13) = 16;
    msg(14) = 0;
    disp(msg)
    write(client,msg);
    pause(0.4);
end