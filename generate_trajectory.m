init;
step_height = 0.03;
step_size = 0.05;
mid_offset = 0.01; 

%%
hConfig = initConfig;
bodyPart = "feet1";

function [soln,tvec] = generateTrajectory(robot,ik,weights,hConfig,bodyPart,step_size,step_height,mid_offset)
    n_step_size = -1*step_size;
    centerPos = tform2trvec(getTransform(robot,hConfig,bodyPart)) + mid_offset*[1,0,0];

    startPos = centerPos - n_step_size * [1,0,0]; 
    endPos = centerPos + n_step_size * [1,0,0];
    halfStart = centerPos + (startPos - centerPos) / 2 + step_height *[0,0,1]; 
    halfEnd = centerPos + (endPos - centerPos) / 2 + step_height *[0,0,1];

    %projM = [1 0; 0 0; 0 1];


    points = [startPos;halfStart;halfEnd; endPos];


    tpts = 0:size(points,1)-1;
    tvec = 0:0.1:size(points,1)-1;

    [qx, qd, qdd, pp] = bsplinepolytraj(points(:,1)', tpts, tvec);
    qy = bsplinepolytraj(points(:,2)', tpts, tvec);
    qz = bsplinepolytraj(points(:,3)', tpts, tvec);


    targetPoints = [qx' qy' qz'];

    targetp_to_states;
    sim_states = [tvec; soln];
end
%%
% rc = rateControl(20);
% for i = 1:size(states,2)
%     show(robot,states(:,i),'Frames','off');
%     waitfor(rc);
%     disp(rc)
%     %pause;
% end
% plot(tvec,qx,'x')
% plot(tvec,targetPoints,'x')


% show(robot,hConfig,'Frames','off');
% hold all
% scatter3(qx,qy,qz);
%plot(qd(1,:),qd(2,:))
%plot(qdd(1,:),qdd(2,:))
% hold off
%plot(tvec, qd,'x')
% hold all
% plot(tpts, wpts, 'x')
% xlabel('t')
% ylabel('Positions')
% legend('X-positions','Y-positions')
% hold off

%plot(points2d(:,1),points2d(:,2))
%show(robot,hConfig,'Frames','off');

%disp(size(points,1));

% %%
% hold on;
% for i=1:size(points,1)
%     point = points(i,:);
%     scatter3(point(1),point(2),point(3),'filled');
% end
% hold off;