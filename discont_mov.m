
init();
step_height = 0.03;
step_size = 0.05;
mid_offset = 0.01; % full step 0.12 = 0.04 + 0.08
%%
feet1Soln = generateLegMovement(robot,ik,weights,hConfig,"feet1",step_size,step_height,mid_offset);
feet2Soln = generateLegMovement(robot,ik,weights,hConfig,"feet2",step_size,step_height,mid_offset);
feet3Soln = generateLegMovement(robot,ik,weights,hConfig,"feet3",step_size,step_height,-mid_offset);
feet4Soln = generateLegMovement(robot,ik,weights,hConfig,"feet4",step_size,step_height,-mid_offset);
%%

states = [];
% default to start position
states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,1)]];

% period 1
states = [states [feet4Soln(1:3,3); feet1Soln(4:6,3);feet2Soln(7:9,1);feet3Soln(10:12,1)]];
states = [states [feet4Soln(1:3,3); feet1Soln(4:6,2);feet2Soln(7:9,1);feet3Soln(10:12,1)]];
states = [states [feet4Soln(1:3,3); feet1Soln(4:6,4);feet2Soln(7:9,1);feet3Soln(10:12,1)]];
states = [states [feet4Soln(1:3,2); feet1Soln(4:6,4);feet2Soln(7:9,1);feet3Soln(10:12,1)]];
states = [states [feet4Soln(1:3,4); feet1Soln(4:6,4);feet2Soln(7:9,1);feet3Soln(10:12,1)]];

states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,3);feet3Soln(10:12,3)]]; % Body move

states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,2);feet3Soln(10:12,3)]];
states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,4);feet3Soln(10:12,3)]];
states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,4);feet3Soln(10:12,2)]];
states = [states [feet4Soln(1:3,1); feet1Soln(4:6,1);feet2Soln(7:9,4);feet3Soln(10:12,4)]];

states = [states [feet4Soln(1:3,3); feet1Soln(4:6,3);feet2Soln(7:9,1);feet3Soln(10:12,1)]]; % Body move

% states = [states [feet4Soln(1:3,6); feet1Soln(4:6,3);feet2Soln(7:9,3);feet3Soln(10:12,3)]]; % Body move
% states = [states [feet4Soln(1:3,4); feet1Soln(4:6,3);feet2Soln(7:9,3);feet3Soln(10:12,3)]]; % Body move
% 
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,6);feet2Soln(7:9,1);feet3Soln(10:12,3)]];
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,4);feet2Soln(7:9,1);feet3Soln(10:12,3)]];
% states = [states [feet4Soln(1:3,3); feet1Soln(4:6,2);feet2Soln(7:9,3);feet3Soln(10:12,3)]];
% states = [states [feet4Soln(1:3,3); feet1Soln(4:6,4);feet2Soln(7:9,3);feet3Soln(10:12,3)]];

% states = [states [feet4Soln(1:3,3); feet1Soln(4:6,4);feet2Soln(7:9,2);feet3Soln(10:12,3)]];
%states = [states [feet4Soln(1:3,3); feet1Soln(4:6,4);feet2Soln(7:9,4);feet3Soln(10:12,3)]];



% states = [states [feet4Soln(1:3,2); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,3)]];% leg fwd
% states = [states [feet4Soln(1:3,4); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,3)]];
% states = [states [feet4Soln(1:3,4); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,2)]];% leg fwd
% states = [states [feet4Soln(1:3,4); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,4)]];
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,3);feet2Soln(7:9,3);feet3Soln(10:12,1)]]; % Body move
% 
% %period 2
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,2);feet2Soln(7:9,3);feet3Soln(10:12,1)]]; % leg fwd
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,4);feet2Soln(7:9,3);feet3Soln(10:12,1)]];
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,4);feet2Soln(7:9,2);feet3Soln(10:12,1)]]; % leg fwd
% states = [states [feet4Soln(1:3,1); feet1Soln(4:6,4);feet2Soln(7:9,4);feet3Soln(10:12,1)]];
% states = [states [feet4Soln(1:3,3); feet1Soln(4:6,1);feet2Soln(7:9,1);feet3Soln(10:12,3)]]; % Body move

%show(robot,states(:,end),'Frames','off');

figure;
peaks;
view(2)
%set(gcf,'Visible','on');
rc = rateControl(20);
for i = 1:size(states,2)
    show(robot,states(:,i),'Frames','off');
    waitfor(rc);
    %pause;
end
%hold off;
%%
% interactiveRigidBodyTree(robot,'MarkerBodyName','feet1','Frames','off');

function [soln] = generateLegMovement(robot,ik,weights,hConfig,bodyPart,step_size,step_height,mid_offset)
    step_size = -1*step_size;
    feetPos = tform2trvec(getTransform(robot,hConfig,bodyPart)) + mid_offset*[1,0,0];
    centerPos = feetPos  + step_height *[0,0,1] ;
    endForw =  feetPos + step_size * [1,0,0];
    endBack =  feetPos - step_size * [1,0,0];
    halfForw = feetPos + (endForw - feetPos) / 2 + step_height *[0,0,1]; 
    halfBack = feetPos + (endBack - feetPos) / 2 + step_height *[0,0,1];
    
    targetPoints = [feetPos; centerPos; endForw; endBack; halfForw; halfBack];   

    soln = [];
    qSol = hConfig;
    %show(robot,hConfig,'Frames','off');
    %hold on
    for i=1:length(targetPoints)
        point = targetPoints(i,:);
        %scatter3(point(1),point(2),point(3),'filled');
        [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
        %solnInfo
        soln = [soln, qSol];
    end
    %hold off;
end