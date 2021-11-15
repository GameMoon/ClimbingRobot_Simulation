
init();
step_height = 0.03;
step_size = 0.00;
mid_offset = 0.00; % full step 0.12 = 0.04 + 0.08
angle = -pi/12;
resolution = 0.12;

%%
feet1Soln = generateLegMovement(robot,ik,weights,initConfig,"feet1",step_size,mid_offset);
feet2Soln = generateLegMovement(robot,ik,weights,initConfig,"feet2",step_size,mid_offset);
feet3Soln = generateLegMovement(robot,ik,weights,initConfig,"feet3",step_size,-mid_offset);
feet4Soln = generateLegMovement(robot,ik,weights,initConfig,"feet4",step_size,-mid_offset);
%%



rotateM = [ cos(angle) -sin(angle) 0;
            sin(angle) cos(angle) 0;
            0 0 1;];

states = [];
% default to start position
feet1CenterPos = tform2trvec(getTransform(robot,initConfig,"feet1")) + mid_offset*[1,0,0];
feet1StartPos = feet1CenterPos + step_size * [1,0,0]; 
feet1EndPos = feet1CenterPos - step_size * [1,0,0]; 
feet1RotatedPos = feet1EndPos * rotateM;


feet2CenterPos = tform2trvec(getTransform(robot,initConfig,"feet2")) + mid_offset*[1,0,0];
feet2StartPos = feet2CenterPos + step_size * [1,0,0]; 
feet2EndPos = feet2CenterPos - step_size * [1,0,0]; 
feet2RotatedPos = feet2StartPos * rotateM;

feet3CenterPos = tform2trvec(getTransform(robot,initConfig,"feet3")) - mid_offset*[1,0,0];
feet3RotatedPos = feet3CenterPos * rotateM;


feet4CenterPos = tform2trvec(getTransform(robot,initConfig,"feet4")) - mid_offset*[1,0,0];
feet4RotatedPos = feet4CenterPos * rotateM;

            

% feetPoints = [  feet2EndPos; feet2RotatedPos;];
%                 feet4CenterPos;
%                feet1RotatedPos; feet2RotatedPos; feet3RotatedPos; feet4RotatedPos;];
    


% 
% show(robot,initConfig,'Frames','off');
% hold on;
% for i=1:length(feetPoints)
%     point = feetPoints(i,:);
%     scatter3(point(1),point(2),point(3),'filled');
% end
% hold off;
%%
% startPos = ik(bodyPart,trvec2tform(point),weights,qSol)

startState = [feet4Soln(1:3,1); feet1Soln(4:6,2);feet2Soln(7:9,3);feet3Soln(10:12,1)];
sim_states = [0; startState];
toffset = 0.5;


[soln,tvec] = generateTrajectory("feet3",feet3CenterPos,feet3RotatedPos,step_height, resolution,ik, weights,startState,-mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;


[soln,tvec] = generateTrajectory("feet2",feet2EndPos,feet2RotatedPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet1",feet1StartPos,feet1RotatedPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;









[soln,tvec] = generateTrajectory("feet4",feet4CenterPos,feet4RotatedPos,step_height, resolution,ik, weights,soln(:,end),-mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

% 
% 
% 
% % %Body move
[body1,tvec] = generateTrajectory("feet1",feet1RotatedPos,feet1StartPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feet2RotatedPos,feet2EndPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feet3RotatedPos,feet3CenterPos,0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feet4RotatedPos,feet4CenterPos,0, resolution,ik, weights,soln(:,end),-mid_offset);

soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

% % [soln,tvec] = generateTrajectory("feet3",feet3RotatedPos,feet3CenterPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
% % sim_states = [sim_states [tvec+toffset; soln]];
% % toffset = toffset + tvec(end) + resolution;
% 



sim("climbingrobot_simulation",toffset);
%%

function [soln,tvec] = generateTrajectory(bodyPart,startPos,endPos,step_height,resolution, ik,weights,hConfig,mid_offset)
    halfStart = startPos + (startPos - endPos) / 4 + step_height *[0,0,1]; 
    halfEnd = endPos + (endPos - startPos) / 4 + step_height *[0,0,1];

    %projM = [1 0; 0 0; 0 1];


    points = [startPos;halfStart;halfEnd; endPos];


    tpts = 0:size(points,1)-1;
    tvec = 0:resolution:size(points,1)-1;

    [qx, qd, qdd, pp] = bsplinepolytraj(points(:,1)', tpts, tvec);
    qy = bsplinepolytraj(points(:,2)', tpts, tvec);
    qz = bsplinepolytraj(points(:,3)', tpts, tvec);


    targetPoints = [qx' qy' qz'];
    
    % Convert targetpoints to states
    soln = [];
    qSol = hConfig;

    for i=1:length(targetPoints)
        point = targetPoints(i,:);
        [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
        soln = [soln, qSol];
    end
end

function [soln] = generateLegMovement(robot,ik,weights,hConfig,bodyPart,step_size,mid_offset)
  
    centerPos = tform2trvec(getTransform(robot,hConfig,bodyPart)) + mid_offset*[1,0,0];
    startPos =  centerPos + step_size * [1,0,0];
    endPos =  centerPos - step_size * [1,0,0];
    
    targetPoints = [centerPos; startPos; endPos; ];   

    soln = [];
    qSol = hConfig;
    for i=1:length(targetPoints)
        point = targetPoints(i,:);
        [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
        soln = [soln, qSol];
    end
end
