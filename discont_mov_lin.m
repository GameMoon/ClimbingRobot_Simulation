
init();
step_height = 0.03;
step_size = 0.05;
mid_offset = 0.01; % full step 0.12 = 0.04 + 0.08
%%
feet1Soln = generateLegMovement(robot,ik,weights,initConfig,"feet1",step_size,mid_offset);
feet2Soln = generateLegMovement(robot,ik,weights,initConfig,"feet2",step_size,mid_offset);
feet3Soln = generateLegMovement(robot,ik,weights,initConfig,"feet3",step_size,-mid_offset);
feet4Soln = generateLegMovement(robot,ik,weights,initConfig,"feet4",step_size,-mid_offset);
%%

states = [];
% default to start position
feet1CenterPos = tform2trvec(getTransform(robot,initConfig,"feet1")) + mid_offset*[1,0,0];
feet1StartPos = feet1CenterPos + step_size * [1,0,0]; 
feet1EndPos = feet1CenterPos - step_size * [1,0,0]; 

feet2CenterPos = tform2trvec(getTransform(robot,initConfig,"feet2")) + mid_offset*[1,0,0];
feet2StartPos = feet2CenterPos + step_size * [1,0,0]; 
feet2EndPos = feet2CenterPos - step_size * [1,0,0]; 

feet3CenterPos = tform2trvec(getTransform(robot,initConfig,"feet3")) - mid_offset*[1,0,0];
feet3StartPos = feet3CenterPos + step_size * [1,0,0]; 
feet3EndPos = feet3CenterPos - step_size * [1,0,0]; 

feet4CenterPos = tform2trvec(getTransform(robot,initConfig,"feet4")) - mid_offset*[1,0,0];
feet4StartPos = feet4CenterPos + step_size * [1,0,0]; 
feet4EndPos = feet4CenterPos - step_size * [1,0,0]; 

%%
% startPos = ik(bodyPart,trvec2tform(point),weights,qSol)
resolution = 0.12;

startState = [feet4Soln(1:3,1); feet1Soln(4:6,2);feet2Soln(7:9,3);feet3Soln(10:12,1)];
sim_states = [0; startState];
toffset = 0.5;


[soln,tvec] = generateTrajectory("feet2",feet2EndPos,feet2StartPos,step_height, resolution,ik, weights,startState,mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet1",feet1StartPos,feet1EndPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;
%Body move
[body1,tvec] = generateTrajectory("feet1",feet1EndPos,feet1CenterPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feet2StartPos,feet2CenterPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feet3CenterPos,feet3StartPos,0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feet4CenterPos,feet4EndPos,0, resolution,ik, weights,soln(:,end),-mid_offset);
soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet3",feet3StartPos,feet3EndPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet4",feet4EndPos,feet4StartPos,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

%Body move
[body1,tvec] = generateTrajectory("feet1",feet1CenterPos,feet1StartPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feet2CenterPos,feet2EndPos,0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feet3EndPos,feet3CenterPos,0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feet4StartPos,feet4CenterPos,0, resolution,ik, weights,soln(:,end),-mid_offset);
soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;
%%
sim("climbingrobot_simulation",toffset);
%%

function [soln,tvec] = generateTrajectory(bodyPart,startPos,endPos,step_height,resolution, ik,weights,hConfig,mid_offset)
%     n_step_size = -1*step_size;
   
%     startPos = centerPos - n_step_size * [1,0,0]; 
%     endPos = centerPos + n_step_size * [1,0,0];
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
        %solnInfo
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
