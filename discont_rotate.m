
rotateM = [ cos(angle/2) -sin(angle/2) 0;
            sin(angle/2) cos(angle/2) 0;
            0 0 1;];
feetPositions = zeros(12,3);

feetPositions(1,:) = tform2trvec(getTransform(robot,initConfig,"feet1")) - mid_offset*[0,1,0];
feetPositions(4,:) = tform2trvec(getTransform(robot,initConfig,"feet2")) + mid_offset*[0,1,0];
feetPositions(7,:) = tform2trvec(getTransform(robot,initConfig,"feet3")) + mid_offset*[0,1,0];
feetPositions(10,:) = tform2trvec(getTransform(robot,initConfig,"feet4")) - mid_offset*[0,1,0];

for i=1:3:12
    feetPositions(i+1,:) = feetPositions(i,:) + step_size * [0,1,0];
    feetPositions(i+2,:) = feetPositions(i,:) - step_size * [0,1,0];
end

defaultFeetPositions = feetPositions;


% default to start position

feet1StartState = ik("feet1",trvec2tform(feetPositions(1+0,:)),weights,initConfig);
feet2StartState = ik("feet2",trvec2tform(feetPositions(4+0,:)),weights,initConfig);
feet3StartState = ik("feet3",trvec2tform(feetPositions(7+2,:)),weights,initConfig);
feet4StartState = ik("feet4",trvec2tform(feetPositions(10+2,:)),weights,initConfig);

%%
startState = [feet4StartState(1:3); feet1StartState(4:6); feet2StartState(7:9); feet3StartState(10:12)];
sim_states = [0; startState];
toffset = resolution;

feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[],[1 4 8 12]);

[soln,tvec] = generateTrajectory("feet2",feetPositions(4+0,:),feetPositions(4+0,:) * rotateM,step_height, resolution,ik, weights,startState,-mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;


[soln,tvec] = generateTrajectory("feet3",feetPositions(7+2,:),feetPositions(7+1,:) * rotateM,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet4",feetPositions(10+2,:),feetPositions(10+1,:) * rotateM,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet1",feetPositions(1+0,:),feetPositions(1+0,:) * rotateM,step_height, resolution,ik, weights,soln(:,end),-mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[1 4 8 11],[]);
%Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1+0,:) * rotateM,feetPositions(1+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4+0,:) * rotateM,feetPositions(4+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7+1,:) * rotateM,feetPositions(7+1,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10+1,:) * rotateM,feetPositions(10+1,:),0, resolution,ik, weights,soln(:,end),-mid_offset);

soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;


feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[],[1 4 8 11]);
% PHASE 2 

[soln,tvec] = generateTrajectory("feet1",feetPositions(1+0,:),feetPositions(1+0,:)*rotateM,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet4",feetPositions(10+1,:),feetPositions(10+2,:)*rotateM,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet3",feetPositions(7+1,:),feetPositions(7+2,:)*rotateM,step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet2",feetPositions(4+0,:),feetPositions(4+0,:)*rotateM,step_height, resolution,ik, weights,soln(:,end),-mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[1 4 8 12],[]);
%Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1+0,:) * rotateM,feetPositions(1+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4+0,:) * rotateM,feetPositions(4+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7+2,:) * rotateM,feetPositions(7+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10+2,:) * rotateM,feetPositions(10+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);

soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;
    
    

