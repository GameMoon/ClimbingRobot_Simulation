% default to start position

% Check obstacles
feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[1 4 9 12]);

feet1StartState = ik("feet1",trvec2tform(feetPositions(1+0,:)),weights,initConfig);
feet2StartState = ik("feet2",trvec2tform(feetPositions(4+0,:)),weights,initConfig);
feet3StartState = ik("feet3",trvec2tform(feetPositions(7+2,:)),weights,initConfig);
feet4StartState = ik("feet4",trvec2tform(feetPositions(10+2,:)),weights,initConfig);

%%

startState = [feet4StartState(1:3); feet1StartState(4:6); feet2StartState(7:9); feet3StartState(10:12)];
sim_states = [0; startState];
toffset = resolution;

% PERIOD 1
[soln,tvec] = generateTrajectory("feet3",feetPositions(7+2,:),feetPositions(7+1,:),step_height, resolution,ik, weights,startState,mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet4",feetPositions(10+2,:),feetPositions(10+1,:),step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[1 4 8 10]);
% Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1,:),feetPositions(1+2,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4,:),feetPositions(4+2,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7+1,:), feetPositions(7,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10+1,:),feetPositions(10,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;


% UPDATE robot position
%robotPosition = robotPosition + (step_size + 0.01)* [0,1,0];
% Check obstacles
feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[]);

% PERIOD 2 ----------------------------

[soln,tvec] = generateTrajectory("feet2",feetPositions(4+2,:),feetPositions(4+1,:),step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

[soln,tvec] = generateTrajectory("feet1",feetPositions(1+2,:),feetPositions(1+1,:),step_height, resolution,ik, weights,soln(:,end),mid_offset);
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;


feetPositions = checkMarkerPosition(markerPoints,robotPosition,defaultFeetPositions,[2 5 7 10]);
% Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1+1,:),feetPositions(1,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4+1,:),feetPositions(4,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7,:),feetPositions(7+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10,:),feetPositions(10+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

%robotPosition = robotPosition + (step_size + 0.01)* [0,1,0];

