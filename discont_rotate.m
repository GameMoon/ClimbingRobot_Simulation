
rotateM = [ cos(angle/2) -sin(angle/2) 0;
            sin(angle/2) cos(angle/2) 0;
            0 0 1;];



% default to start position

feet1StartState = ik("feet1",trvec2tform(feetPositions(1+0,:)),weights,initConfig);
feet2StartState = ik("feet2",trvec2tform(feetPositions(4+0,:)),weights,initConfig);
feet3StartState = ik("feet3",trvec2tform(feetPositions(7+2,:)),weights,initConfig);
feet4StartState = ik("feet4",trvec2tform(feetPositions(10+2,:)),weights,initConfig);

%%
startState = [feet4StartState(1:3); feet1StartState(4:6); feet2StartState(7:9); feet3StartState(10:12)];
sim_states = [0; startState];
toffset = resolution;


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


%Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1+0,:) * rotateM,feetPositions(1+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4+0,:) * rotateM,feetPositions(4+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7+1,:) * rotateM,feetPositions(7+1,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10+1,:) * rotateM,feetPositions(10+1,:),0, resolution,ik, weights,soln(:,end),-mid_offset);

soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;

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


%Body move
[body1,tvec] = generateTrajectory("feet1",feetPositions(1+0,:) * rotateM,feetPositions(1+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body2,tvec] = generateTrajectory("feet2",feetPositions(4+0,:) * rotateM,feetPositions(4+0,:),0, resolution,ik, weights,soln(:,end),mid_offset);
[body3,tvec] = generateTrajectory("feet3",feetPositions(7+2,:) * rotateM,feetPositions(7+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);
[body4,tvec] = generateTrajectory("feet4",feetPositions(10+2,:) * rotateM,feetPositions(10+2,:),0, resolution,ik, weights,soln(:,end),-mid_offset);

soln = [body4(1:3,:);body1(4:6,:);body2(7:9,:);body3(10:12,:)];
sim_states = [sim_states [tvec+toffset; soln]];
toffset = toffset + tvec(end) + resolution;
    
    

