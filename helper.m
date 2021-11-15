%% Calculate max step size

bodyPart="feet3";
feetTr = tform2trvec(getTransform(robot,initConfig,bodyPart));
step_size = 0.10;

while step_size > 0
      qInit = initConfig;
      newFeet = feetTr + step_size * [0,1,0];
      rng(0);
      [qSol, solnInfo] = ik(bodyPart,trvec2tform(newFeet),weights,qInit);
      if strcmp(solnInfo.Status,'success')
          solnInfo;
          show(robot,qSol,'Frames','off');
          break;
      else
          step_size = step_size - 0.01
      end
      disp(step_size);
end

%%
bodyPart="feet1";
feetTr = tform2trvec(getTransform(robot,initConfig,bodyPart));
step_height = 0.05;
while step_height > 0
      qInit = initConfig;
      newFeet = feetTr + step_height * [0,0,1] + step_size/2 * [0,1,0];
      rng(0);
      [qSol, solnInfo] = ik(bodyPart,trvec2tform(newFeet),weights,qInit);
      if strcmp(solnInfo.Status,'success')
          qs(i,:) = qSol;
          qInit = qSol;
          solnInfo;
          break;
      else
          step_height = step_height - 0.01
      end
end

%%
bodyPart="feet1";
feetTr = tform2trvec(getTransform(robot,initConfig,bodyPart)) + mid_offset*[1,1,0]+ step_size * [1,0,0];
step_width = 0.20;
step_size = 0.05;

while step_height > 0
      qInit = initConfig;
      newFeet = feetTr + step_width * [0,1,0];
      rng(0);
      [qSol, solnInfo] = ik(bodyPart,trvec2tform(newFeet),weights,qInit);
      if strcmp(solnInfo.Status,'success')
          qs(i,:) = qSol;
          qInit = qSol;
          solnInfo;
          break;
      else
          step_width = step_width - 0.01
      end
end

%% Show movement
newFeet = feetTr + step_size * [0,1,0] + step_height * [ 0, 0, 1];
newFeet2 = feetTr + -step_size * [0,1,0];
show(robot,qSol,'Frames','off');
hold on;
scatter3(feetTr(1),feetTr(2),feetTr(3),'filled');
scatter3(newFeet(1),newFeet(2),newFeet(3),'filled');
scatter3(newFeet2(1), newFeet2(2),newFeet2(3),'filled');
hold off;


%% Draw center of mass

feet1 = tform2trvec(getTransform(robot,qSol,"feet1"));
feet2 = tform2trvec(getTransform(robot,qSol,"feet2"));
feet3 = tform2trvec(getTransform(robot,qSol,"feet3"));
feet4 = tform2trvec(getTransform(robot,qSol,"feet4"));

drawCenterMass(robot,qSol,feet1,feet2,feet4);


function drawCenterMass(robot,hConfig,feet1, feet2, feet3)
    show(robot,hConfig,'Frames','off');
    [comLocation,comJac] = centerOfMass(robot,hConfig);
    hold on;
    scatter3(comLocation(1),comLocation(2),comLocation(3),'filled');
    line([feet1(1),feet2(1),feet3(1),feet1(1)], ...
    [feet1(2),feet2(2),feet3(2),feet1(2)], ...
    [feet1(3),feet2(3),feet3(3),feet1(3)]);
    hold off;
end