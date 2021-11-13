soln = [];
qSol = hConfig;
%show(robot,hConfig,'Frames','off');
%hold on
for i=1:length(targetPoints)
    point = targetPoints(i,:);
    [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
    %solnInfo
    soln = [soln, qSol];
end