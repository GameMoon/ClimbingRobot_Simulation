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