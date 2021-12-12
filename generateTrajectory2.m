

bodyPart = "feet3";
startPos = feetPositions(7+2,:);
endPos = feetPositions(7+1,:);


    halfStart = startPos + (endPos - startPos) / 4 + step_height *[0,0,1]; 
    halfEnd = endPos + (startPos-endPos) / 4 + step_height *[0,0,1];

    %projM = [1 0; 0 0; 0 1];


    points = [startPos;halfStart;halfEnd; endPos];

    tpts = 0:size(points,1)-1;
    tvec = 0:0.1:size(points,1)-1;

    [qx, qd, qdd, pp] = bsplinepolytraj(points(:,1)', [tpts(1) tpts(end)], tvec);
    qy = bsplinepolytraj(points(:,2)', [tpts(1) tpts(end)], tvec);
    qz = bsplinepolytraj(points(:,3)', [tpts(1) tpts(end)], tvec);
    
  

    targetPoints = [qx' qy' qz'];
    
    axis equal;
    figure
    hold on
    scatter3(targetPoints(:,1), targetPoints(:,2), targetPoints(:,3),'filled');
    scatter3(points(:,1),points(:,2),points(:,3),'filled');
    hold off
    % Convert targetpoints to states
%     soln = [];
%     qSol = hConfig;
%     
% %     errTolerance =  norm(targetPoints(2,:) - targetPoints(1,:));
% %     ik.SolverParameters.ErrorChangeTolerance = errTolerance;
%     %disp(length(targetPoints));
% %     disp("start errTol: "+errTolerance);
%     for i=1:length(targetPoints)
%    
%         point = targetPoints(i,:);
%         [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
%         
% %         disp("iter: " + solnInfo.Iterations+ " status: "+solnInfo.ExitFlag + "error: "+solnInfo.PoseErrorNorm);
%         
%         soln = [soln, qSol];
%     end
