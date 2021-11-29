
function [soln,tvec] = generateTrajectory(bodyPart,startPos,endPos,step_height,resolution, ik,weights,hConfig,mid_offset)

    halfStart = startPos + (startPos - endPos) / 4 + step_height *[0,0,1]; 
    halfEnd = endPos + (endPos - startPos) / 4 + step_height *[0,0,1];

    %projM = [1 0; 0 0; 0 1];


    points = [startPos;halfStart;halfEnd; endPos];

    tpts = 0:size(points,1)-1;
    tvec = 0:resolution:size(points,1)-1;

    [qx, qd, qdd, pp] = bsplinepolytraj(points(:,1)', [tpts(1) tpts(end)], tvec);
    qy = bsplinepolytraj(points(:,2)', [tpts(1) tpts(end)], tvec);
    qz = bsplinepolytraj(points(:,3)', [tpts(1) tpts(end)], tvec);


    targetPoints = [qx' qy' qz'];
    
    % Convert targetpoints to states
    soln = [];
    qSol = hConfig;
    
%     errTolerance =  norm(targetPoints(2,:) - targetPoints(1,:));
%     ik.SolverParameters.ErrorChangeTolerance = errTolerance;
    %disp(length(targetPoints));
%     disp("start errTol: "+errTolerance);
    for i=1:length(targetPoints)
   
        point = targetPoints(i,:);
        [qSol, solnInfo] = ik(bodyPart,trvec2tform(point),weights,qSol);
        
%         disp("iter: " + solnInfo.Iterations+ " status: "+solnInfo.ExitFlag + "error: "+solnInfo.PoseErrorNorm);
        
        soln = [soln, qSol];
    end

end