function [feetPositions] = checkMarkerPosition(markerPoints,robotPos,defaultFeetPositions,startState,forbidState)
    
    markerHeight = 0.04;
    markerSize = 0.06;
    
    feetPositions = defaultFeetPositions;

    for i=1:length(defaultFeetPositions)
        for k =1:size(markerPoints,1)
            dist = norm(defaultFeetPositions(i,1:2)- markerPoints(k,1:2)+ robotPos(1:2));
            
            if dist < markerSize
                
                disp("index: "+i + " dist: "+dist);
                feetPositions(i,:) = feetPositions(i,:) + [0,0,markerHeight];
%                 if ismember(i,forbidState)
%                     feetPositions(i,:) = defaultFeetPositions(i,:);
%                 elseif size(startState) == 0
%                     feetPositions(i,:) = feetPositions(i,:) + [0,0,markerHeight];
%                 elseif ismember(i,startState)
%                     disp("startpos: "+i);
%                     feetPositions(i,:) = feetPositions(i,:) + [0,0,markerHeight];
%                         if rem(i,3) == 0
%                            feetPositions(i-1,:) = feetPositions(i-1,:) + [0,0,markerHeight];  
%                            feetPositions(i-2,:) = feetPositions(i-2,:) + [0,0,markerHeight];  
%                         elseif rem(i,3) == 1
%                            feetPositions(i+1,:) = feetPositions(i+1,:) + [0,0,markerHeight];  
%                            feetPositions(i+2,:) = feetPositions(i+2,:) + [0,0,markerHeight];  
%                         elseif rem(i,3) == 2
%                            feetPositions(i+1,:) = feetPositions(i+1,:) + [0,0,markerHeight];  
%                            feetPositions(i-1,:) = feetPositions(i-1,:) + [0,0,markerHeight];  
%                         end
%                 end

            end
        end
    end

end