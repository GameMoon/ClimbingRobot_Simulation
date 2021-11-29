function [feetPositions] = checkMarkerPosition(markerPoints,robotPos,defaultFeetPositions,startState)
    
    markerHeight = 0.055;
    markerSize = 0.05;
    
    feetPositions = defaultFeetPositions;

    for i=1:length(defaultFeetPositions)
        for k =1:size(markerPoints,1)
            dist = norm(defaultFeetPositions(i,:)-(markerPoints(k,:)-robotPos));

            if dist < markerSize
                disp("index: "+i + " dist: "+dist);


                if size(startState) == 0
                    feetPositions(i,:) = feetPositions(i,:) + [0,0,markerHeight];
                elseif ismember(i,startState)
                    disp("startpos: "+i);
                    feetPositions(i,:) = feetPositions(i,:) + [0,0,markerHeight];
                        if rem(i,3) == 0
                           feetPositions(i-1,:) = feetPositions(i-1,:) + [0,0,markerHeight];  
                           feetPositions(i-2,:) = feetPositions(i-2,:) + [0,0,markerHeight];  
                        elseif rem(i,3) == 1
                           feetPositions(i+1,:) = feetPositions(i+1,:) + [0,0,markerHeight];  
                           feetPositions(i+2,:) = feetPositions(i+2,:) + [0,0,markerHeight];  
                        elseif rem(i,3) == 2
                           feetPositions(i+1,:) = feetPositions(i+1,:) + [0,0,markerHeight];  
                           feetPositions(i-1,:) = feetPositions(i-1,:) + [0,0,markerHeight];  
                        end
                end

            end
        end
    end

end