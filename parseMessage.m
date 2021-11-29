function [markerInfos,latestFrameID] = parseMessage(tcp_data)

    markerInfos = [];
    tempInfos = [];
    latestFrameID = 0;
    
    if rem(length(tcp_data),17) ~= 0
        return
    end
        
    
    for i = 1:17:length(tcp_data)
        
        markerID = double(tcp_data(i));
        posX = typecast(uint8(tcp_data(i+1:i+4)),'single');
        posZ = typecast(uint8(tcp_data(i+5:i+8)),'single');
        posY = typecast(uint8(tcp_data(i+9:i+12)),'single');
        
        frameID = double(typecast(uint8(tcp_data(i+13:i+16)),'int32'));%%double(tcp_data(i+13,i+16));
        latestFrameID = frameID;
        tempInfos =  [tempInfos; [markerID posX posY posZ frameID]];
    end
    
    % select only the latest frame
    for i = 1:size(tempInfos,1)
        if tempInfos(i,5) == latestFrameID
            markerInfos = [markerInfos; tempInfos(i,:)];
        end
    end
end