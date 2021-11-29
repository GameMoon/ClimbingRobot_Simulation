
% 

%%
% init functions


init;
step_height = 0.03;
default_step_size = 0.04;
step_size = default_step_size;
mid_offset = 0.01; % full step 0.12 = 0.04 + 0.08
resolution = 0.48;
marker_size = 0.025;
cameraPos = [0 0.038 0.015];
%%
%
% feet positions


feetPositions = zeros(12,3);

feetPositions(1,:) = tform2trvec(getTransform(robot,initConfig,"feet1")) - mid_offset*[0,1,0];
feetPositions(4,:) = tform2trvec(getTransform(robot,initConfig,"feet2")) + mid_offset*[0,1,0];
feetPositions(7,:) = tform2trvec(getTransform(robot,initConfig,"feet3")) + mid_offset*[0,1,0];
feetPositions(10,:) = tform2trvec(getTransform(robot,initConfig,"feet4")) - mid_offset*[0,1,0];

for i=1:3:12
    feetPositions(i+1,:) = feetPositions(i,:) + step_size * [0,1,0];
    feetPositions(i+2,:) = feetPositions(i,:) - step_size * [0,1,0];
end

defaultFeetPositions = feetPositions;

% markerPoint = [-0.1 0.10 -0.06];
% markerPoints = [markerPoints; markerPoint];
markerPoints = [];
robotPosition = [0.0 0.0 0.0];

angle = -deg2rad(10);
tic
discont_rotate;
toc

sim("climbingrobot_simulation",toffset);

%%
markerPoints = [];
targetPoint = [0.0 0.0 0.0];
endPoint = [0.0 0.0 0.0];

robotPosition = [0.0 0.0 0.0];
cameraClient = tcpclient("127.0.0.1",3335);

client = tcpclient("192.168.4.1",3333);
read(client,6);

disp("connected");
tcp_data = [];

currentFrame = 0;

updateIndex = 0;
while 1
    
    write(cameraClient,uint8('a'));
%     while (cameraClient.NumBytesAvailable > 0)
    pause(0.1);
    tcp_data = read(cameraClient);
%     disp(tcp_data);
%     if length(tcp_data) < 17
%         pause(2)
%         continue
%     end

%     end

    [markerInfos,latestFrameID] = parseMessage(tcp_data);
%     disp("latestFrameDI: "+latestFrameID);
%     disp("markerinfos:"+markerInfos);
    disp("----------------------");
    if size(markerInfos,1) > 0
        
        isRobotPosUpdate = 0;
%         for i = 1:13:size(markerInfos,1)
        for i = 1:size(markerInfos,1)
%             disp("markerInfo iter: "+i)
            markerPoint = markerInfos(i,2:4);
            markerPoint = (cameraPos + markerPoint)+robotPosition;
            markerPoint(3) = markerPoint(3) * -1;
            
            currentAngle = atan2(endPoint(1)-robotPosition(1),endPoint(2)-robotPosition(2));
            disp("before angle: "+ rad2deg(currentAngle) + "dist: "+norm(endPoint-robotPosition));
            
            if markerInfos(i,1) == 1
                if norm(endPoint) > 0
%                     positionChange = norm(endPoint-markerPoint);
                    updateVector = endPoint - markerPoint;
                    updateVector(3) = 0;
                    updateSize = norm(updateVector);
                    if updateSize > 0.01
                        robotPosition = robotPosition + updateVector;
                        isRobotPosUpdate = 1;
                        disp("robot pos updated");
                    end
                    disp("updateSize: "+updateSize);
%                     disp("updateVector: "+updateVector);
                else 
                    endPoint = markerPoint;                   
                    disp("endPoint set:"+endPoint);
%                     continue;
                end
                
%             else
%                 markerPoint(2) = markerPoint(2) + 0.0325; % obstacle radius
%                 
%                 isSame = 0;
%                 for i = 1:1:size(markerPoints,1)
%                     if norm(markerPoints(i,:)-markerPoint) < 0.01
%                         isSame = 1;
%                         break
%                     end
%                 end
% 
%                 if isSame == 0
%                     markerPoints = [markerPoints; markerPoint];
%                 end              
%                 
            end

        end
        disp("markerPoints: "+markerPoints);
        %disp("Marker pos: " + markerPoint);
%         disp(markerPoint);
%         disp("Dist from marker: "+norm(endPoint-robotPosition));
        
   
%         show(robot,initConfig,'Frames','off');
    
%         scatter3(robotPosition(1),robotPosition(2),robotPosition(3),"blue");
%         hold on;
%         scatter3(endPoint(1),endPoint(2),endPoint(3),"red");
        
%         currentAngle = atan2(endPoint(1)-robotPosition(1),endPoint(2)-robotPosition(2));
%         disp("angle: "+ rad2deg(currentAngle) + "dist: "+norm(endPoint-robotPosition));
        if updateIndex == 2
         if norm(endPoint) > 0
            endDist = norm(endPoint -robotPosition);
            if norm(endPoint -robotPosition) < 0.20
                break
            end
            
            
            disp("latest frame: "+latestFrameID);
            currentAngle = atan2(endPoint(1)-robotPosition(1),endPoint(2)-robotPosition(2));
            disp("after angle: "+ rad2deg(currentAngle) + "dist: "+endDist);
            
            if endDist < 0.3
                step_size = 0.02;
            elseif step_size ~= default_step_size
                step_size = default_step_size;
            end
            
            if abs(currentAngle) > deg2rad(5)
                angle = currentAngle;
                discont_rotate;
                disp("rotate: "+rad2deg(angle));
    %                 pause(3);
                playAnimation(sim_states,client);
            else 
                disp("step");
                discont_step;
    %                 pause(3);
                playAnimation(sim_states,client);
            end
         end
            updateIndex = 0;
        else
                updateIndex = updateIndex + 1;
        end
 
    %         else
    %            disp("first point");
    %            endPoint = markerPoint;
%         end
        
    end
    %discont_step;
    %playAnimation(sim_states,client);
%     pause(4);
%     user_inp = input("waiting for user");
end

clear cameraClient;
clear client;


% show(robot,initConfig,'Frames','off');
% selectedPoint = markerPoints(1,:)-robotPosition;
% hold on;
% scatter3(selectedPoint(1),selectedPoint(2),selectedPoint(3));
% hold off;
%%
client = tcpclient("192.168.4.1",3333);
tcp_data = read(client,6);

disp("connected");

% while 1%norm(markerPoints(1,:)-robotPosition) > 0.1
%     if(norm(markerPoints(1,:)-robotPosition))discont_step;
    angle = -deg2rad(10);%-pi / 12;
    discont_rotate;
    playAnimation(sim_states,client);
%     disp("markerDist: "+norm(markerPoints(1,:)-robotPosition));
%     disp("robotPos: "+robotPosition);
% end

clear client;
