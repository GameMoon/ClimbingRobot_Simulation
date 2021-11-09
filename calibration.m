robot = importrobot("climbingrobot_exported.urdf");
robot.DataFormat = 'column';
hConfig = homeConfiguration(robot);

defaultAngle = pi/8;
for i = 1:3:12
    hConfig(i) = 0;
    hConfig(i+1) = defaultAngle;
    hConfig(i+2) = -defaultAngle;
end
initConfig = hConfig;
%hConfig(12) = hConfig(12)*-1;

show(robot,hConfig,'Frames','off');
%%
client = tcpclient("192.168.0.38",3333);

tcp_data = read(client,6);
disp("connected");


measures = [];
%for i = (-pi/3:0.02:pi/3)
for i = (pi/3:-0.02:-pi/3)
%for i = (pi/8:-0.02:-pi/3)
    %show(robot,hConfig','PreservePlot',false,'Frames','off');
    
   
    hConfig(2) = i;
    hConfig(5) = i;
    hConfig(8) = i;
    hConfig(11) = i;
    
    [tcp_data,msg] = setRobotPos(hConfig,client);
    measures = [measures; [msg(2) tcp_data]];
    pause(0.5);
end

setRobotPos(hConfig,client);

clear client;
% Tibia
%     hConfig(3) = i;
%     hConfig(6) = i;
%     hConfig(9) = i;
%     hConfig(12) = i;
% Femur    
%     hConfig(2)
%     hConfig(5)
%     hConfig(8)
%     hConfig(11)
%%
plot(measures(:,1),measures(:,4));
%hold on
%plot(measures(:,1),measures(:,7)-measures(:,1));
%plot(measures(:,1),measures(:,10)-measures(:,1));
%plot(measures(:,1),measures(:,11)-measures(:,1));
%hold off

%%
plot(measures(:,1),measures(:,4)-measures(:,1));
hold on
plot(measures(:,1),measures(:,5)-measures(:,1));
plot(measures(:,1),measures(:,8)-measures(:,1));
plot(measures(:,1),measures(:,13)-measures(:,1));
hold off

%write(client,uint16(interp1(x,v,sol)));

function [tcp_data, msg] = setRobotPos(sol,client)
    x = [-pi/2,0,pi/2];
    v = [128,66,32];
 
    sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
    msg = uint8(interp1(x,v,sol));
    msg(13) = 16;
    msg(14) = 0;
    %disp(msg)
    write(client,msg);
    tcp_data = read(client,12);
    disp(tcp_data);
end