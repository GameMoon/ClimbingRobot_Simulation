% client = tcpclient("192.168.1.1",3333);
client = tcpclient("192.168.4.1",3333);

tcp_data = read(client,6);
disp("connected");

states = sim_states(2:13,:);

setRobotPos(states(:,1)',client);
pause(1);
%  
result = []
sent = []
tcounter = 0
for i = 1:size(states,2)
    [adc,msg] = setRobotPos(states(:,i)',client);
%      pause(0.01);
end
% for i = 1:200%size(states,2)
%     [adc,msg] = setRobotPos(states(:,i)',client);
%     result = [ result [tcounter; adc']];
%     sent = [ sent [tcounter; msg']];
%     tcounter= tcounter + 1;
% %      pause(0.0001);
% end
% for i = 200:-1:1%size(states,2)
%      [adc, msg] = setRobotPos(states(:,i)',client);
%      result = [ result [tcounter; adc']];
%      sent = [ sent [tcounter; msg']];
%      tcounter= tcounter + 1;
% %      pause(0.0001);
% end

% hold on;
% tiledlayout(2,1)
% nexttile
% plot(result(1,:),result(11:13,:))
% nexttile
% plot(result(1,:),sent(11:13,:))

% 
% pause(1);
% setRobotPos(states(:,1)',client);
clear client;

function [tcp_data, msg] = setRobotPos(sol,client)
    x = [-pi/2,0,pi/2];
    v = [128,66,32];
 
    sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
    res_sol = sol;
    res_sol([7 8 9]) = sol([4 5 6]);
    res_sol([10 11 12]) = sol([7 8 9]);
    res_sol([1 2 3]) =  sol([1 2 3]);
    res_sol([4 5 6]) =  sol([10 11 12]);
    
    msg = uint8(interp1(x,v,res_sol,'spline'));
    
    disp(msg);
    msg(13) = 16;
    msg(14) = 0;
    %disp(msg)
    write(client,msg);
    tcp_data = read(client,12);
%     disp(tcp_data);
%     tcp_data = msg;
end