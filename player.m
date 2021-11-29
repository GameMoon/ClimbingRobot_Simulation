% client = tcpclient("192.168.1.1",3333);
client = tcpclient("192.168.4.1",3333);

tcp_data = read(client,6);
disp("connected");

states = sim_states(2:13,:);

setRobotPos(states(:,1)',client);
% pause(2);

%  
% result = []
% sent = []
% tcounter = 0
% for i = 1:size(states,2)
%     [adc,msg] = setRobotPos(states(:,i)',client);
% %      pause(0.01);
% end
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

