client = tcpclient("192.168.0.38",3333);

tcp_data = read(client,6);
disp("connected");

setRobotPos(states(:,1)',client);
pause(1);
 
for i = 1:size(states,2)
   setRobotPos(states(:,i)',client);
   pause(0.5);
end


clear client;

function [tcp_data, msg] = setRobotPos(sol,client)
    x = [-pi/2,0,pi/2];
    v = [128,66,32];
 
    sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
    res_sol = sol;
    res_sol([7 8 9]) = sol([10 11 12]);
    res_sol([10 11 12]) = sol([7 8 9]);
    msg = uint8(interp1(x,v,res_sol));
    msg(13) = 16;
    msg(14) = 0;
    %disp(msg)
    write(client,msg);
    tcp_data = read(client,12);
    disp(tcp_data);
end