function [tcp_data, msg] = setRobotPos(sol,client)
    x = [-pi/2,0,pi/2];
    v = [128,66,32];
 
    sol([3 6 9 12]) = sol([3 6 9 12]) * -1;
    res_sol = sol;
     res_sol([1 2 3]) = sol([4 5 6]);
     res_sol([4 5 6]) = sol([7 8 9]);
     res_sol([7 8 9]) = sol([1 2 3]);
%      res_sol([10 11 12]) = sol([1 2 3]);
%      res_sol([4 5 6]) = sol([7 8 9]);
%     res_sol([10 11 12]) = sol([7 8 9]);
%     res_sol([1 2 3]) =  sol([1 2 3]);
%     res_sol([4 5 6]) =  sol([10 11 12]);
    
    msg = uint8(interp1(x,v,res_sol,'spline'));
    
%     disp(msg);
    msg(13) = 16;
    msg(14) = 0;
    %disp(msg)
    write(client,msg);
    tcp_data = read(client,12);
%     disp(tcp_data);
%     tcp_data = msg;
end