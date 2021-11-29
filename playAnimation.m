function [tcp_data, msg] = playAnimation(sim_states,client)
	states = sim_states(2:13,:);
    
    for i = 1:size(states,2)
        [adc,msg] = setRobotPos(states(:,i)',client);
        pause(0.05);
    end
    
end