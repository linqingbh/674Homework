function [states,states_dot] = rk4(fun,time,initial,end_flag)
    if ~exist('end_flag','var'), end_flag = false;end
    states = zeros(length(initial),length(time));
    states_dot = zeros(length(initial),length(time));
    states(:,1) = initial;
    states_dot(:,1) = fun(time(1),initial);
    for i = 2:length(time)
        step = time(i)-time(i-1);
        k1 = fun(time(i),states(:,i-1));
        k2 = fun(time(i),states(:,i-1) + abs(step)/2*k1);
        k3 = fun(time(i),states(:,i-1) + abs(step)/2*k2);
        k4 = fun(time(i),states(:,i-1) + abs(step)*k3);
        states(:,i) = states(:,i-1) + step/6 * (k1 + 2*k2 + 2*k3 + k4);
        states_dot(:,i) = k1;
    end
    if end_flag
        states = states(:,2:end);
        states_dot = states_dot(:,2:end);
    end
end