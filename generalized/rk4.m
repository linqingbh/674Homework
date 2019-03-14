function [state,state_dot,optional_outputs] = rk4(fun,t,initial)
    
    step = t(2)-t(1);
    k1 = fun(t(1),initial);
    k2 = fun(t(1)+step/2,initial + abs(step)/2*k1);
    k3 = fun(t(2),initial + abs(step)/2*k2);
    k4 = fun(t(2),initial + abs(step)*k3);
    state = initial + step/6 * (k1 + 2*k2 + 2*k3 + k4);
    
    if nargout == 3
        [state_dot,optional_outputs] = fun(t(2),state);
    elseif nargout == 2
        state_dot = fun(t(2),state);
    end
end