function [A,B] = get_linear_model(eqs_motion,param)
    A = numerical_jacobian(@(state) eqs_motion([],state,param.u_0,param),param.x_0);
    B = numerical_jacobian(@(input) eqs_motion([],param.x_0,input,param),param.u_0);
end