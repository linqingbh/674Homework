function [A,B] = get_linear_model(eqs_motion,param)
    A = numerical_jacobian(@(state) eqs_motion([],state,param.u_0,param),param.x_0);
    B = numerical_jacobian(@(input) eqs_motion([],param.x_0,input,param),param.u_0);
end



% classdef linear_model
%     properties
%         core
%         f
%     end
%     methods
%         function [self,A,B,C,D] = linear_model(core)
%             self.core = core;
%             self.f = @(x,u) core.functions.eqs_motion(0,x,u,param);
%             sensors = 
%         end
%         
%         function y_m = h(x,u)
%             x_dot = self.f(x,u);
%             self.core.functions.sensors.sense()
%             
%             z = zeros(size(self.core.param.z_names));
%             for j = 1:length(self.sensors)
%                 z(self.sensors(j).z_indexes) = self.sensors(j).sense(x,x_dot,zeros(self.core.param.d_names,1),-1);
%             end
%             
%             y_m = self.core.functions.get_y_m(z,self.core.param);
%         end
%         
%         function A = get_A()
%             A = numerical_jacobian(@(state) eqs_motion([],state,param.u_0,param),param.x_0);
%         end
%         
%         function B = get_B()
%             B = numerical_jacobian(@(input) eqs_motion([],param.x_0,input,param),param.u_0);
%         end
% 
%         function C = get_C()
%         end
% 
%         function D = get_D()
%         end
%         
%     end
% end
