function [A,B,C,D] = get_linear_model(eqs_motion,get_y_r,sensors,param)
    % Measure
    f = @(x,u) eqs_motion([],x,u,param);
    h = @(x,u) get_y_r(sense(x,eqs_motion([],x,u,param),sensors,param),x,zeros(size(param.d_names)),param);

    A = numerical_jacobian(@(x) f(x,param.u_0),param.x_0);
    B = numerical_jacobian(@(u) f(param.x_0,u),param.u_0);
    C = numerical_jacobian(@(x) h(x,param.u_0),param.x_0,[],[],[],param.r_is_angle);
    D = numerical_jacobian(@(u) h(param.x_0,u),param.u_0,[],[],[],param.r_is_angle);
    
    controllers.is_controllable(A,B,C)
end


function z = sense(x,x_dot,sensors,param)
    z = zeros(size(param.z_names));
    d = zeros(size(param.d_names));
    for j = 1:length(sensors)
        last_update = sensors(j).last_update;
        last_reading = sensors(j).y_m;
         z(sensors(j).z_indexes) = sensors(j).sense(x,x_dot,d,[]);
        sensors(j).last_update = last_update;
        sensors(j).y_m = last_reading;
    end
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
