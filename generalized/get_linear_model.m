function [A,B,C,D] = get_linear_model(eqs_motion,get_y_r,sensors,param)
    f = @(x,u) eqs_motion([],x,u,param);
    h = @(x,u) get_y_r(sense(x,eqs_motion([],x,u,param),sensors,param),x,zeros(size(param.d_names)),param);
    
%     f = @(x,u) y_as_x(x,u,eqs_motion,get_y_r,sensors,param);
%     h = @(x,~) x;

    A = numerical_jacobian(@(x) f(x,param.u_0),param.x_0);
    B = numerical_jacobian(@(u) f(param.x_0,u),param.u_0);
    C = numerical_jacobian(@(x) h(x,param.u_0),param.x_0,[],[],[],param.r_is_angle);
    D = numerical_jacobian(@(u) h(param.x_0,u),param.u_0,[],[],[],param.r_is_angle);
end

function x = y2x(y_r,y_r_dot)
x(1,1) = 0;
x(2,1) = 0;
x(3,1) = -y_r(3);
x(4,1) = fminsearch(@(u) find_velocity(u,y_r(6),y_r(5),-y_r_dot(3),y_r(2),y_r(4),y_r(1)),y_r(6));
x(5,1) = sin(y_r(5))*y_r(6);
x(6,1) = sqrt(y_r(6)^2-x(4)^2-x(5)^2);
x(7,1) = y_r(2);
x(8,1) = y_r(4);
x(9,1) = y_r(1);
x(10:12,1) = [1,sin(y_r(2))*tan(y_r(4)),-cos(y_r(2))*tan(y_r(4));
            0,cos(y_r(2)),sin(y_r(2));
            0,-sin(y_r(2))/cos(y_r(4)),cos(y_r(2))/cos(y_r(4))]*y_r_dot([2,4,1]);
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

function y_dot = y_as_x(y,u,eqs_motion,get_y_r,sensors,param)
    [~,y_r_dot,y_r_ddot] = get_y_r(sense(y2x(y(1:6),y(7:end)),eqs_motion([],y2x(y(1:6),y(7:end)),u,param),sensors,param),y2x(y(1:6),y(7:end)),zeros(size(param.d_names)),param,eqs_motion([],y2x(y(1:6),y(7:end)),u,param));
    y_dot = [y_r_dot;y_r_ddot];
end

function error = find_velocity(u,V_a,beta,p_d_dot_actual,phi,theta,psi)
    v = sin(beta)*V_a;
    w = sqrt(V_a^2-u^2-v^2);
    V_g = get_rotation(phi,theta,psi,'b->v',[u;v;w]);
    p_d_dot = V_g(3);
    error = abs(p_d_dot_actual-p_d_dot);
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
