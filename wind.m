classdef wind < handle
    properties
        gust_type
        base
        u = 0
        t = 0
    end
    
    properties (Constant)
        steady = struct("sigma",[0,0,0;0,0,0],"L",[1,1,1;1,1,1])
        light_lowalt = struct("sigma",[1.06,1.06,0.7],"L",[200,200,50])
        moderate_lowalt = struct("sigma",[2.12,2.12,1.4],"L",[200,200,50])
        light_highalt = struct("sigma",[1.5,1.5,1.5],"L",[533,533,533])
        moderate_highalt = struct("sigma",[3,3,3],"L",[533,533,533])
    end
    
    methods
        function self = wind(gust_type,base)
            self.gust_type = gust_type;
            self.base = base;
        end
        
        function [V_w_g_dot,V_w_g_ddot] = get(self,h,h_dot,V_a,t)
            if t ~= self.t
                new_u = randn;
                u_dot = (new_u - self.u)/(t-self.t);
            else
                new_u = 0;
                u_dot = 0;
            end
            
            self.t = t;
            self.u = new_u;
            
            sigma = self.gust_type.sigma;
            L = self.gust_type.L;
            
            V_w_g_dot = h_dot;
            V_w_g_ddot(1,1) = sigma(1).*sqrt(2*V_a/L(1))*self.u-V_a/L(1)*h(1);
            V_w_g_ddot(2,1) = sigma(2).*sqrt(3*V_a/L(2))*(u_dot-V_a/(sqrt(3)*L(2))*self.u) - V_a/L(2)*(2*h_dot(2) + V_a/L(2)*h(2));
            V_w_g_ddot(3,1) = sigma(3).*sqrt(3*V_a/L(3))*(u_dot-V_a/(sqrt(3)*L(3))*self.u) - V_a/L(2)*(2*h_dot(3) + V_a/L(2)*h(3));
        end
    end
end