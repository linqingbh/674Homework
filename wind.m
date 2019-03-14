classdef wind < handle
    properties
        base = [0;0;0]
        gust
    end
    
    properties (Constant)
        steady = struct("sigma",[0,0,0],"L",[1,1,1])
        light_lowalt = struct("sigma",[1.06,1.06,0.7],"L",[200,200,50])
        moderate_lowalt = struct("sigma",[2.12,2.12,1.4],"L",[200,200,50])
        light_highalt = struct("sigma",[1.5,1.5,1.5],"L",[533,533,533])
        moderate_highalt = struct("sigma",[3,3,3],"L",[533,533,533])
    end
    
    methods
        function self = wind(gust_type,base,V_design)
            if ~isempty(base),self.base = base.*[1;1;-1];end
            
            sigma_u = gust_type.sigma(1);
            sigma_v = gust_type.sigma(2);
            sigma_w = gust_type.sigma(3);
            L_u = gust_type.L(1);
            L_v = gust_type.L(2);
            L_w = gust_type.L(3);
            
            tf_u = tf(sigma_u*sqrt(2*V_design/L_u),[1,V_design/L_u]);
            tf_v = tf(sigma_v*sqrt(3*V_design/L_v)*[1,V_design/(sqrt(3)*L_v)],conv([1,V_design/L_v],[1,V_design/L_v]));
            tf_w = tf(sigma_w*sqrt(3*V_design/L_w)*[1,V_design/(sqrt(3)*L_w)],conv([1,V_design/L_w],[1,V_design/L_w]));
            
            self.gust = [system_solver(tf_u);system_solver(tf_v);system_solver(tf_w)];
        end
        
        function [base,gust,gust_dot] = get(self,t)
            base = self.base;
            
            white_noise = randn;
            [gust(1,1),~,~,gust_dot(1,1)] = self.gust(1).propigate(white_noise,t);
            [gust(2,1),~,~,gust_dot(2,1)] = self.gust(2).propigate(white_noise,t);
            [gust(3,1),~,~,gust_dot(3,1)] = self.gust(3).propigate(white_noise,t);
        end
    end
end