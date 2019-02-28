classdef dirty_direvative < handle
    properties
        sigma
        x
        x_dot
        t
    end
    methods
        function self = dirty_direvative(sigma,x,x_dot,t)
            self.sigma = sigma;
            self.x = x;
            self.x_dot = x_dot;
            self.t = t;
        end
        function x_dot = estimate(self,x,t)
            dt = t - self.t;
            
            beta = (2.*self.sigma - dt)./(2.*self.sigma + dt);
            
            x_dot = beta.*self.x_dot + (1-beta)./dt.*(x-self.x);
            
            self.x = x;
            self.x_dot = x_dot;
            self.t = t;
        end
    end
end