classdef path_follower < handle
    properties
        % Parameters
        core
        
        % Indexes
        r_indexes
        x_indexes
        
        % Gains
        k
        chi_inf
    end
    
    methods
        function self = path_follower(follow,core)
            % Parameters
            self.core = core;
            
            % Indexes
            self.r_indexes = get_indexes(core.param.r_names,follow.r_names);
            self.x_indexes = get_indexes(core.param.x_names,follow.x_names);
            
            % Gains
            self.k = follow.k;
            self.chi_inf = follow.chi_inf;
        end
        
        function r_chi = execute_line(self,leg,p)
            b = leg.b;
            q = leg.q;
            chi_q = atan2(q(2),q(1));
            e_py = -sin(chi_q)*(p(1)-b(1))+cos(chi_q)*(p(2)-b(2));
            chi_d = self.chi_inf*2/pi*atan(self.k.line*e_py);
            r_chi = controllers.get_error(chi_d,chi_q,true);
        end
        
        function [r_chi,r_phi] = execute_orbit(self,leg,p)
            c = leg.b;
            rho = leg.rho;
            lambda = leg.q(3);
            d = sqrt((p(1)-c(1))^2+(p(2)-c(2))^2);
            phi = atan2(p(2)-c(2),p(1)-c(1));
            chi_0 = phi + lambda*pi/2;
            chi_d = -lambda*atan2(self.k.orbit*(d-rho),rho);
            r_chi = controllers.get_error(chi_d,chi_0,true);
            r_phi = lambda*atan(leg.V_a^2/(9.81*d));
        end
        
        function r_h = execute_altitude(~,leg,p)
            b = leg.b;
            q = leg.q;
            e_p = (p-b);
            k_hat = [0;0;1];
            n = cross(q,k_hat)/norm(cross(q,k_hat));
            s = e_p - dot(e_p,n)*n;
            r_h = -b(3) - sqrt(s(1)^2+s(2)^2)*(q(3)/sqrt(q(1)^2+q(2)^2));
        end
        
        function r = follow(self,leg,x)
            
            p = x(self.x_indexes);
            
            switch leg.rho
                case Inf
                    r(1) = self.execute_line(leg,p);
                    r(2) = self.execute_altitude(leg,p);
                    r(3) = leg.V_a; % Make more leg freindly
                    self.core.param.y_r_0(2) = 0;
                otherwise
                    [r(1),r_phi] = self.execute_orbit(leg,p);
                    r(2) = -leg.b(3);
                    r(3) = leg.V_a;
                    self.core.param.y_r_0(2) = r_phi;
                    
            end
                
                
        end
    end
end