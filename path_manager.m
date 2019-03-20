classdef path_manager < handle
    properties
        % Settings
        core
        type
        
        % Parameters
        R
        
        % Indexes
        i = 2;
        state = 1;
        x_indexes
        
        % Waypoints
        W
        last_limit
    end
    
    properties (Constant)
        through = "Fly through waypoints"
        fillets = "Fly filleted transition between legs"
        dubins = "Fly to specific course, altitude, and position"
    end
    
    methods
        function self = path_manager(manage,core)
            % Settings
            self.core = core;
            self.type = manage.type;
            
            % Indexes
            self.x_indexes = get_indexes(core.param.x_names,manage.x_names);

            % Parameters
            self.R = core.param.fillet_radius;
            
            % Waypoints
            self.W = core.subscribe_history('W');
            p = core.param.x_0(self.x_indexes);
            self.last_limit = p;

            if any(self.W(:,1) ~= p)
                self.W = [p,self.W];
            end
        end
        
        function leg = execute_through(self,p)
            if self.i > length(self.W(1,:))
                leg = self.core.param.default;
            else
                q(:,1) = (self.W(1:3,self.i)-self.W(1:3,self.i-1))/norm(self.W(1:3,self.i)-self.W(1:3,self.i-1));
                if self.i < length(self.W(1,:))
                    q(:,2) = (self.W(1:3,self.i+1)-self.W(1:3,self.i))/norm(self.W(1:3,self.i+1)-self.W(1:3,self.i));
                    n = (q(:,1) - q(:,2))/norm(q(:,1) - q(:,2));
                else
                    n = q(:,1);
                end
                
                leg.b = self.W(1:3,self.i-1);
                leg.q = q(:,1);
                leg.rho = Inf;
                leg.limits = [self.last_limit,self.W(1:3,self.i)];
                
                if path_manager.past_half_plane(p,self.W(1:3,self.i),n)
                    self.last_limit = self.W(1:3,self.i);
                    self.i = self.i+1;
                end
            end
        end
        
        function leg = execute_fillets(self,p)
            q(:,1) = (self.W(1:3,self.i)-self.W(1:3,self.i-1))/norm(self.W(1:3,self.i)-self.W(1:3,self.i-1));
            q(:,2) = (self.W(1:3,self.i+1)-self.W(1:3,self.i))/norm(self.W(1:3,self.i+1)-self.W(1:3,self.i));
            
            varrho = acos(dot(-q(:,1),q(:,2)));
            
            if self.state == 1
                z = self.W(1:3,self.i) - (self.R/tan(varrho/2))*q(:,1);
                
                leg.b = self.W(1:3,self.i-1);
                leg.q = q(:,1);
                leg.rho = Inf;
                leg.limits = [self.last_limit(1:3),z];
                
                if path_manager.past_half_plane(p,z,q(:,1))
                    self.last_limit = z;
                    self.state = 2;
                end
            elseif self.state == 2
                z = self.W(1:3,self.i) + (self.R/tan(varrho/2))*q(:,2);
                
                leg.b = self.W(1:3,self.i) - (self.R/sin(varrho/2))*(q(:,1) - q(:,2))/norm(q(:,1) - q(:,2));
                leg.b(3) = self.last_limit(3);
                leg.q = [0;0;sign(q(1,1)*q(2,2)-q(1,2)*q(2,1))];
                leg.rho = self.R;
                leg.limits(1) = sign(self.last_limit(2)-leg.b(2))*acos(dot(self.last_limit-leg.b,[1;0;0])/(norm(self.last_limit-leg.b)));
                leg.limits(2) = sign(z(2)-leg.b(2))*acos(dot(z-leg.b,[1;0;0])/(norm(z-leg.b)));
                                
                if path_manager.past_half_plane(p,z,q(:,2))
                    self.last_limit = z;
                    if self.i == length(self.W(1,:))-1
                        self.i = self.i + 1;
                        self.type = path_manager.through;
                    else
                        self.i = self.i + 1;
                        self.state = 1;
                    end
                end
            end
        end
        
        function leg = execute_dubins(self,p)
            [limits,c_s,q_s,c_e,q_e,z1,q1,z2,z3,q3] = calculate_dubins(self,self.W(1:3,self.i-1),self.W(4,self.i-1),self.W(1:3,self.i),self.W(4,self.i));
            switch self.state
                case 1
                    leg.b = c_s;
                    leg.rho = self.R;
                    leg.q = q_s;
                    leg.limits(:,1) = limits{1};
                    leg.limits(:,2) = limits{2};
                    if path_manager.past_half_plane(p,z1,-q1)
                        self.state = 2;
                    end
                case 2
                    leg.b = c_s;
                    leg.rho = self.R;
                    leg.q = q_s;
                    leg.limits(:,1) = limits{1};
                    leg.limits(:,2) = limits{2};
                    if path_manager.past_half_plane(p,z1,q1)
                        self.state = 3;
                    end
                case 3
                    leg.b = z1;
                    leg.rho = Inf;
                    leg.q = q1;
                    leg.limits(:,1) = limits{3};
                    leg.limits(:,2) = limits{4};
                    if path_manager.past_half_plane(p,z2,q1)
                        self.state = 4;
                    end
                case 4
                    leg.b = c_e;
                    leg.rho = self.R;
                    leg.q = q_e;
                    leg.limits(:,1) = limits{5};
                    leg.limits(:,2) = limits{6};
                    if path_manager.past_half_plane(p,z3,-q3)
                        self.state = 5;
                    end
                case 5
                    leg.b = c_e;
                    leg.rho = self.R;
                    leg.q = q_e;
                    leg.limits(:,1) = limits{5};
                    leg.limits(:,2) = limits{6};
                    if path_manager.past_half_plane(p,z3,q3)
                        if self.i >= length(self.W(1,:))
                            self.state = 6;
                        else
                            self.state = 1;
                            self.i = self.i +1;
                        end
                    end
                case 6
                    leg = self.core.param.default;
            end
        end
        
        function [limits,c_s,q_s,c_e,q_e,z1,q1,z2,z3,q3] = calculate_dubins(self,p_s,chi_s,p_e,chi_e)
            c_rs = p_s + self.R*[cos(chi_s+pi/2);sin(chi_s+pi/2);0];
            c_ls = p_s + self.R*[cos(chi_s-pi/2);sin(chi_s-pi/2);0];
            c_re = p_e + self.R*[cos(chi_e+pi/2);sin(chi_e+pi/2);0];
            c_le = p_e + self.R*[cos(chi_e-pi/2);sin(chi_e-pi/2);0];
            
            nu_rsr = angle_between([1;0;0],[c_re(1)-c_rs(1);c_re(2)-c_rs(2);0]);
            nu_rsl = angle_between([1;0;0],[c_le(1)-c_rs(1);c_le(2)-c_rs(2);0]);
            nu_lsr = angle_between([1;0;0],[c_re(1)-c_ls(1);c_re(2)-c_ls(2);0]);
            nu_lsl = angle_between([1;0;0],[c_le(1)-c_ls(1);c_le(2)-c_ls(2);0]);
            
            l_rsr = norm(c_rs-c_re);
            l_rsl = norm(c_rs-c_le);
            l_lsr = norm(c_ls-c_re);
            l_lsl = norm(c_ls-c_le);
            
            nu_2_rsl = nu_rsl - pi/2 + asin(2*self.R/l_rsl);
            nu_2_lsr = acos(2*self.R/l_lsr);
            
            L_rsr = l_rsr + self.R*angle_between(chi_s-pi/2,nu_rsr-pi/2,'cw') + self.R*angle_between(nu_rsr-pi/2,chi_e-pi/2,'cw');
            L_rsl = sqrt(l_rsl^2-4*self.R^2) + self.R*angle_between(chi_s-pi/2,nu_2_rsl,'cw') + self.R*angle_between(nu_2_rsl+pi,chi_e+pi/2,'ccw');
            L_lsr = sqrt(l_lsr^2-4*self.R^2) + self.R*angle_between(chi_s+pi/2,nu_lsr+nu_2_lsr,'ccw') + self.R*angle_between(nu_lsr+nu_2_lsr-pi,chi_e-pi/2,'cw');
            L_lsl = l_lsl + self.R*angle_between(chi_s+pi/2,nu_lsl+pi/2,'ccw') + self.R*angle_between(nu_lsl+pi/2,chi_e+pi/2,'ccw');
            
            L = min([L_rsr,L_rsl,L_lsr,L_lsl]);
            e1 = [1;0;0];
            switch L
                case L_rsr
                    c_s = c_rs;
                    q_s = [0;0;1];
                    c_e = c_re;
                    q_e = [0;0;1];
                    q1 = (c_e-c_s)/norm(c_e-c_s);
                    z1 = c_s + self.R*self.R_z(-pi/2)*q1;
                    z2 = c_e + self.R*self.R_z(-pi/2)*q1;
                    limits{1} = wrap(chi_s - pi/2,pi);
                    limits{2} = wrap(nu_rsr- pi/2,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_rsr- pi/2,pi);
                    limits{6} = wrap(chi_e - pi/2,pi);
                case L_rsl
                    c_s = c_rs;
                    q_s = [0;0;1];
                    c_e = c_le;
                    q_e = [0;0;-1];
                    q1 = self.R_z(nu_2_rsl+pi/2)*e1;
                    z1 = c_s + self.R*self.R_z(nu_2_rsl)*e1;
                    z2 = c_e + self.R*self.R_z(nu_2_rsl+pi)*e1;
                    limits{1} = wrap(chi_s - pi/2,pi);
                    limits{2} = wrap(nu_2_rsl,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_2_rsl + pi,pi);
                    limits{6} = wrap(chi_e + pi/2,pi);
                case L_lsr
                    c_s = c_ls;
                    q_s = [0;0;-1];
                    c_e = c_re;
                    q_e = [0;0;1];
                    q1 = self.R_z(nu_lsr+nu_2_lsr-pi/2)*e1;
                    z1 = c_s + self.R*self.R_z(nu_lsr+nu_2_lsr)*e1;
                    z2 = c_e + self.R*self.R_z(nu_lsr+nu_2_lsr-pi)*e1;
                    limits{1} = wrap(chi_s + pi/2,pi);
                    limits{2} = wrap(nu_lsr + nu_2_lsr,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_lsr + nu_2_lsr - pi,pi);
                    limits{6} = wrap(chi_e - pi/2,pi);
                case L_lsl
                    c_s = c_ls;
                    q_s = [0;0;-1];
                    c_e = c_le;
                    q_e = [0;0;-1];
                    q1 = (c_e-c_s)/norm(c_e-c_s);
                    z1 = c_s + self.R*self.R_z(pi/2)*q1;
                    z2 = c_e + self.R*self.R_z(pi/2)*q1;
                    limits{1} = wrap(chi_s + pi/2,pi);
                    limits{2} = wrap(nu_lsl + pi/2,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_lsl + pi/2,pi);
                    limits{6} = wrap(chi_e + pi/2,pi);
            end
            z3 = p_e;
            q3 = self.R_z(chi_e)*e1;
        end
        
        function R = R_z(~,angle)
           R =  [cos(angle),-sin(angle),0;
                 sin(angle),cos(angle),0;
                 0,0,1];
        end

        function leg = manage(self,x)
            
            p = x(self.x_indexes);
            
            switch self.type
                case self.through
                    leg = self.execute_through(p(1:3));
                case self.fillets
                    leg = self.execute_fillets(p(1:3));
                case self.dubins
                    leg = self.execute_dubins(p(1:3));
            end
            
            leg.V_a = self.core.param.trim.V_a;
        end
    end
    
    methods (Static)
        function boolean = past_half_plane(p,r,n)
            boolean = dot((p-r),n) >= 0;
        end
    end
end