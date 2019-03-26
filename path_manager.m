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
        
        planner
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
            p = core.param.x_0(self.x_indexes);
            self.last_limit = p;
            
            self.planner = manage.planner;

%             if any(self.W(:,1) ~= p)
%                 self.W = [p,self.W];
%             end
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
                leg.line = [self.last_limit,self.W(1:3,self.i)];
                
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
                leg.line = [self.last_limit(1:3),z];
                
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
                limits(:,1) = sign(self.last_limit(2)-leg.b(2))*acos(dot(self.last_limit-leg.b,[1;0;0])/(norm(self.last_limit-leg.b)));
                limits(:,2) = sign(z(2)-leg.b(2))*acos(dot(z-leg.b,[1;0;0])/(norm(z-leg.b)));
                leg.line = draw_circle(leg.b,self.R,leg.q(3),limits(:,1),limits(:,2));
                
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
            try
                [line,c_s,q_s,c_e,q_e,z1,q1,z2,z3,q3] = path_manager.calculate_dubins(self.W(1:3,self.i-1),self.W(4,self.i-1),self.R,self.W(1:3,self.i),self.W(4,self.i));
            catch 
                throw = 1;
            end
            switch self.state
                case 1
                    leg.b = c_s;
                    leg.rho = self.R;
                    leg.q = q_s;
                    leg.line = line;
                    line_unit_vec = (z2-z1)/norm(z2-z1);
                    if path_manager.past_half_plane(p,z1,-q1)
                        self.state = 2;
                    elseif all(approx(q1(1:2),[cos(self.W(4,self.i-1));sin(self.W(4,self.i-1))]),2) % check altitude issues to fix this?
                        self.state = 3;
                    end
                case 2
                    leg.b = c_s;
                    leg.rho = self.R;
                    leg.q = q_s;
                    leg.line = line;
                    if path_manager.past_half_plane(p,z1,q1)
                        self.state = 3;
                    end
                case 3
                    leg.b = z1;
                    leg.rho = Inf;
                    leg.q = q1;
                    leg.line = line;
                    if path_manager.past_half_plane(p,z2,q1)
                        self.state = 4;
                    end
                case 4
                    leg.b = c_e;
                    leg.rho = self.R;
                    leg.q = q_e;
                    leg.line = line;
                    if path_manager.past_half_plane(p,z3,-q3)
                        self.state = 5;
                    elseif all(approx(q3(1:2),q1(1:2)),2)
                        self.state = 5;
                    end
                case 5
                    leg.b = c_e;
                    leg.rho = self.R;
                    leg.q = q_e;
                    leg.line = line;
                    if path_manager.past_half_plane(p,z3,q3)
                        if self.i >= length(self.W(1,:))
                            self.state = 1;
                            self.planner.update = true;
                            self.i = 2;
                        else
                            self.state = 1;
                            self.i = self.i +1;
                        end
                    end
                case 6
                    leg = self.core.param.default;
            end
        end

        function leg = manage(self,W,x)
            
            p = x(self.x_indexes);
            self.W = W;
            
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
        
        function [line,c_s,q_s,c_e,q_e,z1,q1,z2,z3,q3,L] = calculate_dubins(p_s,chi_s,R,p_e,chi_e)
            if ~exist('chi_e','var') || isempty(chi_e)
                c_rs = p_s + R*[cos(chi_s+pi/2);sin(chi_s+pi/2);0];
                c_ls = p_s + R*[cos(chi_s-pi/2);sin(chi_s-pi/2);0];
                
                r2end = p_e - c_rs;
                l2end = p_e - c_ls;
                
                nu_r = angle_between([1;0;0],r2end);
                nu_l = angle_between([1;0;0],l2end);
                
                nu_2_r = real(asin(R/norm(r2end))); % should be asin or acos
                nu_2_l = real(asin(-R/norm(l2end)));
                
                chi_e_r = wrap(nu_r+nu_2_r);
                chi_e_l = wrap(nu_l+nu_2_l);
            else
                chi_e_r = chi_e;
                chi_e_l = chi_e;
            end
            
%             figure(1)
%             hold on
%             grid on
%             plot(p_s(2),p_s(1),'.k','MarkerSize',10)
%             plot(p_e(2),p_e(1),'.k','MarkerSize',10)
            
            c_rs = p_s + R*[cos(chi_s+pi/2);sin(chi_s+pi/2);0];
            c_ls = p_s + R*[cos(chi_s-pi/2);sin(chi_s-pi/2);0];
            c_rre = p_e + R*[cos(chi_e_r+pi/2);sin(chi_e_r+pi/2);0];
            c_lre = p_e + R*[cos(chi_e_l+pi/2);sin(chi_e_l+pi/2);0];
            c_rle = p_e + R*[cos(chi_e_r-pi/2);sin(chi_e_r-pi/2);0];
            c_lle = p_e + R*[cos(chi_e_l-pi/2);sin(chi_e_l-pi/2);0];
%             plot(c_rs(2),c_rs(1),'.k','MarkerSize',10)
%             plot(c_ls(2),c_ls(1),'.k','MarkerSize',10)
%             plot(c_rre(2),c_rre(1),'.k','MarkerSize',10)
%             plot(c_lre(2),c_lre(1),'.k','MarkerSize',10)
%             plot(c_rle(2),c_rle(1),'.k','MarkerSize',10)
%             plot(c_lle(2),c_lle(1),'.k','MarkerSize',10)
            
%             nu_rsr = angle_between([1;0;0],[c_rre(1)-c_rs(1);c_rre(2)-c_rs(2);0]);
%             nu_rsl = angle_between([1;0;0],[c_rle(1)-c_rs(1);c_rle(2)-c_rs(2);0]);
%             nu_lsr = angle_between([1;0;0],[c_lre(1)-c_ls(1);c_lre(2)-c_ls(2);0]);
%             nu_lsl = angle_between([1;0;0],[c_lle(1)-c_ls(1);c_lle(2)-c_ls(2);0]);
            if ~exist('chi_e','var') || isempty(chi_e)
                nu_rsr = chi_e_r;
                nu_rsl = chi_e_r;
                nu_lsr = chi_e_l;
                nu_lsl = chi_e_l;
            else
                nu_rsr = angle_between([1;0;0],[c_rre(1)-c_rs(1);c_rre(2)-c_rs(2);0]);
                nu_rsl = angle_between([1;0;0],[c_rle(1)-c_rs(1);c_rle(2)-c_rs(2);0]);
                nu_lsr = angle_between([1;0;0],[c_lre(1)-c_ls(1);c_lre(2)-c_ls(2);0]);
                nu_lsl = angle_between([1;0;0],[c_lle(1)-c_ls(1);c_lle(2)-c_ls(2);0]);
            end
            
            l_rsr = norm(c_rs-c_rre);
            l_rsl = norm(c_rs-c_rle);
            l_lsr = norm(c_ls-c_lre);
            l_lsl = norm(c_ls-c_lle);
            
%             nu_2_rsl = nu_rsl - pi/2 + asin(2*R/l_rsl);
%             nu_2_lsr = acos(2*R/l_lsr);
            if ~exist('chi_e','var') || isempty(chi_e)
                nu_2_rsl = nu_rsl - pi/2;
                nu_2_lsr = pi/2;
            else
                nu_2_rsl = nu_rsl - pi/2 + asin(2*R/l_rsl);
                nu_2_lsr = acos(2*R/l_lsr);
            end
            
            L_rsr = l_rsr + R*angle_between(chi_s-pi/2,nu_rsr-pi/2,'cw') + R*angle_between(nu_rsr-pi/2,chi_e_r-pi/2,'cw');
            L_rsl = sqrt(l_rsl^2-4*R^2) + R*angle_between(chi_s-pi/2,nu_2_rsl,'cw') + R*angle_between(nu_2_rsl+pi,chi_e_r+pi/2,'ccw');% this is to high
            L_lsr = sqrt(l_lsr^2-4*R^2) + R*angle_between(chi_s+pi/2,nu_lsr+nu_2_lsr,'ccw') + R*angle_between(nu_lsr+nu_2_lsr-pi,chi_e_l-pi/2,'cw');
            L_lsl = l_lsl + R*angle_between(chi_s+pi/2,nu_lsl+pi/2,'ccw') + R*angle_between(nu_lsl+pi/2,chi_e_l+pi/2,'ccw');
            
            L = min([L_rsr,L_rsl,L_lsr,L_lsl]);
            e1 = [1;0;0];
            switch L
                case L_rsr
                    c_s = c_rs;
                    q_s = [0;0;1];
                    c_e = c_rre;
                    q_e = [0;0;1];
                    q1 = (c_e-c_s)/norm(c_e-c_s);
                    z1 = c_s + R*path_manager.R_z(-pi/2)*q1;
                    z2 = c_e + R*path_manager.R_z(-pi/2)*q1;
                    limits{1} = wrap(chi_s - pi/2,pi);
                    limits{2} = wrap(nu_rsr- pi/2,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_rsr- pi/2,pi);
                    limits{6} = wrap(chi_e_r - pi/2,pi);
                    q3 = path_manager.R_z(chi_e_r)*e1;
                    chi_e = chi_e_r;
                case L_rsl
                    c_s = c_rs;
                    q_s = [0;0;1];
                    c_e = c_rle;
                    q_e = [0;0;-1];
                    q1 = path_manager.R_z(nu_2_rsl+pi/2)*e1;
                    z1 = c_s + R*path_manager.R_z(nu_2_rsl)*e1;
                    z2 = c_e + R*path_manager.R_z(nu_2_rsl+pi)*e1;
                    limits{1} = wrap(chi_s - pi/2,pi);
                    limits{2} = wrap(nu_2_rsl,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_2_rsl + pi,pi);
                    limits{6} = wrap(chi_e_r + pi/2,pi);
                    q3 = path_manager.R_z(chi_e_r)*e1;
                    chi_e = chi_e_r;
                case L_lsr
                    c_s = c_ls;
                    q_s = [0;0;-1];
                    c_e = c_lre;
                    q_e = [0;0;1];
                    q1 = path_manager.R_z(nu_lsr+nu_2_lsr-pi/2)*e1;
                    z1 = c_s + R*path_manager.R_z(nu_lsr+nu_2_lsr)*e1;
                    z2 = c_e + R*path_manager.R_z(nu_lsr+nu_2_lsr-pi)*e1;
                    limits{1} = wrap(chi_s + pi/2,pi);
                    limits{2} = wrap(nu_lsr + nu_2_lsr,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_lsr + nu_2_lsr - pi,pi);
                    limits{6} = wrap(chi_e_l - pi/2,pi);
                    q3 = path_manager.R_z(chi_e_l)*e1;
                    chi_e = chi_e_l;
                case L_lsl
                    c_s = c_ls;
                    q_s = [0;0;-1];
                    c_e = c_lle;
                    q_e = [0;0;-1];
                    q1 = (c_e-c_s)/norm(c_e-c_s);
                    z1 = c_s + R*path_manager.R_z(pi/2)*q1;
                    z2 = c_e + R*path_manager.R_z(pi/2)*q1;
                    limits{1} = wrap(chi_s + pi/2,pi);
                    limits{2} = wrap(nu_lsl + pi/2,pi);
                    limits{3} = z1;
                    limits{4} = z2;
                    limits{5} = wrap(nu_lsl + pi/2,pi);
                    limits{6} = wrap(chi_e_l + pi/2,pi);
                    q3 = path_manager.R_z(chi_e_l)*e1;
                    chi_e = chi_e_l;
            end
            if all(approx([cos(chi_s);sin(chi_s)],q1(1:2),2))
                T1 = [];
            else
                T1 = draw_circle(c_s,R,q_s(3),limits{1},limits{2});
            end
            
            S = [limits{3},limits{4}];
            
            if all(approx([cos(chi_e);sin(chi_e)],q1(1:2),2))
                T2 = [];
            else
                T2 = draw_circle(c_e,R,q_e(3),limits{5},limits{6});
            end
            
            line = [T1,S,T2];
            
            z3 = p_e;
        end
        
        function R = R_z(angle)
           R =  [cos(angle),-sin(angle),0;
                 sin(angle),cos(angle),0;
                 0,0,1];
        end
    end
end