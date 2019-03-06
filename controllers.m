classdef controllers < handle
    
    properties
        % General to pass to other functions
        param
        
        % Functions
        get_equilibrium
        
        % States
        t = 0
        r_in_indexes
        r_out_indexes
        u_indexes
        
        % Param
        K
        
        % Settings
        type
        sat_lim
        windup_limit
        r_is_angle
        
        % Controller Specifc
        t_vec
        plan
        prefilter
        compensator
        
        % Defaults
        count = 0;
        error = 0;
        cascade = [];
        intigrator_correction = 0;
        isangle = false
    end
    properties (Constant) % Types of Controllers
        PID = 'PID';
        FSF = 'Full State Feedback';
        LS = 'Loopshapping';
        OL = 'Open Loop';
    end
    
    methods
        % Class Initializer -----------------------------------------------
        function self = controllers(control,core)
            
            % Unapck
            param = core.param;
            functions = core.functions;

            % General to pass to other functions
            self.param = param;
            
            % Functions
            self.get_equilibrium = functions.get_equilibrium;
            
            % States
            self.r_in_indexes = get_indexes(param.r_names,control.r_names);
            self.r_out_indexes = get_indexes(param.r_names,control.u_names);
            self.u_indexes = get_indexes(param.u_names,control.u_names);
            
            % Param
            self.K = control.K;
            
            % Settigs
            self.type = control.type;
            self.sat_lim.high = control.sat_lim.high;
            self.sat_lim.low = control.sat_lim.low;
            self.windup_limit = control.windup_limit;
            self.r_is_angle = param.r_is_angle(self.r_in_indexes);
            
            % Controller Specific
            switch self.type
                case self.OL
                    self.t_vec = control.t_vec;
                    self.plan = control.plan;
                case self.PID
                case self.FSF
                case self.LS
                    self.prefilter = control.prefilter;
                    self.compensator = control.compensator;
            end
        end
         
        % PID -------------------------------------------------------------
        function u = execute_PID(self,u_P,u_I,u_D)
            u = -self.K.D.*u_D + self.K.P.*u_P + self.K.I.*u_I;
        end
        
        % Full State Feadback ---------------------------------------------
        function u = execute_FSF(self,x,r,sum_of_error)
            u = -self.K.K*(x-self.x_e) + self.K.k_r*(r-self.r_e) - self.K.I*sum_of_error;
        end
        
        % Loopshaping -----------------------------------------------------
        function u = execute_LS(self,zc)
            u = self.K.C*zc;
        end
        
        % General ---------------------------------------------------------
        function F = saturate(self,F)
            for i = 1:length(F)
                if F(i) >= self.sat_lim.high
                    F(i) = self.sat_lim.high;
                    warning('SATURATING: High saturation limit reached.')
                elseif F <= self.sat_lim.low
                    F(i) = self.sat_lim.low;
                    warning('SATURATING: Low saturation limit reached.')
                end
            end
        end
        
        function saturation_anti_windup(self,u_unsat,u_sat)
            if self.K.I ~= 0
                self.intigrator_correction = self.intigrator_correction + 1./self.K.I.*(u_unsat - u_sat);
            end
        end
        
        function derivative_anti_windup(self,velocity,dt)
            if abs(velocity) > self.windup_limit
                self.intigrator_correction = self.intigrator_correction + dt*trapz(self.error(end-1:end));
            end
        end
        
        % Main ------------------------------------------------------------
        function [u,r_out] = control(self,y_r_in,y_r_dot_in,r_in,d_hat,t)
            
            % Unpack
            dt = t - self.t;self.t = t;
            r = r_in(self.r_in_indexes);
            y_r = y_r_in(self.r_in_indexes);
            y_r_dot = y_r_dot_in(self.r_in_indexes);
            self.error = [self.error,self.get_error(y_r,r,self.r_is_angle)];
            
            % Anti-Windup
            self.derivative_anti_windup(y_r_dot,dt);
            
            sum_of_error = dt*trapz(self.error) - self.intigrator_correction;
               
            % Execute Controller
            switch self.type
                case self.OL
                    u = self.plan(:,find(self.t_vec>=t,1));
                case self.PID
                    u_P = self.error(end);
                    u_I = sum_of_error;
                    u_D = y_r_dot;
                    u = self.execute_PID(u_P,u_I,u_D);
                case self.FSF
                    u = self.execute_FSF(x,r,sum_of_error);
                case self.LS
                    r_filtered = self.prefilter.filter_next(r,t);
                    u = self.compensator.propigate(r_filtered,t);
            end
            
            % Add Equilibrium
            [u_equilibrium,~,y_r_equilibrium] = self.get_equilibrium(y_r,self.param);
            u_e = [u_equilibrium(self.u_indexes);y_r_equilibrium(self.r_out_indexes)];
            u_unsat = u + u_e - d_hat;

            % Saturation & Anti-Windup
            u_sat = self.saturate(u_unsat);
            self.saturation_anti_windup(u_unsat,u_sat)
            u = u_sat;
            
            % Cascaded controller
            r_out = r_in;
            if ~isempty(self.cascade)
                r_out(self.r_out_indexes) = u;
                [u,r_out] = self.cascade.control(y_r_in,y_r_dot_in,r_out,d_hat,t);
            end
        end
    end
    
    % Static Functions ----------------------------------------------------
    methods (Static)
        function e = get_error(actual,commanded,is_angle)
            if isempty(is_angle),is_angle=false(length(commanded(:,1)));end
            
            e = zeros(size(actual));
            for i = 1:length(is_angle)
                for j = 1:length(actual(1,:))
                    if length(commanded(1,:))==1,k=1;else,k=j;end
                    if is_angle
                        e_y_r = [cos(actual(i,j));sin(actual(i,j));0];
                        e_r = [cos(commanded(i,k));sin(commanded(i,k));0];
                        direction = cross(e_y_r,e_r);
                        e(i,j) = sign(direction(3))*atan2(sqrt(norm(e_y_r)^2*norm(e_r)^2-dot(e_y_r,e_r)^2),dot(e_y_r,e_r));
                    else
                        e(i,j) = commanded(i,k) - actual(i,j);
                    end
                end
            end
        end
        
        % Gains Calculators -----------------------------------------------
        function K = get_PID_gains()
        end
        
        function K = get_FSF_gains()
        end
        
        function K = get_LS_gains()
        end
    end
end

