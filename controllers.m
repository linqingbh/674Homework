classdef controllers < handle
    
    properties
        controller_type
        param
        K
        u
        u_e
        add_equilibrium
        e
        dt
        index
        beta
        derivative_source
        anti_windup
        windup_limit
        impose_sat
        sat_lim
        count
        time_step
        x_e
        r_e
        core
        C_m
        C_r
        A
        B
        D
        observer
        x_names
        u_names
        r_names
        output_names
        use_names
        error
        cascade
        intigrator_correction
        y_m_names
        x_indexes
        u_indexes
        y_m_indexes
        r_indexes
        num
        den
        con
        tf
        mags
        phases
        omegas
        zeros
        poles
        zc
        zf
        F
        plan
        history_index
        y_r
        wrapping
    end
    properties (Constant)
        PID = 'PID';
        SS = 'SS';
        LS = 'LS';
        OL = 'OL';
    end
    
    methods
        function self = controllers(control,core)
            
            % Unapck
            self.core = core;
            param = core.param;
            functions = core.functions;
            settings = core.settings;
            
            self.controller_type = control.controller_type;
            
            switch self.controller_type
                case self.OL
                    self.plan = control.plan;
                case self.PID
                    self.K = control.K;
                case self.SS
                case self.LS
                otherwise
                    
                    self.K = control.K;
                    if isfield(control,'F')
                        self.F = control.F;
                    end
                    self.anti_windup = control.anti_windup;
                    self.windup_limit = control.windup_limit;
                    if isfield(control,'cascade')
                        self.cascade = control.cascade;
                    end
                    
                    % States
                    self.r_names = param.x_names(param.C_r*(1:length(self.x_names)).');
                    self.x_e = self.get_compressed_state(self.x_names,self.use_names,param.x_e);
                    self.r_e = self.get_compressed_state(self.x_names,self.r_names,param.x_e);

                    % Param
                    self.y_m_names = param.x_names(param.C_m*(1:length(self.x_names)).'); 
                    self.y_m_indexes = self.get_indexes(self.y_m_names,self.use_names);
                    self.y_r_indexes = self.get_indexes(self.r_names,self.use_names);
                    self.A = param.A(self.x_indexes,self.x_indexes);
                    self.B = param.B(self.x_indexes,self.u_indexes);
                    self.C_m = param.C_m(self.y_m_indexes,self.x_indexes);
                    self.D = param.D(self.y_r_indexes,self.u_indexes);

                    % initiallize variables
                    if strcmp(self.controller_type,self.LS)
                        self.zc = zeros(length(self.K.A),1);
                    end
                    if isfield(control,'F')
                        self.zf = zeros(length(self.F.A),1);
                    end
                    
            end
            
            
            
            % General to pass to other functions
            self.core = core;
            self.param = param;
            
            % Functions
            self.u_e = functions.u_e;
            self.y_r = functions.y_r;
            
            % States
            x = core.subscribe('x');
            self.use_names = control.y_r_names;
            self.output_names = control.u_names;
            self.x_names = param.x_names;
            self.u_names = param.u_names;
            self.r_names = param.r_names;
            
            % Settigs
            self.dt = settings.step;
            self.sat_lim.high = control.sat_lim.high;
            self.sat_lim.low = control.sat_lim.low;
            self.anti_windup = control.anti_windup;
            self.windup_limit = control.windup_limit;
            
            % Param
%             self.x_indexes = self.get_indexes(self.x_names,self.use_names);
%             self.u_indexes = self.get_indexes(self.u_names,self.output_names);

            % initiallize variables
            self.error = 0;
            self.intigrator_correction = 0;
            self.count = 0;
            self.time_step = 0;
            self.history_index = 0;
            
        end
         
        function u = execute_PID(self,u_P,u_I,u_D)
            u = -self.K.D.*u_D + self.K.P.*u_P + self.K.I.*u_I;
        end
        
        function u = execute_SS(self,x,r,sum_of_error)
            u = -self.K.K*(x-self.x_e) + self.K.k_r*(r-self.r_e) - self.K.I*sum_of_error;
        end
        
        function u = execute_LS(self,zc)
            u = self.K.C*zc;
        end
        
        function zc_dot = updatestate(self,zc,e)
            zc_dot = self.K.A*zc+self.K.B*e;
        end
        
        function zf_dot = updateprefilter(self,zf,r)
            zf_dot = self.F.A*zf+self.F.B*r;
        end
        
        function summed_error = sum_error(self)
            summed_error = self.dt*trapz(self.error);
        end
        
        function saturation_anti_windup(self,u_unsat,u_sat)
            self.intigrator_correction = self.intigrator_correction + 1./self.K.I.*(u_unsat - u_sat);
        end
        
        function derivative_anti_windup(self,velocity)
            if abs(velocity) > self.windup_limit
                self.intigrator_correction = self.intigrator_correction + self.dt*trapz(self.error(end-1:end));
            end
        end
        
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
        
        function x_out = get_compressed_state(self,options,choices,x_in)
            x_out = zeros(size(choices));
            for i = 1:length(choices)
                if any(strcmp(options,choices(i)))
                    x_out(i) = x_in(strcmp(options,choices(i)));
                end
            end
        end
        
        function indexes = get_indexes(self,options,names)
            indexes = false(size(options));
            for i = 1:length(names)
                indexes = indexes | strcmp(options,names(i));
            end
        end
        
        function u = control(self,full_x,r,d_hat)
            
            % Count 
            self.history_index = self.history_index + 1;
            
%             if self.history_index >= 161
%                 throw = 1;
%             end
            
%             if strcmp(self.output_names,"delta_a") && self.history_index>=159
%                 throw = 1;
%             end
            
            % Unpack
%             x = self.get_compressed_state(self.x_names,self.use_names,full_x);
            [y_r_full,y_r_dot_full] = self.y_r(full_x,self.core);
            y_r = self.get_compressed_state(self.r_names,self.use_names,y_r_full);
            y_r_dot = self.get_compressed_state(self.r_names,self.use_names,y_r_dot_full);
            if isempty(self.wrapping)
                self.error = [self.error,r - y_r];
            else
                if self.wrapping
                    e_y_r = [cos(y_r);sin(y_r);0];
                    e_r = [cos(r);sin(r);0];
                    direction = cross(e_y_r,e_r);
                    self.error = [self.error,sign(direction(3))*atan2(sqrt(norm(e_y_r)^2*norm(e_r)^2-dot(e_y_r,e_r)^2),dot(e_y_r,e_r))];
                else
                    self.error = [self.error,r - y_r];
                end
            end
            
            % Anti-Windup
            if (strcmp(self.anti_windup,'derivative') || strcmp(self.anti_windup,'both'))
                    self.derivative_anti_windup(y_r_dot);
            end
               
            % Execute Controller
            switch self.controller_type
                case self.OL
                    self.time_step = self.time_step+1;
                    u = self.plan(:,self.time_step);
                case self.PID
                    u_P = self.error(end);
                    u_I = self.sum_error() - self.intigrator_correction;
                    u_D = y_r_dot;
                    u = self.execute_PID(u_P,u_I,u_D);
                case self.SS
                    sum_of_error = self.sum_error() - self.intigrator_correction;
                    u = self.execute_SS(x,r,sum_of_error);
                case self.LS
                    
                    if ~isempty(self.F)
%                         k1 = self.updateprefilter(self.zf,r_tilda);
%                         k2 = self.updateprefilter(self.zf + self.dt/2*k1,r_tilda);
%                         k3 = self.updateprefilter(self.zf + self.dt/2*k2,r_tilda);
%                         k4 = self.updateprefilter(self.zf + self.dt*k3,r_tilda);
%                         zf = self.zf + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

                        for i = 1:10
                            self.zf = self.zf + self.dt/10*(self.updateprefilter(self.zf,(r-self.x_e(self.y_r_indexes))));
                        end
                        e = self.F.C*self.zf - (y_r-self.x_e(self.y_r_indexes));
                    else
                        e = self.error(end);
                    end
                    
                    
%                     k1 = self.updatestate(self.zc,e);
%                     k2 = self.updatestate(self.zc + self.dt/2*k1,e);
%                     k3 = self.updatestate(self.zc + self.dt/2*k2,e);
%                     k4 = self.updatestate(self.zc + self.dt*k3,e);
%                     self.zc = self.zc + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
                    
                    
                    for i = 1:10
                        self.zc = self.zc + self.dt/10*(self.updatestate(self.zc,e));
                    end
                    u = self.execute_LS(self.zc);
            end
            
            if strcmp(self.controller_type,self.SS)
                x_e = self.x_e;
            else
                x_e = full_x;
            end
            
            % Add Equilibrium
            [u_e_full,x_e_full] = self.u_e(x_e,self.param);
            u_e = self.get_compressed_state([self.u_names;self.x_names],self.output_names,[u_e_full;x_e_full]);
            if isempty(u_e)
                u_e = 0;
            end
            u_unsat = u + u_e - d_hat;

            % Anti-Windup
            u_sat = self.saturate(u_unsat);
            if ((strcmp(self.anti_windup,'saturation') || strcmp(self.anti_windup,'both'))) && self.K.I
                self.saturation_anti_windup(u_unsat,u_sat)
            end
            u = u_sat;
            
            % Cascaded controller
            if ~isempty(self.cascade)
                indexes = self.get_indexes(self.r_names,self.output_names);
                r = self.core.subscribe_specific("r",self.history_index);
                r(indexes) = u;
                self.core.publish_specific("r",r,self.history_index)
                u = self.cascade.control(full_x,u,d_hat);
            end
        end
        
    end
end

