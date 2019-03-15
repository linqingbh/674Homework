classdef observers < handle
    
    properties
        % General to pass to other functions
        param

        % Param
        L
        
        % States
        x_e
        u_e
        x_hat
        x_indexes
        z_indexes
        m_indexes
        r_indexes
        u_indexes
        d_indexes
        position
        velocity

        % Settings
        type
        m_is_angle
        x_is_angle
        
        % Defaults
        t = 0;
    end
    
    properties (Constant)
        pass = 'pass measured values'
        exact = 'Exact'
        luenberger = 'Luenberger Observer'
        de = 'Dirivative based on error'
        dy = 'Dirivative based on state'
        ekf = 'Extended Kalman Filter'
    end
    
    methods
        function self = observers(observe,core)
            
            % Unpack
            param = core.param;
            functions = core.functions;
            
            % General to pass to other functions
            self.param = param;
            
            % Functions of Import
            observe.L.f = @(x,u) [functions.eqs_motion(0,x(1:length(observe.x_names)),u,param);0;0;0];
            observe.L.h = @(x,u) functions.get_y_m(set_indexes(functions.sensors,'z_indexes','sense',{x,functions.eqs_motion(0,x,u,param),[],0}),param);
%             observe.L.h = @(x,u) set_indexes(functions.sensors,'z_indexes','sense',{x,functions.eqs_motion(0,x,u,param),[],0});
            observe.L.A = @(x,u) numerical_jacobian(@(state) self.L.f(state,u),x);
            observe.L.B = @(x,u) numerical_jacobian(@(input) self.L.f(x,input),u);
            observe.L.C = @(x,u,indexes) numerical_jacobian(@(state) self.L.h(state,u),x,0.01,indexes);
            observe.L.D = @(x,u) numerical_jacobian(@(input) self.L.h(x,input),u);
            observe.L.P = zeros(length(observe.x_names)+length(observe.d_names));
            observe.L.R = sensors.callibrate_sensors(core,param.x_0);
            
            % Param
            self.L = observe.L;
            
            % States
            self.x_indexes = get_indexes(param.x_names,observe.x_names);
            self.m_indexes = get_indexes(param.m_names,observe.m_names);
            self.z_indexes = get_indexes(param.z_names,observe.z_names);
            self.r_indexes = get_indexes(param.r_names,observe.r_names);
            self.u_indexes = get_indexes(param.u_names,observe.u_names);
            self.d_indexes = get_indexes(param.d_names,observe.d_names);
            x_0 = core.subscribe('x_hat');
            d_0 = core.subscribe('d_hat');
            y_m = core.subscribe('y_m');
            z = core.subscribe('z');
            [self.x_e,self.u_e] = functions.get_equilibrium(x_0,param);
            self.x_hat = [x_0(self.x_indexes);d_0(self.d_indexes)];
            self.position = y_m(self.m_indexes);
            self.velocity = zeros(size(self.position));
            
            % Settings
            self.type = observe.type;
            self.m_is_angle = param.m_is_angle(self.m_indexes);  
            self.x_is_angle = param.x_is_angle(self.x_indexes); 

            % Observer Specific
            switch self.type
                case self.exact
                case self.luenberger
                case self.de
                case self.dy
                case self.ekf
%                     self.position = z(self.z_indexes);
            end
        end
        
        % Luenberger Observer ---------------------------------------------
        function x_hat_dot = luenberger_eqs(self,~,x_hat,y_m,u,d)
            x_hat_dot = self.L.A*(x_hat - self.x_e) + self.L.B*(u - self.u_e + d) + self.L.L*(y_m - self.L.C_m*x_hat);
        end
        
        % Dirty Diriviative -----------------------------------------------
        function velocity = dirty_direvative(self,position,beta,dt)
            velocity = beta.*self.velocity + (1-beta)./dt.*(position-self.position);
        end
        
        % Extended Kalman Filter ------------------------------------------
        function P_dot = covariance(self,P,A)
            P_dot = A*P + P*A.' + self.L.Q;
        end
        
        % Main ------------------------------------------------------------
        function [x_hat,d_hat] = observe(self,x,z,y_m,r,u,d,t)
            % Unpack
            dt  = t - self.t;
            x   = [x(self.x_indexes);d(self.d_indexes)];
            z   = z(self.z_indexes);
            y_m = y_m(self.m_indexes);
            r   = r(self.r_indexes);
            u   = u(self.u_indexes);
            
            %tic
            % Obtain measurment
            switch self.type
                case self.pass
                    x_hat = y_m;
                case self.exact
                    x_hat = x;
                case self.luenberger
                    x_hat = rk4(@(time,state) self.luenberger_eqs(time,state,y_m,u,d),[self.t,t],self.x_hat);
                case self.de
                    error = control.get_error(y_m,r,self.m_is_angle);
                    beta = (2.*self.L.sigma - dt)./(2.*self.L.sigma + dt);
                    self.velocity = dirty_direvative(self,error,beta,dt);
                    self.position = y_m;
                    x_hat = -self.velocity;
                case self.dy
                    beta = (2.*self.L.sigma - dt)./(2.*self.L.sigma + dt);
                    self.velocity = dirty_direvative(self,y_m,beta,dt);
                    self.position = y_m;
                    x_hat = self.velocity;
                case self.ekf
                    x_hat = rk4(@(~,state) self.L.f(state,u),[self.t,t],self.x_hat);
                    %tic
                    % Long
                    A = self.L.A(x,u);
                    %toc
                    self.L.P = rk4(@(~,covariance) self.covariance(covariance,A),[self.t,t],self.L.P);
                    if any(y_m ~= self.position)
                        i = y_m~=self.position;
                        %tic
                        % Long
                        C = self.L.C(x_hat,u,i);
                        %toc
                        self.L.L = self.L.P*C.'*(self.L.R(i,i)+C*self.L.P*C.')^-1;
                        self.L.P = (eye(length(x_hat))-self.L.L*C)*self.L.P;
                        y_m_hat = self.L.h(x_hat,u);
                        x_hat = x_hat + self.L.L*(controllers.get_error(y_m_hat(i),y_m(i),self.m_is_angle(i)));
                        self.position = y_m;
                    end
            end
            %toc
            
            % Save Values
            self.t = t;
            self.x_hat = x_hat;
            
            % Pack
            d_hat = x_hat(end-(length(self.d_indexes)-1):end);
            x_hat = x_hat(1:(length(x_hat)-length(self.d_indexes)));
        end
    end
end

