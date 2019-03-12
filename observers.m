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
        m_indexes
        r_indexes
        u_indexes
        d_indexes
        position
        velocity

        % Settings
        type
        m_is_angle
        
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
            
            % Param
            self.L = observe.L;
            
            % States
            self.x_indexes = get_indexes(param.x_names,observe.x_names);
            self.m_indexes = get_indexes(param.m_names,observe.m_names);
            self.r_indexes = get_indexes(param.r_names,observe.r_names);
            self.u_indexes = get_indexes(param.u_names,observe.u_names);
            self.d_indexes = get_indexes(param.d_names,observe.d_names);
            x_0 = core.subscribe('x_hat');
            d_0 = core.subscribe('d_hat');
            y_m = core.subscribe('y_m');
            [self.x_e,self.u_e] = functions.get_equilibrium(x_0,param);
            self.x_hat = [x_0(self.x_indexes);d_0(self.d_indexes)];
            self.position = y_m(self.m_indexes);
            self.velocity = zeros(size(self.position));
            
            % Settings
            self.type = observe.type;
            self.m_is_angle = param.m_is_angle(self.m_indexes);  
            
            % Observer Specific
            switch self.type
                case self.exact
                case self.luenberger
                case self.de
                case self.dy
                case self.ekf
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
        function P_dot = covariance(self,P,x,u)
            P_dot = self.L.A(x,u)*P + P*self.L.A(x,u).' + self.L.Q;
        end
        
        % Main ------------------------------------------------------------
        function [x_hat,d_hat] = observe(self,x,y_m,r,u,d,t)
            
            % Unpack
            dt  = t - self.t;
            x   = [x(self.x_indexes);d(self.d_indexes)];
            y_m = y_m(self.m_indexes);
            r   = r(self.r_indexes);
            u   = u(self.u_indexes);
            
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
                    x_hat = rk4(@(~,state) self.L.f(state,y_m,r,u,d),[self.t,t],self.x_hat);
                    self.L.P = rk4(@(~,covariance) self.covariance(covariance),[self.t,t],self.L.P);
                    if y_m ~= self.position
                        self.L.L = self.L.P*self.L.C(x_hat,y_m,r,u,d).'*(self.L.R+self.L.C(x_hat,y_m,r,u,d)*self.L.P*self.L.C(x_hat,y_m,r,u,d).')^-1;
                        self.L.P = (eye(length(self.L.L))-self.L.L*self.L.C(x_hat,y_m,r,u,d))*self.L.P;
                        x_hat = x_hat + self.L.L*(y_m-self.L.h(x_hat,y_m,r,u,d));
                        self.position = y_m;
                    end
            end
            
            % Save Values
            self.t = t;
            self.x_hat = x_hat;
            
            % Pack
            d_hat = x_hat(end-(length(self.d_indexes-1)):end);
            x_hat = x_hat(1:length(x_hat));
        end
    end
end

