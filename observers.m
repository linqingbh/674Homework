classdef observers < handle
    
    properties
        % General to pass to other functions
        param

        % States
        x_e
        u_e
        x_hat
        x_indexes
        m_indexes
        r_indexes
        u_indexes
        position
        velocity

        % Param
        L

        % Settings
        type
        m_is_angle
        
        % Defaults
        t = 0;
        d_hat = 0;
        convert = @(in) in
    end
    
    properties (Constant)
        exact = 'Exact'
        luenberger = 'Luenberger Observer'
        de = 'Dirivative based on error'
        dy = 'Dirivative based on state'
    end
    
    methods
        function self = observers(observe,core)
            
            % Unpack
            param = core.param;
            functions = core.functions;
            
            % General to pass to other functions
            self.param = param;
            
            % States
            x_0 = core.subscribe('x_hat');
            y_m = core.subscribe('y_m');
            [self.x_e,self.u_e] = functions.get_equilibrium(x_0,param);
            self.x_hat = x_0(self.x_indexes);
            self.x_indexes = get_indexes(param.x_names,observe.x_names);
            self.m_indexes = get_indexes(param.m_names,observe.m_names);
            self.r_indexes = get_indexes(param.r_names,observe.r_names);
            self.u_indexes = get_indexes(param.u_names,observe.u_names);
            self.position = y_m(self.m_indexes);
            self.velocity = zeros(size(self.position));
            
            % Param
            self.L = observe.L;
            
            % Settings
            self.type = observe.type;
            self.m_is_angle = param.m_is_angle(self.m_indexes);
            
            
            % Observer Specific
            switch self.type
                case self.exact
                case self.luenberger
                case self.de
                case self.dy
            end
        end
        
        % Luenberger Observer ---------------------------------------------
        function x_hat_dot = observer_eqs(self,~,x_hat,y_m,u)
            x_hat_dot = self.L.A*(x_hat - self.x_e) + self.L.B*(u-self.u_e + self.d_hat) + self.L.L*(y_m - self.C_m*x_hat);
        end
        
        function d_hat_dot = dist_eqs(self,~,~,x_hat,y_m)
            d_hat_dot = self.L.d*(y_m - self.L.C_m*x_hat);
        end
        
        % Dirty Diriviative -----------------------------------------------
        function velocity = dirty_direvative(self,position,beta,dt)
            velocity = beta.*self.velocity + (1-beta)./dt.*(position-self.position);
        end
        
        % Main ------------------------------------------------------------
        function [x_hat,d_hat] = observe(self,y_m,r,u,t)
            
            % Unpack
            dt = t - self.t;
            y_m = y_m(self.m_indexes);
            r = r(self.r_indexes);
            u = u(self.u_indexes);
            
            % Obtain measurment
            switch self.type
                case self.exact
                    x_hat = y_m;
                case self.luenberger
                    x_hat = rk4(@(time,state) self.observer_eqs(time,state,y_m,u),[0,dt],self.x_hat,true);
                    self.d_hat = rk4(@(time,state) self.dist_eqs(time,state,self.x_hat,y_m),[0,dt],self.d_hat,true);
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
            end
                
            % Math
            self.x_hat = self.convert(x_hat);
            
            % Save Values
            self.t = t;
            
            % Pack
            x_hat = self.x_hat;
            d_hat = self.d_hat;
        end
    end
end

