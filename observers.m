classdef observers < handle
    
    properties
        A
        B
        C_m
        L
        u_e
        x_e
        dt
        x_hat
        param
        x_indexes
        u_indexes
        y_m_indexes
        x_names
        u_names
        y_m_names
        use_names
        output_names
        type
        sigma
        old_x
        core
        x
        beta
        d_hat
        t_gps
        t_sensor
        time
    end
    
    properties (Constant)
        O = 'Observer';
        de = 'Dirivative based on error';
        dp = 'Dirivative based on position';
        m = 'Measure';
        e = 'exact';
        gps = 'gps';
        RateGyro = 'reat'
    end
    
    methods
        function self = observers(observe,core)
            
            % Unpack
            param = core.param;
            settings = core.settings;
            functions = core.functions;
            
            % Observer settings
            self.type = observe.type;
            switch self.type
                case self.O
                    self.L = observe.L;
                case self.de
                    self.sigma = observe.L;
                    self.x = param.x_0;
                    self.L = observe.L;
                case self.dp
                    self.sigma = observe.L;
                    self.L = observe.L;
                case self.e
                    
                case self.m
                otherwise
                    % Parmeters
                    self.x_names = param.x_names;
                    self.u_names = param.u_names;
                    self.x_e = param.x_e(self.get_indexes(param.x_names,observe.x_names));
                    self.y_m_names = param.x_names(param.C_m*(1:length(self.x_hat)).'); 
                    self.x_indexes = self.get_indexes(self.x_names,self.output_names);
                    self.u_indexes = self.get_indexes(self.u_names,self.use_names);
                    self.y_m_indexes = self.get_indexes(self.y_m_names,self.output_names);
                    self.A = param.A(self.x_indexes,self.x_indexes);
                    self.B = param.B(self.x_indexes,self.u_indexes);
                    self.C_m = param.C_m(self.y_m_indexes,self.x_indexes);
                    self.x_hat = self.x_hat(self.x_indexes);
                    self.dt = settings.step;
                    self.u_e = functions.u_e(self.x_e,param);
                    self.u_e = self.u_e(self.get_indexes(param.u_names,observe.u_names));
            end
            
            self.output_names = observe.x_names;
            self.use_names = observe.u_names;
            
            % Initalize
            self.x_hat = core.subscribe('x_hat');
            self.x = core.subscribe('x');
            self.d_hat = 0;
            self.t_sensor = -Inf;

            % Parmeters
            self.x_names = param.x_names;
            self.u_names = param.u_names;
            self.x_e = param.x_e(self.get_indexes(param.x_names,observe.x_names));
            self.x_indexes = self.get_indexes(self.x_names,self.output_names);
            self.u_indexes = self.get_indexes(self.u_names,self.use_names);
            self.x_hat = self.x_hat(self.x_indexes);
            self.dt = settings.step;
            self.u_e = functions.u_e(self.x_e,param);
            self.u_e = self.u_e(self.get_indexes(param.u_names,observe.u_names));

            % General
            self.param = param;
            self.core = core;
        end
        
        function x_hat_dot = observer_eqs(self,x_hat,y_m,u)
            x_hat_dot = self.A*(x_hat - self.x_e) + self.B*(u-self.u_e + self.d_hat) + self.L.L*(y_m - self.C_m*x_hat);
        end
        
        function d_hat_dot = dist_eqs(self,x_hat,y_m)
            d_hat_dot = self.L.d*(y_m - self.C_m*x_hat);
        end
        
        function update_gps(self)
        end
        
        function update_rate_gyros(self)
        end
        
        function update_accelorometers(self)
        end
        
        function update_pressure_sensors(self)
        end
        
        function [x_hat,d_hat] = observe(self,x,t)
            
            self.time = t;
            
            x = x(self.get_indexes(self.x_names,self.output_names));
            
            % get data
%             y_m = self.C_m*x;
            u = self.core.subscribe('u');
            
            u = u(self.get_indexes(self.u_names,self.use_names));
            
            switch self.type
                case self.O
                    % Runga Kuta
                    k1 = self.observer_eqs(self.x_hat,y_m,u);
                    k2 = self.observer_eqs(self.x_hat + self.dt/2*k1,y_m,u);
                    k3 = self.observer_eqs(self.x_hat + self.dt/2*k2,y_m,u);
                    k4 = self.observer_eqs(self.x_hat + self.dt*k3,y_m,u);
                    x_hat = self.x_hat + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
                    
                    k1 = self.dist_eqs(self.x_hat,y_m);
                    k2 = self.dist_eqs(self.x_hat,y_m);
                    k3 = self.dist_eqs(self.x_hat,y_m);
                    k4 = self.dist_eqs(self.x_hat,y_m);
                    d_hat = self.d_hat + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
                case self.dp
                    x_hat = zeros(size(x));
                    
                    for k = 1:length(x_hat)/2
                        indexes_of_velocities = ~cellfun(@isempty,regexp(self.output_names,strcat(self.output_names(k),"_{dot}")));
                    
                        x_hat(~indexes_of_velocities) = x(~indexes_of_velocities);
                        
                        % find related
                        j = find(indexes_of_velocities);
                        
                        x_hat(j) = self.dirty_direvative(self.x(j),x(k),self.x(k));
                    end
                    d_hat = 0;
                    self.x = x_hat;
                case self.de
                    x_hat = zeros(size(x));
                    
                    for k = 1:length(x_hat)/2
                        indexes_of_velocities = ~cellfun(@isempty,regexp(self.output_names,strcat(self.output_names(k),"_{dot}")));
                    
                        x_hat(~indexes_of_velocities) = x(~indexes_of_velocities);
                        
                        % find related
                        j = find(indexes_of_velocities);
                        
                        x_hat(j) = self.dirty_direvative(self.x(j),x(k),self.x(k));
                    end
                    d_hat = 0;
                    self.x = x_hat;
                case self.e
                    x_hat = x;
                    d_hat = 0;
                case self.m
                    d_hat = 0;
            end
                
            % Save Values
            self.x_hat = x_hat;
            self.d_hat = d_hat;
        end
        
        function x_dot = dirty_direvative(self,old_x_dot,current_x,old_x)
            self.beta = (2.*self.sigma - self.dt)./(2.*self.sigma + self.dt);
            x_dot = self.beta.*old_x_dot + (1-self.beta)./self.dt.*(current_x-old_x);
        end
        
        function indexes = get_indexes(self,options,names)
            indexes = false(size(options));
            for i = 1:length(names)
                indexes = indexes | strcmp(options,names(i));
            end
        end
    end
end

