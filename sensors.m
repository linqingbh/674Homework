classdef sensors < handle
    properties
        % Error
        sigma
        eta
        beta = 0
        
        % Saturation
        sat_lim = struct('high',Inf,'low',-Inf)
        
        % Basic Sensor Functionality
        sensor_function
        update_rate = 100
        last_update = -Inf
        
        % For Implimentation
        x_indexes
        sensor_indexes
        filter
        y_m
        y_m_filtered
        
        % Gps Specific Values
        k_gps = 1/1600;
        nu = [0;0;0];
    end
    
    properties (Constant)
        GPS = "Global Positioning System"
        Bar = "Barometer"
        Pito = "Pito Tube"
        Comp = "Compass"
        Accel = "Accelerometer"
        RateGyro = "Rate Gyro"
        Exact = "Pass Exact States"
    end
    
    methods
        % Class Initializer -----------------------------------------------
        function self = sensors(sense,param)
            if ~isfield(sense,'exact'),sense.exact = false;end
            self.x_indexes = get_indexes(param.x_names,sense.x_names);
            self.sensor_indexes = get_indexes(param.sensor_names,sense.sensor_names);
            self.eta = @self.normal_error;
            switch sense.type
                case self.GPS
                    self.sensor_function = @self.execute_GPS;
                    self.eta = @self.gps_error;
                    self.sigma = [2.1;2.1;4.0];
                    self.update_rate = 1;
                    initial = [0;0;0;0;0];
                case self.Bar
                    self.sensor_function = @self.execute_Bar;
                    self.sigma = 10;
                    self.beta = 125;
                    self.sat_lim.high = 100000;
                    self.sat_lim.low = 0;
                    initial = 0;
                case self.Pito
                    self.sensor_function = @self.execute_Pito;
                    self.sigma = 2;
                    self.beta = 20;
                    self.sat_lim.high = 4000;
                    self.sat_lim.low = 0;
                    initial = 0;
                case self.Comp
                    self.sensor_function = @self.execute_Comp;
                    self.sigma = 0.3*pi/180;
                    self.beta = 1*pi/180+param.declination;
                    self.update_rate = 8;
                    initial = 0;
                case self.Accel
                    self.sensor_function = @self.execute_Accel;
                    self.sigma = 0.0025*9.81;
                    self.sat_lim.high = 6*9.81;
                    self.sat_lim.low = -6*9.81;
                    initial = [0;0;0];
                case self.RateGyro
                    self.sensor_function = @self.execute_RateGyro;
                    self.sigma = 0.0023;
                    self.sat_lim.high = 350*pi/180;
                    self.sat_lim.low = -350*pi/180;
                    initial = [0;0;0];
                case self.Exact
                    self.sensor_function = @self.execute_Exact;
                    self.sigma = 0;
                    initial = zeros(length(self.x_indexes),1);
            end
            self.filter = my_filter(1,0.5,initial,0);
            if sense.exact
                self.filter = my_filter(0,0.5,initial,0);
                self.sat_lim.high = Inf;
                self.sat_lim.low = -Inf;
                self.sigma = 0;
                self.beta = 0;
            end
        end

        function nu = gps_error(self,measurement)
            nu = exp(-self.k_gps.*self.update_rate).*self.nu + self.sigma.*randn(3,1);
            self.nu = nu;
            nu(4) = self.sigma(1).*randn;
            nu(5) = self.sigma(1).*randn./measurement(4);
            
        end
        
        function eta = normal_error(self,~)
            eta = randn.*self.sigma;
        end
        
        function eta = no_error(~,~)
            eta = 0;
        end
        
        function [reading,reading_filtered] = sense(self,x,x_dot,t)
            dt = t - self.last_update;
            
            if dt >= 1/self.update_rate

                % Get Significant Variables
                x = x(self.x_indexes);
                x_dot = x_dot(self.x_indexes);

                % Use sensors to sense value
                measurment = self.sensor_function(x,x_dot);
                
                % Saturate
                measurment = min(measurment,self.sat_lim.high);
                measurment = max(measurment,self.sat_lim.low);
                
                % Add Error
                reading = measurment + self.beta + self.eta(measurment);
                
                % Remove Error
                reading_filtered = self.filter.filter_next(reading,t);
                
                % Save
                self.y_m = reading;
                self.y_m_filtered = reading_filtered;
                self.last_update = t;
            else
                reading = self.y_m;
                reading_filtered = self.y_m_filtered;
            end
        end
    end
    
    methods (Static)
        % Sensor Functions ------------------------------------------------
        function y_m = execute_GPS(x,x_dot)
            p_n=x(1);p_e=x(2);h=-x(3);
            V_n=x_dot(1);V_e=x_dot(2);
            
            y_m = zeros(5,1);
            y_m(1) = p_n;
            y_m(2) = p_e;
            y_m(3) = h;
            y_m(4) = sqrt(V_n^2+V_e^2);
            y_m(5) = atan2(V_e,V_n);
        end
        
        function y_m = execute_Bar(x,~)
            g = 9.81;
            air = atmosphere(0);
            y_m = air.rho*g*(-x);
        end
        
        function y_m = execute_Pito(x,~)
            air = atmosphere(0);
            y_m = air.rho*x^2/2;
        end
        
        function y_m = execute_Comp(x,~)
            y_m = x;
        end
        
        function y_m = execute_Accel(x,x_dot)
            u=x(1);v=x(2);w=x(3);phi=x(4);theta=x(5);p=x(6);q=x(7);r=x(8);
            u_dot=x_dot(1);v_dot=x_dot(2);w_dot=x_dot(3);
            g = 9.81;
            
            y_m = zeros(3,1);
            y_m(1) = u_dot+q*w-r*v+g*sin(theta);
            y_m(2) = v_dot+r*u-p*w-g*cos(theta)*sin(phi);
            y_m(3) = w_dot+p*v-q*u-g*cos(theta)*cos(phi);
        end
        
        function y_m = execute_RateGyro(x,~)
            y_m = x;
        end
        
        function y_m = execute_Exact(x,~)
            y_m = x;
        end
        
    end
end