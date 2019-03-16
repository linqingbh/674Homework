classdef sensors < handle
    properties
        % Error
        sigma
        eta
        beta = 0 % Add randomly scalled bias ###################################
        sigma_hat = 0
        beta_hat = 0
        
        % Saturation
        sat_lim = struct('high',Inf,'low',-Inf)
        
        % Basic Sensor Functionality
        sensor_function
        update_rate = 100
        last_update = -Inf
        
        % For Implimentation
        d_indexes
        x_indexes
        z_indexes
        y_m
        
        % Gps Specific Values
        k_gps = 1/1100;
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
            
            self.d_indexes = get_indexes(param.d_names,sense.d_names);
            self.x_indexes = get_indexes(param.x_names,sense.x_names);
            self.z_indexes = get_indexes(param.z_names,sense.z_names);
            self.eta = @self.normal_error;
            
            switch sense.type
                case self.GPS
                    self.sensor_function = @self.execute_GPS;
                    self.eta = @self.gps_error;
                    self.sigma = [0.21;0.21;0.40;0.05];
                    self.update_rate = 1;
                case self.Bar
                    self.sensor_function = @self.execute_Bar;
                    self.sigma = 10;
                    self.beta = 0;
                    self.sat_lim.high = 100000;
                    self.sat_lim.low = 0;
                case self.Pito
                    self.sensor_function = @self.execute_Pito;
                    self.sigma = 2;
                    self.beta = 0;
                    self.sat_lim.high = 4000;
                    self.sat_lim.low = 0;
                case self.Comp
                    self.sensor_function = @self.execute_Comp;
                    self.sigma = 0.03*pi/180;
                    self.beta = 1*pi/180+param.declination;
                    self.update_rate = 8;
                case self.Accel
                    self.sensor_function = @self.execute_Accel;
                    self.sigma = 0.0025*9.81;
                    self.sat_lim.high = 6*9.81;
                    self.sat_lim.low = -6*9.81;
                case self.RateGyro
                    self.sensor_function = @self.execute_RateGyro;
                    self.sigma = 0.13*pi/180;
                    self.sat_lim.high = 350*pi/180;
                    self.sat_lim.low = -350*pi/180;
                case self.Exact
                    self.sensor_function = @self.execute_Exact;
                    self.sigma = 0;
            end
            if sense.perfect
                self.sat_lim.high = Inf;
                self.sat_lim.low = -Inf;
                self.sigma = zeros(size(self.sigma));
                self.beta = zeros(size(self.beta));
            end
        end

        function nu = gps_error(self,~)
            nu = exp(-self.k_gps.*1./self.update_rate).*self.nu + self.sigma(1:3).*randn(3,1);
            self.nu = nu;
            nu(4) = self.sigma(4).*randn;
            nu(5) = self.sigma(4).*randn./10;
            
        end
        
        function eta = normal_error(self,~)
            eta = randn.*self.sigma;
        end
        
        function eta = no_error(~,~)
            eta = 0;
        end
        
        function [truth,reading] = sense(self,x,x_dot,d,t)
            dt = t - self.last_update;
            
            % Get Significant Variables
            x = x(self.x_indexes);
            x_dot = x_dot(self.x_indexes);
            d = d(self.d_indexes);

            % Use sensors to sense value
            truth = self.sensor_function(x,x_dot,d);
            
            if dt >= 1/self.update_rate
                if length(truth) == 5 && t > 0
                    throw =1;
                end
                
                % Saturate
                sat_truth = min(truth,self.sat_lim.high);
                sat_truth = max(sat_truth,self.sat_lim.low);

                % Add Error
                reading = sat_truth + (self.beta-self.beta_hat) + self.eta(sat_truth);
                
                % Save
                self.y_m = reading;
                self.last_update = t;
            else
                reading = self.y_m;
            end
        end
    end
    
    methods (Static)
        % Sensor Functions ------------------------------------------------
        function y_m = execute_GPS(x,x_dot,~)
            p_n=x(1);p_e=x(2);h=-x(3);
            V_n=x_dot(1);V_e=x_dot(2);
            
            y_m = zeros(5,1);
            y_m(1) = p_n;
            y_m(2) = p_e;
            y_m(3) = h;
            y_m(4) = sqrt(V_n^2+V_e^2);
            y_m(5) = atan2(V_e,V_n);
        end
        
        function y_m = execute_Bar(x,~,~)
            h = -x(1);
            g = 9.81;
            air = atmosphere(h);
            y_m = air.rho*g*h;
        end
        
        function y_m = execute_Pito(x,~,d)
            h=-x(1);u=x(2);phi=x(3);theta=x(4);psi=x(5);
            V_w = get_rotation(phi,theta,psi,'v->b',d);
            u_w = V_w(1);
            air = atmosphere(h);
            y_m = air.rho*(u-u_w)^2/2;
        end
        
        function y_m = execute_Comp(x,~,~)
            y_m = x;
        end
        
        function y_m = execute_Accel(x,x_dot,~)
            u=x(1);v=x(2);w=x(3);phi=x(4);theta=x(5);p=x(6);q=x(7);r=x(8);
            u_dot=x_dot(1);v_dot=x_dot(2);w_dot=x_dot(3);
            g = 9.81;
            
            y_m = zeros(3,1);
            y_m(1) = u_dot+q*w-r*v+g*sin(theta);
            y_m(2) = v_dot+r*u-p*w-g*cos(theta)*sin(phi);
            y_m(3) = w_dot+p*v-q*u-g*cos(theta)*cos(phi);
        end
        
        function y_m = execute_RateGyro(x,~,~)
            y_m = x;
        end
        
        function y_m = execute_Exact(x,~,~)
            y_m = x;
        end
        
        % Calibration -----------------------------------------------------
        function covariance_matrix = callibrate_sensors(core,x,iterations)
            if ~exist('iterations','var'),iterations = 100;end
            
            sensors = core.functions.sensors;
            get_y_m = core.functions.get_y_m;
            param = core.param;
            
            for i = 1:iterations
                for j = 1:length(sensors)
                    last_update = sensors(j).last_update;
                    last_reading = sensors(j).y_m;
                    [z(sensors(j).z_indexes,i),z_hat(sensors(j).z_indexes,i)] = sensors(j).sense(x,zeros(size(x)),param.wind.base,sensors(j).last_update+1/sensors(j).update_rate); %#ok<AGROW>
                    sensors(j).last_update = last_update;
                    sensors(j).y_m = last_reading;
                end
                y_m(i,:) = get_y_m(z(:,i),param);
                y_m_hat(i,:) = get_y_m(z_hat(:,i),param);
            end
            
            covariance_matrix = cov(y_m_hat);
            covariance_matrix(14,14) = 4;% fix
            covariance_matrix(15,15) = 4;% fix
            bias_list = mean(z_hat-z,2);
            
            for j = 1:length(sensors)
                sensors(j).beta_hat = bias_list(sensors(j).z_indexes);
            end
            
        end
    end
end