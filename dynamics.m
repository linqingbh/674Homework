classdef dynamics < handle
    
    properties
        
        % General Parameters for passing information
        core
        param

        % State
        x
        u

        % Functions
        eqs_motion
        sensors
        filters
        observers
        get_y_m
        get_y_r
        controllers
        controller_architecture

        % Uncertianty
        theoretical_param
        
        % Settings
        progress_update
        percent_done = 0;
    end
    
    methods
        function self = dynamics(core)
            % General Parameters for passing information
            param = core.param;
            functions = core.functions;
            settings = core.settings;
            self.core = core;
            self.param = param;
            
            % Initialize State
            self.x = core.subscribe('x');
            self.u = core.subscribe('u');
            
            % Functions
            self.eqs_motion = functions.eqs_motion;
            self.sensors = functions.sensors;
            self.filters = functions.filters;
            self.observers = functions.observers;
            self.get_y_m = functions.get_y_m;
            self.get_y_r = functions.get_y_r;
            self.controllers = functions.controllers;
            self.controller_architecture = functions.controller_architecture;
            
            % Uncertian Param
            self.theoretical_param = param;
            self.param = uncertainty(self.theoretical_param,param.D_in_param,param.uncertian_param);
            
            % Settings
            self.progress_update = settings.progress_update;
            
        end
        
        function simulate(self)
            
            r = self.core.subscribe_history('r');
            t = self.core.subscribe_history('t');
            
            disp("Begining system analysis.")
            
            tic
            % Iterate through each timestep
            for i = 2:length(r)
                % Propigate Dynamic Model
                [self.x,x_dot,d] = rk4(@(time,state) self.eqs_motion(time,state,self.u,self.param),[t(i-1),t(i)],self.x);

                % Measure
                z = zeros(size(self.param.z_names));
                z_hat = zeros(size(self.param.z_names));
                for j = 1:length(self.sensors)
                    [z(self.sensors(j).z_indexes),z_hat(self.sensors(j).z_indexes)] = self.sensors(j).sense(self.x,x_dot,d,t(i));
                end

                % Filter
                z_f = self.filters.filter_next(z_hat);

                % Convert
                y_m_hat = self.get_y_m(z_f,self.param);
                y_m = self.get_y_m(z,self.param);

                % Observe State
                x_hat = zeros(size(self.x));
                d_hat = zeros(size(d));
                for j = 1:length(self.observers)
                    [x_hat(self.observers(j).x_indexes),d_hat(self.observers(j).d_indexes)] = self.observers(j).observe(self.x,y_m_hat,r(:,i),self.u,d,t(i));
                end

                % Convert
                [y_r_hat,y_r_dot_hat] = self.get_y_r(z_f,x_hat,d_hat,self.param);
                [y_r,y_r_dot] = self.get_y_r(z,self.x,d,self.param);

                % Implimennt controller
                [self.u,r(:,i)] = self.controller_architecture(self.controllers,y_r_hat,y_r_dot_hat,r(:,i),d_hat,t(i),self.param);

                % Save history
                self.core.publish_specific('r',r(:,i),i);
                self.core.publish('d',d);
                self.core.publish('x',self.x);
                self.core.publish('x_dot',x_dot);
                self.core.publish('z',z);
                self.core.publish('z_f',z_f);
                self.core.publish('z_hat',z_hat);
                self.core.publish('y_m',y_m);
                self.core.publish('y_m_hat',y_m_hat);
                self.core.publish('x_hat',x_hat);
                self.core.publish('d_hat',d_hat);
                self.core.publish('y_r',y_r);
                self.core.publish('y_r_hat',y_r_hat);
                self.core.publish('y_r_dot',y_r_dot);
                self.core.publish('y_r_dot_hat',y_r_dot_hat);
                self.core.publish('u',self.u);
                
                % Print Progress Indicator
                if self.progress_update
                    new_percent_done = round(i/length(r)*100);
                    if abs(new_percent_done - self.percent_done) > 1
                        disp(string(self.percent_done)+"%")
                        self.percent_done = new_percent_done;
                    end
                end
            end
            disp("100%")
            disp("System successfully analized.")
            toc
        end
    end
end