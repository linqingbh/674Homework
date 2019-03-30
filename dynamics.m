classdef dynamics < handle
    
    properties
        
        % General Parameters for passing information
        core
        param

        % State
        command
        x
        r
        u

        % Functions
        eqs_motion
        sensors
        filters
        observers
        get_y_m
        get_y_r
        planner
        manager
        followers
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
            self.command = param.command;
            self.x = core.subscribe('x');
            self.u = core.subscribe('u');
            
            % Functions
            self.eqs_motion = functions.eqs_motion;
            self.sensors = functions.sensors;
            self.filters = functions.filters;
            self.observers = functions.observers;
            self.get_y_m = functions.get_y_m;
            self.get_y_r = functions.get_y_r;
%             self.planner = functions.planner;
%             self.manager = functions.manager;
%             self.followers = functions.followers;
            self.controllers = functions.controllers;
            
            % Uncertian Param
            self.theoretical_param = param;
            self.param = uncertainty(self.theoretical_param,param.D_in_param,param.uncertian_param);
            
            % Settings
            self.progress_update = settings.progress_update;
            
        end
        
        function simulate(self)
            
            t = self.core.subscribe_history('t');
            
            disp("Begining system analysis.")
            
            tic
            
            % Iterate through each timestep
            for i = 2:length(t)
                self.r = self.command(:,i);
                
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
                    [x_hat(self.observers(j).x_indexes),d_hat(self.observers(j).d_indexes)] = self.observers(j).observe(self.x,y_m_hat,self.r,self.u,d,t(i));
                end

                % Convert
                [y_r_hat,y_r_dot_hat] = self.get_y_r(z_f,x_hat,d_hat,self.param);
                [y_r,y_r_dot] = self.get_y_r(z,self.x,d,self.param);

                % Path Planner
%                 W = self.planner.plan(x_hat,self.pe);
%                 
%                 % Path Manager
%                 leg = self.manager.manage(W,x_hat);
%                 
%                 % Path Follower
%                 self.r = zeros(size(self.param.r_names));
%                 for j = 1:length(self.followers)
%                     self.r(self.followers(j).r_indexes) = self.followers(j).follow(leg,x_hat);
%                 end
                
                % Implimennt controller
                for j = 1:length(self.controllers)
                    [self.u(self.controllers(j).u_indexes),self.r] = self.controllers(j).control(x_hat,y_r_hat,y_r_dot_hat,self.r,d_hat,t(i));
                end

                % Save history
%                 self.core.publish_update('W',W);
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
%                 self.core.publish_update('path',leg.line);
                self.core.publish_specific('r',self.r,i);
                self.core.publish('u',self.u);
                
                % Print Progress Indicator
                if self.progress_update
                    new_percent_done = round(i/length(t)*100);
                    if abs(new_percent_done - self.percent_done) > 1
                        disp(string(self.percent_done)+"%")
                        self.percent_done = new_percent_done;
                    end
                end
            end
            disp("100%")
            disp("System successfully analyzed.")
            toc
        end
    end
end