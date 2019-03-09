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
        observers
        get_y_m
        get_y_r
        controllers
        controller_architecture
        filters

        % Uncertianty
        implement_uncertainty
        theoretical_param
        uncertian_u
        uncertian_x
        uncertian_N
        D_in_u
        D_out
        N
    end
    
    methods
        function self = dynamics(core)
            % General Parameters for passing information
            param = core.param;
            settings = core.settings;
            functions = core.functions;
            self.core = core;
            self.param = param;
            
            % Initialize State
            self.x = core.subscribe('x');
            self.u = core.subscribe('u');
            
            % Functions
            self.eqs_motion = functions.eqs_motion;
            self.sensors = functions.sensors;
            self.observers = functions.observers;
            self.get_y_m = functions.get_y_m;
            self.get_y_r = functions.get_y_r;
            self.controllers = functions.controllers;
            self.controller_architecture = functions.controller_architecture;
            
            % Uncertianty
            self.implement_uncertainty = settings.implement_uncertainty;
            self.theoretical_param = param;
            self.uncertian_u = param.uncertian_u;
            self.uncertian_x = param.uncertian_x;
            self.uncertian_N = param.uncertain_N;
            self.D_in_u = param.D_in_u;
            self.D_out = param.D_out;
            self.N = param.N;
            if self.implement_uncertainty
                % set biases in parameters
                param.D_in_param.bias = param.D_in_param.bias.*(rand(1,length(param.D_in_param.bias)).*2-1);
                self.D_in_u.bias = self.D_in_u.bias.*(rand(1,length(self.D_in_u.bias)).*2-1);
                self.D_out.bias = self.D_out.bias.*(rand(1,length(self.D_out.bias)).*2-1);
                self.N.bias = self.N.bias.*(rand(1,length(self.N.bias)).*2-1);

                % set parameters randomness
                self.param = uncertainty(self.theoretical_param,param.D_in_param,param.uncertian_param);
            else
                self.param = param;
            end
            
        end
        
        function [new_state,new_state_dot] = propagate(self,dt)
            
            % Impliment uncertainty
            if self.implement_uncertainty
                actual_u = uncertainty(self.u,self.D_in_u,self.uncertian_u);
            else
                actual_u = self.u;
            end
            
            % Runga Kuta
            [self.x,new_state_dot] = rk4(@(t,state) self.eqs_motion(t,state,actual_u,self.param),[0,dt],self.x,true);
            
            
            % Impliment uncertainty
            if self.implement_uncertainty
                new_state = uncertainty(self.x,self.D_out,self.uncertian_x);
            else
                new_state = self.x;
            end
        end
        
        function simulate(self)
            
            r = self.core.subscribe_history('r');
            t = self.core.subscribe_history('t');
            
            % Iterate through each timestep
            for i = 2:length(r)
                
                dt = t(i) - t(i-1);
                
                [state,x_dot] = self.propagate(dt);

                % Impliment uncertianty
                if self.implement_uncertainty
                    uncertian_state = uncertainty(state,self.N,self.uncertian_N);
                else
                    uncertian_state = state;
                end

                % Measure
                z_hat = zeros(size(self.param.z_names));
                z_f = zeros(size(self.param.z_names));
                z = zeros(size(self.param.z_names));
                for j = 1:length(self.sensors)
                    [z_hat(self.sensors(j).z_indexes),z_f(self.sensors(j).z_indexes),z(self.sensors(j).z_indexes)] = self.sensors(j).sense(uncertian_state,x_dot,t(i));
                end
                
                % Convert
                y_m_hat = self.get_y_m(z_f,self.param);
                y_m = self.get_y_m(z,self.param);
                % Observe
                x_hat = zeros(size(state));
                d_hat = zeros(size(r(:,i)));
                for j = 1:length(self.observers)
                    [x_hat(self.observers(j).x_indexes),d_hat(self.observers(j).r_indexes)] = self.observers(j).observe(self.x,y_m_hat,r(:,i),self.u,t(i));
                end
                
                % Convert
                [y_r_hat,y_r_dot_hat] = self.get_y_r(z_f,x_hat,self.param);
                [y_r,y_r_dot] = self.get_y_r(z,self.x,self.param);
                % Need to fix d_hat!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                
                % Implimennt controller
                [self.u,r(:,i)] = self.controller_architecture(self.controllers,y_r_hat,y_r_dot_hat,r(:,i),d_hat(1),t(i),self.param);

                % Save history
                self.core.publish_specific('r',r(:,i),i);
                self.core.publish('x',self.x);
                self.core.publish('x_dot',x_dot);
                self.core.publish('z',z);
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
            end
        end
    end
end