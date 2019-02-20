classdef dynamics < handle
    
    properties
        x
        u
        param
        core
        settings
        theoretical_param
        time
        step
        stop
        eqs_motion
        controllers
        observers
        controller_architecture
        D_in_param
        D_in_u
        D_out
        N
        uncertian_param
        uncertian_u
        uncertian_x
        uncertian_N
        implement_uncertainty
        u_names
        x_names
        y_r
    end
    
    methods
        function self = dynamics(core)
            % General Parameters for passing information
            param = core.param;
            settings = core.settings;
            functions = core.functions;
            self.core = core;
            self.param = param;
            self.settings = settings;
            
            % Initialize State
            self.x = core.subscribe('x');
            self.u = core.subscribe('u');
            self.x_names = core.param.x_names;
            self.u_names = core.param.u_names;
            
            % Functions
            self.eqs_motion = functions.eqs_motion;
            self.controllers = functions.controllers;
            self.observers = functions.observers;
            self.controller_architecture = functions.controller_architecture;
            self.y_r = functions.y_r;
            
            % Simulation Parameters
            self.time = settings.start;
            self.step = settings.step;
            self.stop = settings.end;
            
            % Uncertianty
            self.theoretical_param = param;
            self.uncertian_u = param.uncertian_u;
            self.uncertian_x = param.uncertian_x;
            self.uncertian_N = param.uncertain_N;
            self.D_in_u = param.D_in_u;
            self.D_out = param.D_out;
            self.N = param.N;
            self.implement_uncertainty = settings.implement_uncertainty;
            if self.implement_uncertainty
                % set biases in parameters
                param.D_in_param.bias = param.D_in_param.bias.*(rand(1,length(param.D_in_param.bias)).*2-1);
                self.D_in_u.bias = self.D_in_u.bias.*(rand(1,length(self.D_in_u.bias)).*2-1);
                self.D_out.bias = self.D_out.bias.*(rand(1,length(self.D_out.bias)).*2-1);
                self.N.bias = self.N.bias.*(rand(1,length(self.N.bias)).*2-1);

                % set parameters randomness
                self.param = self.uncertainty(self.theoretical_param,param.D_in_param,param.uncertian_param);
            else
                self.param = param;
            end
            
        end
        
        function new_state = propagate(self)
            
            % Impliment uncertainty
            if self.implement_uncertainty
                self.u = self.uncertainty(self.u,self.D_in_u,self.uncertian_u);
            end
            
%             if abs(new_state(5))>10
%                 throw = 1;
%             end
            
            % Runga Kuta
            k1 = self.eqs_motion(self.step,self.x,self.u,self.core);
            k2 = self.eqs_motion(self.step,self.x + self.step/2*k1, self.u,self.core);
            k3 = self.eqs_motion(self.step,self.x + self.step/2*k2, self.u,self.core);
            k4 = self.eqs_motion(self.step,self.x + self.step*k3, self.u,self.core);
            new_state = self.x + self.step/6 * (k1 + 2*k2 + 2*k3 + k4);

            % Impliment uncertainty
            if self.implement_uncertainty
                new_state = self.uncertainty(new_state,self.D_out,self.uncertian_x);
            end
              
            % Pack results
            self.x = new_state;
            
        end
        
        function simulate(self)
            
            r = self.core.subscribe_history('r');
            t = self.core.subscribe_history('t');
            
            % Iterate through each timestep
            for i = 1:length(r)-1
                
                self.time = t(i+1);
                
                if self.time >=15.9
                    throw = 1;
                end
                
                self.x = self.propagate();
                
                [y_r,y_r_dot] = self.y_r(self.x,self.core);
                
                % Impliment uncertianty
                if self.implement_uncertainty
                    sensor_readings = self.uncertainty(self.x,self.N,self.uncertian_N);
                else
                    sensor_readings = self.x;
                end
                
                % Measure
                measurements = zeros(size(sensor_readings));
                d_hat = zeros(size(self.observers));
                for j = 1:length(self.observers)
                    indexes = self.get_indexes(self.x_names,self.observers(j).output_names);
                    [measurements(indexes),d_hat(j)] = self.observers(j).observe(sensor_readings);
                end
                    
                % Implimennt controller
                self.u = self.controller_architecture(self.controllers,measurements,r(:,i),d_hat,self.param);
%                 for j = 1:length(self.controllers)
%                     indexes = self.get_indexes(self.u_names,self.controllers(j).output_names);
%                     self.u(indexes) = self.controllers(j).control(measurements,r(:,i),d_hat(j));
%                 end
                    
                % Save history
                self.core.publish('u',self.u);
                self.core.publish('x_hat',measurements)
                
                % Simulate Responce to new input
                self.core.publish('x',self.x);
                self.core.publish('y_r',y_r);
                self.core.publish('y_r_dot',y_r_dot);
            end
        end
        
        function output = uncertainty(self,input,uncertainty,keys)    
            
            % Scale the uncertianty
            scale = uncertainty.random.*(rand(1,length(uncertainty.random)).*2-1);
            
            % If we're adjusting the parameters
            if isstruct(input)
                
                output = input;
                for i = 1:length(keys)
                    % Only impliment the uncertianty for the variables given.
                    variable = keys{i};
                    code = ['output.',variable,' = input.',variable,'+ scale(i) + uncertainty.bias(i);'];
                    eval(code)
%                     % This is to see the new values if needed
%                     disp(variable)
%                     eval(['disp(input.',variable,')'])
%                     eval(['disp(output.',variable,')'])
                end
                
             else % if we're adjusting any other number
                
                output = zeros(size(input));
                for i = 1:length(input)
                    % Only make uncertian if the key tells us to.
                    if keys(i)
                        output(i) = input(i) + scale(i) + uncertainty.bias(i);
                    else
                        output(i) = input(i);
                    end
                end
                
            end
        end
        
        function indexes = get_indexes(self,options,names)
            indexes = false(size(options));
            for i = 1:length(names)
                indexes = indexes | strcmp(options,names(i));
            end
        end
    end
end

