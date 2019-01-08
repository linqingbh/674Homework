classdef animate < handle
    
    properties
        get_drawing
        window
        animation_handle
        figure_handle
        display_time
        real_time
        plot_names
        plot_handles
        x_names
        x_hat_names
        u_names
        r_names
        e_names
        start
        step
        animation
        animation_figure
        plot_figure
        observer
        core
        param
        settings
        t
    end
    
    methods
        function self = animate(core)
            
            settings = core.settings;
            functions = core.functions;
            param = core.param;
            
            % Unpack data
            % Display Window Info
            self.window = settings.window;
            self.display_time = settings.publish;
            % Playback Info
            self.real_time = settings.real_time;
            self.start = settings.start;
            self.step = settings.step;
            self.plot_names  = settings.plot_names;
            self.animation = settings.animation;
            self.observer = any(strcmp(settings.plot_names,"e - Observer Error"));
            % Handle to the function that draws the simulation
            self.get_drawing = functions.get_drawing;
            % General Parameters for passing information
            self.core = core;
            self.param = core.param;
            self.settings = core.settings;
            % Names
            self.x_names = param.x_names;
            self.u_names = param.u_names;
            self.x_hat_names = param.x_names + "_{hat}";
            self.e_names = "e_{" + param.x_names + "}";
            
            % Publishers & Subscribers
            x = core.subscribe_specific('x',1);
            x_hat = core.subscribe_specific('x_hat',1);
            u = core.subscribe_specific('u',1);
            r = core.subscribe_specific('r',1);
            t = core.subscribe_specific('t',1);
            e = x-x_hat;
            core.param.r_names = self.r_names;
            core.param.x_hat_names = self.x_hat_names;
            core.param.e_names = self.e_names;
            
            % Assign figure numbers
            if self.animation
                self.animation_figure = 1;
                self.plot_figure = 2;
            else
                self.plot_figure = 1;
            end
            
            % Animation Figure
            if self.animation
                
                % Get points
                [points,colors] = self.get_drawing(x,self.settings,self.param);
                
                % Plot
                figure(self.animation_figure); 
                clf
                hold on
                for i = 1:length(colors)
                    self.animation_handle(i) = fill3(points{i}(:,1),points{i}(:,2),points{i}(:,3),colors{i});
                end
                axis equal
                axis(self.window)
                hold off
            end
            
            % Plot Data Figure
            number_of_plots = length(self.plot_names);
            figure(self.plot_figure), clf
            for i = 1:number_of_plots
                subplot(number_of_plots,1,i,'FontSize', 10)
                hold on
                for j = 2:length(self.plot_names{i})
                    [index,type] = self.get_indexes(self.plot_names{i}(j));
                    switch type
                        case "x"
                            data = x(index,:);
                        case "x_hat"
                            data = x_hat(index,:);
                        case "u"
                            data = u(index,:);
                        case "r"
                            data = r(index,:);
                        case "e"
                            data = e(index,:);
                    end
                            
                    self.plot_handles{i}(j) = plot(t,data,'linewidth',1.5);
                end
                grid on
                xlabel('t - Time (s)')
                ylabel(self.plot_names{i}(1))
                legend(self.plot_names{i}(2:end))
                hold off
            end
            
            % Reactivate the animation figure
            figure(1)
        end
        
        function redraw(self,x)
            
            % Place the points of the drawing in the new location
            [points,colors] = self.get_drawing(x,self.settings,self.param);          
            
            % Update the chart data.
            for i = 1:length(colors)
                set(self.animation_handle(i),...
                    'X',points{i}(:,1),...
                    'Y',points{i}(:,2),...
                    'Z',points{i}(:,3))
            end
            
            % Redraw
            drawnow
        end
        
        function add_data(self,x,x_hat,e,u,r,t)
            
            % Plot Data Figure
            for i = 1:length(self.plot_names)
                for j = 2:length(self.plot_names{i})
                    [index,type] = self.get_indexes(self.plot_names{i}(j));
                    switch type
                        case "x"
                            data = x(index,:);
                        case "x_hat"
                            data = x_hat(index,:);
                        case "u"
                            data = u(index,:);
                        case "r"
                            data = r(index,:);
                        case "e"
                            data = e(index,:);
                    end
                            
                    set(self.plot_handles{i}(j),'XData',t,'YData',data);
                end
            end
            
            % Redraw
            drawnow
        end
        
        function play(self)
            
            x = self.core.subscribe_history('x');
            x_hat = self.core.subscribe_history('x_hat');
            u = self.core.subscribe_history('u');
            r = self.core.subscribe_history('r');
            t = self.core.subscribe_history('t');
            e = x - x_hat;
            
            if self.animation
                
                % Start timers for accurate timing
                printwatch = tic; % Timer that helps skip unessisary images
                stopwatch = tic; % Timer that iterates through each image
                j = 0;

                % Iterate through each position
                for i = 1:length(x)

                    % Wait for the first timer to indicate it's time to move on
                    % to the next image
                    while (toc(stopwatch) < t(i)) && self.real_time
                    end

                    % If the second timer indicates that it is also time to
                    % display a new image.
                    if ((toc(printwatch) > self.display_time) && self.real_time) || (~self.real_time && (t(i)>j*self.display_time))
                        
                        j = j + 1;

                        % Reset the timer
                        printwatch = tic;

                        % Update the dispaly
                        self.redraw(x(:,i));
                        self.add_data(x(:,1:i),x_hat(:,1:i),e(:,1:i),u(:,1:i),r(:,1:i),t(:,1:i))
                    end
                end
                
                % For information's sake, display the time of the output
                toc(stopwatch)
                
            else
                % Update the plots only
                self.add_data(x,x_hat,u,r,t)
            end
        end
        
        function [index,type] = get_indexes(self,name)
            if any(strcmp(self.x_names,name))
                index = strcmp(self.x_names,name);
                type = 'x';
            elseif any(strcmp(self.x_hat_names,name))
                index = strcmp(self.x_hat_names,name);
                type = 'x_hat';
            elseif any(strcmp(self.u_names,name))
                index = strcmp(self.u_names,name);
                type = 'u';
            elseif any(strcmp(self.r_names,name))
                index = strcmp(self.r_names,name);
                type = 'r';
            elseif any(strcmp(self.e_names,name))
                index = strcmp(self.e_names,name);
                type = 'e';
            else
                error("ERROR: Your trying to plot '" + name + "' which is not a plotable value.")
            end
        end
    end
end

