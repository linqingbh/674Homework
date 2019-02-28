classdef animate < handle
    
    properties
        % Display Window Info
        window
        view
        labels
        gph_per_img

        % Playback Info
        real_time
        publish
        step
        animation
        plot
        active_fig
        show_hist
        var_indexes

        % Handle to the function that draws the simulation
        get_drawing

        % General Parameters for passing information
        core
        param
        settings
        
        % Handles
        animation_handle
        history_handle
        plot_handles

        % Settings
        x_is_angle
        r_is_angle
    end
    
    methods
        function self = animate(core)
            
            % Unpack -----------------------------------------------------
            settings = core.settings;
            functions = core.functions;
            param = core.param;
            
            % Display Window Info
            self.window = settings.window;
            self.view = settings.view;
            self.labels = settings.labels;
            self.gph_per_img = settings.gph_per_img;
            
            % Playback Info
            self.real_time = settings.real_time;
            self.publish = settings.publish;
            self.step = settings.step;
            self.animation = settings.animation;
            self.plot = settings.plot;
            self.active_fig = settings.active_fig;
            self.show_hist = settings.show_hist;
            
            % Handle to the function that draws the simulation
            self.get_drawing = functions.get_drawing;
            
            % General Parameters for passing information
            self.core = core;
            self.param = core.param;
            self.settings = core.settings;
            
            % Settings
            self.x_is_angle = param.x_is_angle;
            self.r_is_angle = param.r_is_angle;
            
            % Names
            r_names = "r_{" + param.r_names + "}";
            x_names = param.x_names;
            x_dot_names = param.x_names + "_{dot}";
            sensor_names = param.sensor_names;
            y_m_names = "y_m_{" + param.m_names + "}";
            x_hat_names = "x_hat_{" + param.x_names + "}";
            d_hat_names = "d_hat_{" + param.r_names + "}";
            d_hat_e_names = "d_hat_e_{" + param.x_names + "}";
            y_r_names = "y_r_{" + param.r_names + "}";
            y_r_e_names = "y_r_e_{" + param.r_names + "}";
            y_r_dot_names = "y_r_dot_{" + param.r_names + "}";
            u_names = param.u_names;
            all_names = [r_names;x_names;x_dot_names;sensor_names;y_m_names;x_hat_names;d_hat_names;d_hat_e_names;y_r_names;y_r_e_names;y_r_dot_names;u_names];
            var_to_plot_names = [];
            for i = 1:length(settings.plot_names)
                var_to_plot_names = [var_to_plot_names;settings.plot_names{i}(2:end).']; %#ok<AGROW>
            end
            self.var_indexes = get_indexes(all_names,var_to_plot_names);
            
            
            % Publishers & Subscribers
            t = self.core.subscribe_specific('t',1);
            r = self.core.subscribe_specific('r',1);
            x = self.core.subscribe_specific('x',1);
            x_dot = self.core.subscribe_specific('x_dot',1);
            sensor_data = self.core.subscribe_specific('sensor_data',1);
            y_m = self.core.subscribe_specific('y_m',1);
            x_hat = self.core.subscribe_specific('x_hat',1);
            d_hat = self.core.subscribe_specific('d_hat',1);
            d_hat_e = controllers.get_error(x_hat,x,self.x_is_angle);
            y_r = self.core.subscribe_specific('y_r',1);
            y_r_e = controllers.get_error(y_r,r,self.r_is_angle);
            y_r_dot = self.core.subscribe_specific('y_r_dot',1);
            u = self.core.subscribe_specific('u',1);
            
            data = [r;x;x_dot;sensor_data;y_m;x_hat;d_hat;d_hat_e;y_r;y_r_e;y_r_dot;u];
            data = data(self.var_indexes,:);
            
            
            % Initialize Figures ------------------------------------------
            % Assign figure numbers
            number_of_plots = length(settings.plot_names);
            number_of_figs = ceil(number_of_plots/self.gph_per_img);
            if self.animation
                plot_figure = 2:number_of_figs+1;
            else
                plot_figure = 1:number_of_figs;
            end
            
            % Animation Figure
            if self.animation
                
                % Get points
                [points,colors,history] = self.get_drawing(x,self.settings,self.param);
                
                % Plot
                figure(1),clf; 
                clf
                hold on
                if self.show_hist
                    for i = 1:length(history)
                        self.history_handle(i) = plot3(history{i}(1,:),history{i}(2,:),history{i}(3,:),"-r",'linewidth',1.5);
                    end
                end
                for i = 1:length(colors)
                    self.animation_handle(i) = fill3(points{i}(1,:),points{i}(2,:),points{i}(3,:),colors{i});
                end
                axis equal
                axis(self.window)
                view(self.view)
                xlabel(self.labels(1))
                ylabel(self.labels(2))
                zlabel(self.labels(3))
                grid on
                hold off
            end
            
            % Plot Data Figure
            if self.plot
                l = 1;
                for k = plot_figure
                    figure(k), clf
                    if (k ~= plot_figure(end)) || ~rem(number_of_plots,self.gph_per_img) 
                        on_this_plot = self.gph_per_img;
                    else
                        on_this_plot = rem(number_of_plots,self.gph_per_img);
                    end
                    for i = 1:on_this_plot
                        subplot(on_this_plot,1,i,'FontSize', 10)
                        hold on
                        for j = 2:length(settings.plot_names{self.gph_per_img*(k-plot_figure(1))+i})
                            self.plot_handles(l) = plot(t,data(l,:),'linewidth',1.5);
                        end
                        grid on
                        xlabel('t - Time (s)')
                        ylabel(settings.plot_names{self.gph_per_img*(k-plot_figure(1))+i}(1))
                        legend(settings.plot_names{self.gph_per_img*(k-plot_figure(1))+i}(2:end))
                        hold off
                    end
                end
            end
            
            % Reactivate the animation figure
            figure(self.active_fig)
        end
        
        function redraw(self,x)
            
            % Place the points of the drawing in the new location
            [points,colors,history] = self.get_drawing(x,self.settings,self.param);          
            
            % Update the chart data.
            if self.show_hist
                for i = 1:length(history)
                    set(self.history_handle(i),...
                        'XData',history{i}(1,:),...
                        'YData',history{i}(2,:),...
                        'ZData',history{i}(3,:));
                end
            end
            for i = 1:length(colors)
                set(self.animation_handle(i),...
                    'X',points{i}(1,:),...
                    'Y',points{i}(2,:),...
                    'Z',points{i}(3,:))
            end
            
            % Redraw
            drawnow
        end
        
        function add_data(self,t,data)
            if self.plot
                % Plot Data Figure
                for i = 1:length(self.plot_handles)
                    set(self.plot_handles(i),'XData',t,'YData',data(i,:));
                end

                % Redraw
                drawnow
            end
        end
        
        function play(self)
            
            % Get time
            t = self.core.subscribe_history('t');
            time_indexes = 1:ceil(self.publish/self.step):length(t);
            
            % Get Data
            r = self.core.subscribe_history('r');
            x = self.core.subscribe_history('x');
            x_dot = self.core.subscribe_history('x_dot');
            sensor_data = self.core.subscribe_history('sensor_data');
            y_m = self.core.subscribe_history('y_m');
            x_hat = self.core.subscribe_history('x_hat');
            d_hat = self.core.subscribe_history('d_hat');
            d_hat_e = controllers.get_error(x_hat,x,self.x_is_angle);
            y_r = self.core.subscribe_history('y_r');
            y_r_e = controllers.get_error(y_r,r,self.r_is_angle);
            y_r_dot = self.core.subscribe_history('y_r_dot');
            u = self.core.subscribe_specific('u');
            
            data = [r;x;x_dot;sensor_data;y_m;x_hat;d_hat;d_hat_e;y_r;y_r_e;y_r_dot;u];
            data = data(self.var_indexes,time_indexes);
            
            if self.animation
                
                time_watch = tic; % Timer that iterates through each image

                % Iterate through each timestep
                for i = 1:length(x)

                    % Wait for the time to move on
                    while (toc(time_watch) < t(i)) && self.real_time,end

                    % Update the dispaly
                    self.redraw(x(:,i));
                    self.add_data(data(:,1:i))
                end
                
                % For information's sake, display the time of the output
                toc(time_watch)
                
            else
                % Update the plots only
                self.add_data(t,data)
            end
        end
    end
end

