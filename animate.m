classdef animate < handle
    
    properties
        % Display Window Info
        window
        view
        labels
        gph_per_img

        % Playback Info
        playback_rate
        animation
        plot
        active_fig
        var_indexes

        % Handle to the function that draws the simulation
        get_drawing

        % General Parameters for passing information
        core
        param
        settings
        
        % Handles
        poly_handle
        lines_handle
        points_handle
        vecs_handle
        plot_handles

        % Settings
        x_is_angle
        r_is_angle
    end
    
    methods
        function [self,animation_fig] = animate(core)
            
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
            self.playback_rate = settings.playback_rate;
            self.animation = settings.animation;
            self.plot = settings.plot;
            self.active_fig = settings.active_fig;
            
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
            [var_names,var_to_plot_names] = self.get_names();
            self.var_indexes = get_indexes(var_names,var_to_plot_names);

            % Initialize Figures ------------------------------------------
            [x,data,t] = self.get_data(1);
            
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
                [poly,poly_colors,lines,line_colors,points,point_colors,vecs,vec_colors] = self.get_drawing(x,self.core,true);
                
                % Plot
                animation_fig = figure(1);clf
                animation_fig.WindowStyle = 'docked';
                hold on
                for i = 1:length(lines)
                    self.lines_handle(i) = plot3(lines{i}(1,:),lines{i}(2,:),lines{i}(3,:),line_colors{i},'linewidth',1.5);
                end
                for i = 1:length(points)
                    self.points_handle(i) = plot3(points{i}(1,:),points{i}(2,:),points{i}(3,:),point_colors{i},'MarkerSize',20);
                end
                for i = 1:length(vecs)
                    self.vecs_handle(i) = quiver3(vecs{i}(1,1),vecs{i}(2,1),vecs{i}(3,1),vecs{i}(1,2),vecs{i}(2,2),vecs{i}(3,2),vec_colors{i},'linewidth',1.5);
                end
                for i = 1:length(poly)
                    self.poly_handle(i) = fill3(poly{i}(1,:),poly{i}(2,:),poly{i}(3,:),poly_colors{i});
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
                    fig = figure(k);clf
                    fig.WindowStyle = 'docked';
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
                            l = l+1;
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
            [poly,~,lines,~,points,~,vecs] = self.get_drawing(x,self.core,false);          
            
            % Update the chart data.
            for i = 1:length(lines)
                set(self.lines_handle(i),...
                    'XData',lines{i}(1,:),...
                    'YData',lines{i}(2,:),...
                    'ZData',lines{i}(3,:));
            end
            for i = 1:length(points)
                set(self.points_handle(i),...
                    'XData',points{i}(1,:),...
                    'YData',points{i}(2,:),...
                    'ZData',points{i}(3,:));
            end
            for i = 1:length(vecs)
                set(self.vecs_handle(i),...
                    'UData',vecs{i}(1,2),...
                    'VData',vecs{i}(2,2),...
                    'WData',vecs{i}(3,2),...
                    'XData',vecs{i}(1,1),...
                    'YData',vecs{i}(2,1),...
                    'ZData',vecs{i}(3,1));
            end
            for i = 1:length(poly)
                set(self.poly_handle(i),...
                    'X',poly{i}(1,:),...
                    'Y',poly{i}(2,:),...
                    'Z',poly{i}(3,:))
            end
        end
        
        function add_data(self,t,data)
            if self.plot
                % Plot Data Figure
                for i = 1:length(self.plot_handles)
                    set(self.plot_handles(i),'XData',t,'YData',data(i,:));
                end
            end
        end
        
        function [x,data,t] = get_data(self,time_indexes)
            data = [self.core.subscribe_history('r');
                    self.core.subscribe_history('d');
                    self.core.subscribe_history('x');
                    self.core.subscribe_history('x_dot');
                    self.core.subscribe_history('z');
                    self.core.subscribe_history('z_f');
                    self.core.subscribe_history('z_hat');
                    self.core.subscribe_history('y_m');
                    self.core.subscribe_history('y_m_hat');
                    self.core.subscribe_history('x_hat');
                    self.core.subscribe_history('d_hat');
                    controllers.get_error(self.core.subscribe_history('x_hat'),self.core.subscribe_history('x'),self.x_is_angle);
                    self.core.subscribe_history('y_r');
                    self.core.subscribe_history('y_r_hat');
                    controllers.get_error(self.core.subscribe_history('y_r'),self.core.subscribe_history('r'),self.r_is_angle);
                    self.core.subscribe_history('y_r_dot');
                    self.core.subscribe_history('y_r_dot_hat');
                    self.core.subscribe_history('u')];
            
            if exist('time_indexes','var')
                t = self.core.subscribe_specific('t',time_indexes);
                x = self.core.subscribe_specific('x',time_indexes);
                data = data(self.var_indexes,time_indexes);
            else
                t = self.core.subscribe_history('t');
                x = self.core.subscribe_history('x');
                data = data(self.var_indexes,:);
            end
        end
        
        function [var_names,plot_names] = get_names(self)
            var_names = ["r_{" + self.param.r_names + "}";
                         self.param.d_names;
                         self.param.x_names;
                         self.param.x_names + "_{dot}";
                         self.param.z_names;
                         "z_f_{" + self.param.z_names + "}";
                         "z_hat_{" + self.param.z_names + "}";
                         "y_m_{" + self.param.m_names + "}";
                         "y_m_hat_{" + self.param.m_names + "}";
                         "x_hat_{" + self.param.x_names + "}";
                         "d_hat_{" + self.param.d_names + "}";
                         "x_hat_e_{" + self.param.x_names + "}";
                         "y_r_{" + self.param.r_names + "}";
                         "y_r_hat_{" + self.param.r_names + "}";
                         "y_r_e_{" + self.param.r_names + "}";
                         "y_r_dot_{" + self.param.r_names + "}";
                         "y_r_dot_hat_{" + self.param.r_names + "}";
                         self.param.u_names];
            plot_names = [];
            for i = 1:length(self.settings.plot_names)
                plot_names = [plot_names;...
                    self.settings.plot_names{i}(2:end).']; %#ok<AGROW>
            end
        end
        
        function play(self)
            
            disp("Animator is getting data.")
            
            % Get time
            [x,data,t] = self.get_data();
            if self.animation
                
                disp("Animation Starting.")
                
                time_watch = tic; % Timer that iterates through each image
                
                % Iterate through each timestep
                while any(t>=toc(time_watch)/self.playback_rate)
                    j = find(t>=toc(time_watch)/self.playback_rate,1);
                    
                    % Update the dispaly
                    self.redraw(x(:,j));
                    self.add_data(t(1:j),data(:,1:j))
                    drawnow limitrate
                end
                
                disp("Animation finished.")
                
                % For information's sake, display the time of the output
                toc(time_watch)
                
            else
                % Update the plots only
                self.add_data(t,data)
                drawnow limitrate
            end
        end
    end
end

