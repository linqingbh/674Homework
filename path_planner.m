classdef path_planner < handle
    properties
        collision_step
        margin_of_saftey
        p_d
        distance
        world
        W
        G
        param
        x_indexes
        pe
        core
        R
        plot_handles = []
        max_iterations
        type
        cost
        plot
        cost_map
        update = true;
        circle
    end
    
    properties (Constant)
        p2p = "Point 2 Point"
        cover = "Explore Map Evenly"
    end
    
    methods
        function self = path_planner(plan,core)
            self.core = core;
            self.collision_step = plan.collision_step;
            self.margin_of_saftey = plan.margin_of_saftey;
            self.p_d = -plan.height;
            self.distance = plan.distance;
            self.x_indexes = get_indexes(core.param.x_names,plan.x_names);
            self.world = core.param.world;
            self.R = core.param.fillet_radius;
            self.max_iterations = plan.max_iterations;
            self.type = plan.type;
            self.plot = plan.plot;
            self.circle = plan.circle;
            
            switch plan.type
                case path_planner.p2p
                    self.cost = @self.min_distance;
                case path_planner.cover
                    self.cost_map = zeros(self.world.edges(2)-self.world.edges(1),self.world.edges(4)-self.world.edges(3));
                    self.cost_map(1:400,:) = 10;
                    self.cost_map(1200:1600,:) = 10;
                    self.cost_map(:,1:400) = 10;
                    self.cost_map(:,1200:1600) = 10;
                    self.cost = @self.min_visits;
            end
        end
        
        function cost = min_visits(self,~,pe,parent_cost,d2s)
            n = ceil(pe(1)-self.world.edges(1));
            e = ceil(pe(2)-self.world.edges(3));
            if ((n>0) && (n <= size(self.cost_map,1))) && ((e>0) &&(e <= size(self.cost_map,2)))
                cost = (self.cost_map(n,e)+parent_cost)/d2s; % add per leg
            else
                cost = Inf;
            end
        end
        
        function cost = min_distance(~,L,~,parent_cost,~)
            cost = L + parent_cost;
        end
        
        function update_cost(self,p)
            n = ceil(p(1)-self.world.edges(1));
            e = ceil(p(2)-self.world.edges(3));
            for i = 1:length(self.cost_map)
                for j = 1:length(self.cost_map)
                    d = sqrt((n-i)^2+(e-j)^2);
                    if d < self.circle
                        self.cost_map(i,j) = self.cost_map(i,j)+1;
                    end
                end
            end
        end
        
        function [W,G] = execute_RRT(self,world,ps,pe)
            [W,G] = self.plan_RRT(world,ps,pe);
        end
        
        function [W,G] = plan_RRT(self,world,ps,pe)
            G(1).V = ps(1:3);
            G(1).E = [];
            G(1).chi = ps(4);
            G(1).path = ps(1:3);
            G(1).C = 0;
            G(1).C2end = Inf;
            G(1).d2s = 0;
            
            figure(self.core.param.animation_fig)
            hold on
            if self.plot,self.plot_handles(end+1) = plot3([ps(2),pe(2)],[ps(1),pe(1)],[-ps(3),-pe(3)],'.k','MarkerSize',20);end
            if self.plot,view([-90,90]),end
            if self.plot,drawnow,end
            while length(G) == 1
                for i = 1:self.max_iterations
    %                 disp(i)
                    p = [rand(2,1).*(world.edges([2,4]).'-world.edges([1,3]).') + world.edges([1,3]).';self.p_d];
                    if self.plot,self.plot_handles(end+1) = plot3(p(2),p(1),-p(3),'.k','MarkerSize',10);end
                    if self.plot,drawnow,end
                    distances = vecnorm(p-[G.V]);
                    nearest = find(distances==min(distances),1);
                    v_star = G(nearest).V;
                    chi_star = G(nearest).chi;
                    v_plus = (p-v_star)/norm(p-v_star)*self.distance+v_star;
                    [path,~,~,~,~,~,q,~,~,~,L] = path_manager.calculate_dubins(v_star,chi_star,self.R,v_plus);
                    if ~self.collision(path,world)
                        G(end+1).V = v_plus; %#ok<AGROW>
                        G(end).E = nearest;
                        G(end).chi = angle_between([1;0;0],q);
                        G(end).path = path;
                        G(end).d2s = G(G(end).E).d2s + 1;
                        G(end).C = self.cost(L,v_plus,G(G(end).E).C,G(end).d2s);
                        G(end).C2end = Inf;

                        if self.plot,self.plot_handles(end+1) = plot3(path(2,:),path(1,:),-path(3,:),'-c','LineWidth',1.5);end
                        if self.plot,drawnow,end
                    end
                    if norm(pe-G(end).V)<=self.distance && self.type == path_planner.p2p
                        try
                            [path_end,~,~,~,~,~,q_end,~,~,~,L] = path_manager.calculate_dubins(G(end).V,G(end).chi,self.R,pe);
                            if  ~self.collision(path_end,world)
                                G(end+1).V = pe; %#ok<AGROW>
                                G(end).E = length(G) - 1;
                                G(end).chi = angle_between([1;0;0],q_end);
                                G(end).path = path_end;
                                G(end).d2s = G(G(end).E).d2s + 1;
                                G(end).C = G(G(end).E).C + self.cost(L,pe,G(end).d2s);
                                G(end).C2end = G(end).C;
                                if self.plot,self.plot_handles(end+1) = plot3(path_end(2,:),path_end(1,:),-path_end(3,:),'-c','LineWidth',1.5);end
                                if self.plot,drawnow,end
                            end
                        catch
                            [path_end,~,~,~,~,~,q_end,~,~,~,L] = path_manager.calculate_dubins(G(G(end).E).V,G(G(end).E).chi,self.R,pe);
                            if  ~self.collision(path_end,world)
                                G(end+1).V = pe; %#ok<AGROW>
                                G(end).E = G(end-1).E;
                                G(end).chi = angle_between([1;0;0],q_end);
                                G(end).path = path_end;
                                G(end).d2s = G(G(end-1).E).d2s + 1;
                                G(end).C = G(G(end).E).C + self.cost(L,pe,G(end).d2s);
                                G(end).C2end = G(end-1).C;
                                if self.plot,self.plot_handles(end+1) = plot3(path_end(2,:),path_end(1,:),-path_end(3,:),'-c','LineWidth',1.5);end
                                if self.plot,drawnow,end
                            end
                        end
                    end
                end

                if length(G) > 1
                    if self.type ==  path_planner.cover
                        for i = 2:length(G)
                            G(i).C2end = G(i).C;
                        end
                    end

                    branch_end = find([G(2:end).C2end]==min([G(2:end).C2end]),1)+1;
                    W = [G(branch_end).V;G(branch_end).chi];
                    last_index = G(branch_end).E;
                    choosen_path = G(branch_end).path;
                    while ~ismember(ps.',W.','rows')
                        W = [[G(last_index).V;G(last_index).chi],W];
                        choosen_path = G(last_index).path;
                        self.update_cost(G(last_index).V);
                        last_index = G(last_index).E;
                    end
                    if self.plot,self.plot_handles(end+1) = plot3(choosen_path(2,:),choosen_path(1,:),-choosen_path(3,:),'--g','LineWidth',2);end
                    W = self.smooth(W);
                    disp("Paused: press any key to continue...")
                    pause
                    if self.plot,view(self.core.settings.view),end
                    if self.plot,delete(self.plot_handles);self.plot_handles=[];end
                end
            end
        end
        
        function Ws = smooth(self,W)
            Ws = W(:,1);
            last_added = 1;
            skip_this_time = [];
            total_path = {};
            timer = tic;
            while ~ismember(W(1:3,end).',Ws(1:3,:).','rows')
                for i = fliplr(2:length(W(1,:)))
                    if (i <= last_added(end)) && ~ismember(i,skip_this_time)
                        Ws = Ws(:,1:end-1);
                        skip_this_time = [skip_this_time,last_added(end)]; %#ok<AGROW>
                        last_added = last_added(1:end-1);
                        total_path = total_path(1:end-1);
                        break
                    end
                    if norm(Ws(1:3,end)-W(1:3,i)) >= self.distance-0.001
                        [path,~,~,~,~,~,q] = path_manager.calculate_dubins(Ws(1:3,end),Ws(4,end),self.R,W(1:3,i));
                        if ~self.collision(path,self.world) && ~ismember(i,skip_this_time)
                            chi = angle_between([1;0;0],q);
                            Ws = [Ws,[W(1:3,i);chi]]; %#ok<AGROW>
                            last_added = [last_added,i]; %#ok<AGROW>
                            skip_this_time = [];
                            total_path{end+1} = path; %#ok<AGROW>
                            break
                        end
                    end
                end
                if toc(timer)>10
                    Ws = W;
                    break
                end
            end
            combined_path = cell2mat(total_path);
            try
                self.plot_handles(end+1) = plot3(combined_path(2,:),combined_path(1,:),-combined_path(3,:),'-g','LineWidth',3);
            catch
                Ws = W;
                self.plot_handles(end+1) = plot3(Ws(2,:),Ws(1,:),-Ws(3,:),'-g','LineWidth',3);
            end
            drawnow
        end
        
        function is_collision = collision(self,vector,world)
            for i = 2:length(vector(1,:))
                v1 = vector(:,i-1);
                v2 = vector(:,i);
                line = v2-v1;
                q = line/norm(line);
                step = q*self.collision_step;

                line_segment = [0;0;0];
                
                while norm(line) >= norm(line_segment)
                    check_point = line_segment+v1;

                    for j = 1:length(world.obsticals)
                        if    (check_point(1) <= (max(world.obsticals(j).corners(1,:)))+self.margin_of_saftey)...
                           && (check_point(1) >= (min(world.obsticals(j).corners(1,:)))-self.margin_of_saftey)...
                           && (check_point(2) <= (max(world.obsticals(j).corners(2,:)))+self.margin_of_saftey)...
                           && (check_point(2) >= (min(world.obsticals(j).corners(2,:)))-self.margin_of_saftey)...
                           && (check_point(3) <= (max(world.obsticals(j).corners(3,:)))+self.margin_of_saftey)...
                           && (check_point(3) >= (min(world.obsticals(j).corners(3,:)))-self.margin_of_saftey)
                            is_collision = true;
                            break
                        else
                            is_collision = false;
                        end
                        
                        if (vector(1,end) <= (max(world.obsticals(j).corners(1,:)))+self.R)...
                        && (vector(1,end) >= (min(world.obsticals(j).corners(1,:)))-self.R)...
                        && (vector(2,end) <= (max(world.obsticals(j).corners(2,:)))+self.R)...
                        && (vector(2,end) >= (min(world.obsticals(j).corners(2,:)))-self.R)...
                        && (vector(3,end) <= (max(world.obsticals(j).corners(3,:)))+self.R)...
                        && (vector(3,end) >= (min(world.obsticals(j).corners(3,:)))-self.R)
                            is_collision = true;
                            break
                        end
                    end

                    if is_collision
                        break
                    end

                    line_segment = line_segment + step;
                end
                if is_collision
                    break
                end
            end
            
        end
        
        function [W,G] = plan(self,x,pe)

            if self.update
                ps = x(self.x_indexes);
                self.pe = pe;
                [W,G] = self.execute_RRT(self.world,ps,self.pe);
                self.W = W;
                self.G = G;
                self.update = false;
            else
                W = self.W;
                G = self.G;
            end
        end
    end
end