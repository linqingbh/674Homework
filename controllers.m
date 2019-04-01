classdef controllers < handle
    % Add ability to swap controllers a set times
    
    properties
        % General to pass to other functions
        core
        
        % Functions
        get_equilibrium
        
        % States
        t = 0
        d_indexes
        r_in_indexes
        r_out_indexes
        u_indexes
        x_indexes
        
        % Param
        K
        u_e
        x_e
        y_r_e
        
        % Settings
        type
        r_sat_lim
        u_sat_lim
        windup_limit
        r_is_angle
        x_is_angle
        error
        intigrator_correction
        i_indexes
        
        % Controller Specifc
        t_vec
        plan
        prefilter
        compensator
        
        % Defaults
        count = 0;
        cascade = [];
    end
    properties (Constant) % Types of Controllers
        PID = 'PID';
        FSF = 'Full State Feedback';
        LS = 'Loopshapping';
        OL = 'Open Loop';
    end
    
    methods
        % Class Initializer -----------------------------------------------
        function self = controllers(control,core)
            
            % Unapck
            param = core.param;
            functions = core.functions;

            % General to pass to other functions
            self.core = core;
            
            % Functions
            self.get_equilibrium = functions.get_equilibrium;
            
            % States
            self.d_indexes = get_indexes(param.d_names,control.d_names);
            self.x_indexes = get_indexes(param.x_names,control.x_names);
            self.r_in_indexes = get_indexes(param.r_names,control.r_names);
            self.r_out_indexes = get_indexes(param.r_names,control.u_names);
            self.u_indexes = get_indexes(param.u_names,control.u_names);
            self.i_indexes = get_indexes(control.r_names,control.i_names);
%             self.x_indexes = control.x_indexes;
            
            % Param
            self.K = control.K;
            [self.u_e,self.x_e,self.y_r_e] = self.get_equilibrium(param.y_r_0,self.core);
            self.u_e = self.u_e(self.u_indexes);
            self.x_e = self.x_e(self.x_indexes);
            self.y_r_e = self.y_r_e(self.r_in_indexes);
            
            % Settigs
            self.type = control.type;
            self.r_sat_lim = control.r_sat_lim;
            self.u_sat_lim = control.u_sat_lim;
            self.windup_limit = control.windup_limit;
            self.r_is_angle = param.r_is_angle(self.r_in_indexes);
            self.x_is_angle = param.x_is_angle(self.x_indexes);
            self.error = zeros(length(control.r_names),1);
            self.intigrator_correction = zeros(length(control.r_names),1);
            
            % Controller Specific
            switch self.type
                case self.OL
                    self.t_vec = control.t_vec;
                    self.plan = control.plan;
                case self.PID
                case self.FSF
                    [self.K.K,self.K.k_r,self.K.I] = controllers.get_FSF_gains(control.t_r,control.zeta,self.i_indexes,control.poles,param.A(self.x_indexes,self.x_indexes),param.B(self.x_indexes,self.u_indexes),param.C(self.r_in_indexes,self.x_indexes));
                case self.LS
                    self.prefilter = control.prefilter;
                    self.compensator = control.compensator;
            end
        end
         
        % PID -------------------------------------------------------------
        function u = execute_PID(self,u_P,u_I,u_D)
            u = -self.K.D.*u_D + self.K.P.*u_P + self.K.I.*u_I;
        end
        
        % Full State Feadback ---------------------------------------------
        function u = execute_FSF(self,x,r,sum_of_error)
            if isempty(sum_of_error),sum_of_error = 0;self.K.I = 0;end
            u = -self.K.K*(x-self.x_e) + self.K.k_r*(r-self.y_r_e) - self.K.I*sum_of_error;
        end
        
        % Loopshaping -----------------------------------------------------
        function u = execute_LS(self,zc)
            u = self.K.C*zc;
        end
        
        % General ---------------------------------------------------------
        function saturation_anti_windup(self,u_unsat,u_sat)
            if self.K.I ~= 0
                self.intigrator_correction = self.intigrator_correction + self.K.I^(-1)*(u_unsat - u_sat);
            end
        end
        
        function derivative_anti_windup(self,velocity,dt)
            self.error((abs(velocity) >= self.windup_limit),end) = 0;
        end
        
        % Main ------------------------------------------------------------
        function [u,r_out] = control(self,x_in,y_r_in,y_r_dot_in,r_in,d,t)

            % Unpack
            dt = t - self.t;self.t = t;
            r = saturate(r_in(self.r_in_indexes),self.r_sat_lim+y_r_in(self.r_in_indexes));
            d = d(self.d_indexes);
            if isempty(d),d = 0;end
            y_r = y_r_in(self.r_in_indexes);
            y_r_dot = y_r_dot_in(self.r_in_indexes);
            self.error = [self.error,self.get_error(y_r,r,self.r_is_angle)];
            
            
            % Anti-Windup
            self.derivative_anti_windup(y_r_dot,dt);
            
            sum_of_error = dt*trapz(self.error,2) - self.intigrator_correction;
            
            % Execute Controller
            switch self.type
                case self.OL
                    u = self.plan(:,find(self.t_vec>=t,1));
                case self.PID
                    u_P = self.error(end);
                    u_I = sum_of_error;
                    u_D = y_r_dot;
                    u = self.execute_PID(u_P,u_I,u_D);
                case self.FSF
                    x = x_in(self.x_indexes);
                    u = self.execute_FSF(x,r,sum_of_error(self.i_indexes));
                case self.LS
                    r_filtered = self.prefilter.filter_next(r,t);
                    u = self.compensator.propigate(r_filtered,t);
            end
            
            % Add Equilibrium
            [u_equilibrium,~,y_r_equilibrium] = self.get_equilibrium(y_r,self.core);
            u_e = [u_equilibrium(self.u_indexes);y_r_equilibrium(self.r_out_indexes)];
            u_unsat = u + u_e - d;

            % Saturation & Anti-Windup
            u_sat = saturate(u_unsat,self.u_sat_lim);
%             self.saturation_anti_windup(u_unsat,u_sat)
            u = u_sat;
            
            % Cascaded controller
            r_out = r_in;
            if ~isempty(self.cascade)
                r_out(self.r_out_indexes) = u;
                [u,r_out] = self.cascade.control(x_in,y_r_in,y_r_dot_in,r_out,d,t);
            end
        end
    end
    
    % Static Functions ----------------------------------------------------
    methods (Static)
        function e = get_error(actual,commanded,is_angle)
            if isempty(is_angle),is_angle=false(length(commanded(:,1)));end
                
            e = zeros(size(actual));
            for i = 1:length(is_angle)
                for j = 1:length(actual(1,:))
                    if length(commanded(1,:))==1,k=1;else,k=j;end
                    if is_angle(i)
                        e_y_r = [cos(actual(i,j));sin(actual(i,j));0];
                        e_r = [cos(commanded(i,k));sin(commanded(i,k));0];
                        direction = cross(e_y_r,e_r);
                        try
                            e(i,j) = sign(direction(3))*atan2(sqrt(norm(e_y_r)^2*norm(e_r)^2-dot(e_y_r,e_r)^2),dot(e_y_r,e_r));
                        catch
                            e(i,j) = commanded(i,k) - actual(i,j);
                        end
                    else
                        e(i,j) = commanded(i,k) - actual(i,j);
                    end
                end
            end
        end
        
        % Gains Calculators -----------------------------------------------
        function K = get_PID_gains()
        end
        
        function [K,k_r,k_i] = get_FSF_gains(t_r,zeta,i_indexes,additional_poles,A,B,C_r)
            w_n = 2.2./t_r;

%             poles = eig(A);

            poles = [];
            for i = 1:length(t_r)
                Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
                poles = [poles;roots(Delta)];
            end
            poles = [poles;additional_poles];

%             poles = -abs(real(eig(A)));
%             
%             for i = 1:length(poles)
%                 if ismember(poles(i),poles([1:i-1,i+1:end]))
%                     j = find(poles==poles(i));
%                     poles(j) = poles(j).*(1+0.1*[-1;1]);
%                 end
%             end
%             
%             poles = poles*20;
%             x_indexes = 1:length(A);
%             while length(A)+length(i_indexes)>length(poles)
%                 possible_states = intersect(find(vecnorm(C_r)==0),find(vecnorm(B,2,2)==0));
%                 if isempty(possible_states)
%                     possible_states = find(vecnorm(B,2,2)==0);
%                 end
%                 indexes = 1:length(A);
%                 indexes(possible_states) = [];
%                 [~,i] = min(vecnorm(A(indexes,possible_states)));
%                 A(possible_states(i),:) = [];
%                 A(:,possible_states(i)) = [];
%                 B(possible_states(i),:) = [];
%                 C_r(:,possible_states(i)) = [];
%                 x_indexes(possible_states(i)) = [];
%             end


%             indexes = 1:5;
%             remove = combnk(indexes,1);
%             for i = 1:length(remove)
%                 indexes2 = indexes;
%                 indexes2(remove(i,:))= [];
%                 A2 = A(indexes2,indexes2);
%                 B2 = B(indexes2,:);
%                 C2 = C_r(:,indexes2);
% 
%                 try
% %                     controllers.is_controllable(A2,B2,C2)
%                     
%                     A_aug = [A2,zeros(length(A2),length(i_indexes));-C2(i_indexes,:),zeros(length(i_indexes))];
%                     B_aug = [B2;zeros(length(i_indexes),length(B2(1,:)))];
% 
%                     K = place(A_aug,B_aug,poles);
%                     
%                     disp(remove(i,:))
%                 end
%             end
%             
%             k_i = zeros(length(i_indexes));
%             for i = fliplr(1:length(i_indexes))
%                 k_i(:,i_indexes(i)) = K(:,end);
%                 K = K(:,1:end-1);
%             end
%             
%             k_r = -1./(C2*((A2-B2*K)^-1)*B2);

            A_aug = [A,zeros(length(A),length(i_indexes));
                -C_r(i_indexes,:),eye(length(i_indexes))];
            B_aug = [B;zeros(length(i_indexes),length(B(1,:)))];
%             
%             figure(4),clf
%             hold on
%             plot(poles,'.k')
%             plot(eig(A_aug),'.r')
%             grid on
%             hold off
            
            K = place(A_aug,B_aug,poles);
            
            k_i = zeros(length(i_indexes));
            for i = fliplr(1:length(i_indexes))
                k_i(:,i_indexes(i)) = K(:,end);
                K = K(:,1:end-1);
            end
            k_r = -inv(C_r*(inv(A-B*K))*B);
            
%             k_r = zeros(length(C_r(:,1)),1);
%             for i = 1:length(C_r(:,1))
%                 k_r(i) = -1./(C_r(i,:)*((A-B(:,i)*K(i,:))^-1)*B(:,i));
%             end
%             k_r = diag(k_r);
%             k_r = zeros(length(C_r(:,1)));
        end
        
        function is_controllable(A,B,C)
            
            [~,~,~,~,k]=ctrbf(A,B,C);
            
            if sum(k) ~= length(A)
                unc = length(A) - sum(k);
                error('System not controllable! There are ' + string(unc) + ' uncontrollable states.')
            end
        end
        
        function K = get_LS_gains()
        end
    end
end

