classdef system_solver < handle
    properties
        A
        B
        C
        D
        I
        y
        y_dot
        x
        x_dot
        time = 0;
        solver = @rk4
        
    end
    
    methods
        function self = system_solver(varargin)
            if isa(varargin{1},'tf')
                [num,den] = tfdata(varargin{1},'v');
                [self.A,self.B,self.C,self.D] = tf2ss(num,den);
                if nargin>=2,self.x = varargin{2};
                else, self.x = zeros(length(self.A),1);end
                if nargin>=3,self.time = varargin{3};end
                if nargin>=4,self.solver = varargin{4};end
            else
                self.A = varargin{1};
                self.B = varargin{2};
                self.C = varargin{3};
                self.D = varargin{4};
                if nargin>=5,self.x = varargin{5};
                else, self.x = zeros(length(self.A),1);end
                if nargin>=6,self.time = varargin{6};end
                if nargin>=7,self.solver = varargin{7};end
            end
            self.I = eye(length(self.x));
            self.x_dot = self.A*self.x;
            self.y = self.C*self.x;
            self.y_dot = self.C*self.x_dot;
        end
        
        function [y,x,x_dot,y_dot] = propigate(self,u,t)
            if isempty(t)
                t_vec = [0,self.time];
            else
                t_vec = [self.time,t];
            end
            
            if t_vec(2) > t_vec(1)
                ode = @(t,x) self.A*x+self.B*u;
                [x,x_dot] = self.solver(ode,t_vec,self.x);
                self.time = t;
            else
                x = self.x;
                x_dot = self.x_dot;
            end
            y = self.C*x + self.D*u;
            y_dot = self.C*x_dot;
            
            self.y = x;
            self.y_dot = x_dot;
            self.x = x;
            self.x_dot = x_dot;
        end
    end
end