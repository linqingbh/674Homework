classdef system_solver < handle
    properties
        A
        B
        C
        D
        I
        x
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
        end
        
        function [y,x,x_dot] = propigate(self,u,t)
            if isempty(t)
                dt = self.time;
            else
                dt = t - self.time;
            end
            ode = @(t,x) self.A*x+self.B*u;
            [x,x_dot] = self.solver(ode,[0,dt],self.x);
            y = self.C*x + self.D*u;
            x = x(:,end);
            x_dot = x_dot(:,end);
            self.x = x;
        end
    end
end