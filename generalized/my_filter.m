classdef my_filter < handle
    properties
        system
        solver = @rk4
    end
    
    methods
        function [self,out_signal] = my_filter(n,wn,in_signal,time,solver)
            
            if exist('solver','var'),self.solver = solver;end
            if length(n)==1,n=ones(size(in_signal(:,1))).*n;end
            if length(wn)==1,wn=ones(size(in_signal(:,1))).*wn;end
            
            out_signal = zeros(size(in_signal));
            out_signal(:,1) = in_signal(:,1);

            
            for i = 1:length(in_signal(:,1))
                initial = [in_signal(i,1);zeros(n(i)-1,1)];
                if n >= 1
                    [A,B,C,D] = butter(n(i),2*pi*wn(i),'s');
                    self.system{i} = system_solver(A,B,C,D,initial,time(1));
                else
                    self.system{i}.propigate = @(in,~) in;
                end
                if length(in_signal(1,:)) > 1
                    for j = 2:length(in_signal)
                        if length(time) > 1
                            out_signal(i,j) = self.filter_next(in_signal(i,j),time(j));
                        else
                            out_signal(i,j) = self.filter_next(in_signal(i,j));
                        end
                    end
                end
            end
            
        end
        
        function out = filter_next(self,in,t)
            out = zeros(size(in));
            for i = 1:length(in(:,1))
                for j = 1:length(in(1,:))
                    out(i,j) = self.system{i}.propigate(in(i,j),t);
                end
            end
        end
    end
end