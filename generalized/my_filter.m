classdef my_filter < handle
    properties
        system
        last_input
        last_output
        update_every_step = true
        x
    end
    
    methods
        function [self,out_signal] = my_filter(n,wn,in_signal)
            
            if length(n)==1,n=ones(size(in_signal(:,1))).*n;end
            if length(wn)==1,wn=ones(size(in_signal(:,1))).*wn;end
            
            out_signal = zeros(size(in_signal));
            out_signal(:,1) = in_signal(:,1);
            self.last_input = in_signal(:,1);
            self.last_output = in_signal(:,1);
            
            for i = 1:length(in_signal(:,1))
                if n > 0
                    [self.system(i).A,self.system(i).B,self.system(i).C,self.system(i).D] = butter(n(i),wn(i));
                    self.system(i).x = zeros(n(i),1);
                else
                    self.system(i).A = 0;
                    self.system(i).B = 0;
                    self.system(i).C = 0;
                    self.system(i).D = 1;
                    self.system(i).x = 0;
                end
                if length(in_signal(1,:)) > 1
                    for j = 2:length(in_signal)
                        if length(time) > 1
                            out_signal(i,j) = self.filter_next(in_signal(i,j));
                        else
                            out_signal(i,j) = self.filter_next(in_signal(i,j));
                        end
                    end
                end
            end
        end
        
        function out = filter_next(self,in)
            out = zeros(size(in));
            for i = 1:length(in(:,1))
                for j = 1:length(in(1,:))
                    if (self.last_input(i) ~= in(i,j)) || self.update_every_step
                        out(i,j) = self.system(i).C*self.system(i).x + self.system(i).D*in(i,j);
                        self.system(i).x = self.system(i).A*self.system(i).x + self.system(i).B*in(i,j);
                        self.last_input(i) = in(i,j);
                        self.last_output(i) = out(i,j);
                    else
                        out(i,j) = self.last_output(i);
                    end
                end
            end
        end
    end
end