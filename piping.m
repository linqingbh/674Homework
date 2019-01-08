classdef piping < handle
    
    properties
        topics
        topic_names
        param
        settings
        functions
    end
    
    methods
        function self = piping()
            self.topics = {};
            self.topic_names = {};
        end
        
        function publish(self,name,data)
            index = strcmp(name,[self.topic_names{1:end}]);
            if any(index)
                self.topics{index}{end+1} = data;
            else
                self.topic_names{end+1} = name;
                self.topics{end+1}{1} = data;
            end
        end
        
        function publish_list(self,name,data)
            for packet = data
                self.publish(name,packet);
            end
        end
        
        function data = subscribe(self,name)
            index = strcmp(name,[self.topic_names{1:end}]);
            if any(index)
                data = self.topics{index}{end};
            else
                error('Subcribed to a unpublished topic.')
            end
        end
        
        function data = subscribe_history(self,name)
            index = strcmp(name,[self.topic_names{1:end}]);
            if any(index)
                data = cell2mat(self.topics{index});
            else
                error('Subcribed to a unpublished topic.')
            end
        end
        
        function data = subscribe_specific(self,name,history_index)
            index = strcmp(name,[self.topic_names{1:end}]);
            if any(index)
                data = self.topics{index}{history_index};
            else
                error('Subcribed to a unpublished topic.')
            end
        end
    end
end

