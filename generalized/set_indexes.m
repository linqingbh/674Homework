function output = set_indexes(obj_list,indexes,method,varargin)
    if length(varargin)==1
        for i = 1:length(obj_list)
            varargin{i} = varargin{1};
        end
    end
    for i = 1:length(obj_list)
        output(obj_list(i).(indexes),1) = obj_list(i).(method)(varargin{i}{:}); %#ok<AGROW>
    end
end