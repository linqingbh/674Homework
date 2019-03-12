function output = set_indexes(obj_list,indexes,method,varargin)
    for i = 1:length(obj_list)
        output(obj_list(i).(indexes)) = obj_list(i).(method)(varargin{i}{:}); %#ok<AGROW>
    end
end