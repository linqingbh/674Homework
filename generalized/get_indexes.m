function indexes = get_indexes(options,names)
    indexes = zeros(size(names));
    for i = 1:length(names)
        if any(strcmp(options,names(i)))
            indexes(i) = find(strcmp(options,names(i)),1);
        else
            indexes(i) = [];
        end
    end
end