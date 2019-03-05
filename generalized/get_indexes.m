function indexes = get_indexes(options,names)
    indexes = zeros(size(names));
    corrector = 0;
    for i = 1:length(names)
        if any(strcmp(options,names(i - corrector)))
            indexes(i - corrector) = find(strcmp(options,names(i - corrector)),1);
        else
            indexes(i - corrector) = [];
            corrector = corrector + 1;
        end
    end
end