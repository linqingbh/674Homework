function my_unpack(structure)
    names = fieldnames(structure);
    for i = 1:length(names)
        assignin('caller',names{i},structure.(names{i}));
    end
end