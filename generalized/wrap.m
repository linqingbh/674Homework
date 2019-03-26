function angle = wrap(angle,boundary)
    if ~exist('boundary','var') || isempty(boundary), boundary = pi; end
    while (angle < min([-boundary,boundary])) || (angle > max([-boundary,boundary]))
        if (angle < min([-boundary,boundary]))
            angle = angle + 2*pi;
        elseif (angle > max([-boundary,boundary]))
            angle = angle - 2*pi;
        end
    end
end