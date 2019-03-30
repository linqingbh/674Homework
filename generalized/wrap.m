function angle = wrap(angle,boundary)
    if ~exist('boundary','var') || isempty(boundary), boundary = pi; end
    while (angle < min([boundary-2*pi,boundary])) || (angle > max([boundary-2*pi,boundary]))
        if (angle < min([boundary-2*pi,boundary]))
            angle = angle + 2*pi;
        elseif (angle > max([boundary-2*pi,boundary]))
            angle = angle - 2*pi;
        end
    end
end