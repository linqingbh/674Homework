function angle = angle_between(a1,a2,direction)
    if ~exist('direction','var')
        cross_product = cross(a1,a2);
        angle = sign(cross_product(3))*atan2(norm(cross_product),dot(a1,a2));
    else
        switch direction
            case 'cw'
                angle = bracket(2*pi + bracket(a2) - bracket(a1));
            case 'ccw'
                angle = bracket(2*pi - bracket(a2) + bracket(a1));
        end
    end
end

function out = bracket(angle)
    out = mod(angle,2*pi);
end