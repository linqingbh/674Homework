function diff = percent_differance(xi,xf)
    if ~any(xi) && ~any(xf)
        diff = 0;
    elseif xi==0
        if length(xi)>1
            diff = norm(xf-xi)/norm(xf);
        else
            diff = (xf-xi)/xf;
        end
    else
        if length(xi)>1
            diff = norm(xf-xi)/norm(xi);
        else
            diff = (xf-xi)/xi;
        end
    end
end